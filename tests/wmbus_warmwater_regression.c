#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/data.h"
#include "../include/decoder_util.h"
#include "../include/jsmn.h"

#include "../src/devices/m_bus.c"

static data_t *g_captured_data = NULL;
static const char *const k_ignored_keys[] = {
        "time",
        "mod",
        "freq1",
        "freq2",
        "rssi",
        "snr",
        "noise",
};

static void capture_output(r_device *decoder, data_t *data)
{
    (void)decoder;
    g_captured_data = data;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

static int decode_hex(const char *hex, uint8_t **out, size_t *out_len)
{
    size_t hex_len = strlen(hex);
    if (hex_len % 2 != 0)
        return 0;

    uint8_t *buf = calloc(hex_len / 2, sizeof(*buf));
    if (!buf)
        return 0;

    for (size_t i = 0; i < hex_len; i += 2) {
        int hi = hex_value(hex[i]);
        int lo = hex_value(hex[i + 1]);
        if (hi < 0 || lo < 0) {
            free(buf);
            return 0;
        }
        buf[i / 2] = (uint8_t)((hi << 4) | lo);
    }

    *out = buf;
    *out_len = hex_len / 2;
    return 1;
}

typedef struct {
    char *key;
    char *value;
} json_pair_t;

typedef struct {
    json_pair_t *items;
    size_t count;
    size_t capacity;
} json_object_t;

static void free_json_object(json_object_t *obj)
{
    if (!obj)
        return;

    for (size_t i = 0; i < obj->count; ++i) {
        free(obj->items[i].key);
        free(obj->items[i].value);
    }
    free(obj->items);
    obj->items = NULL;
    obj->count = 0;
    obj->capacity = 0;
}

static char *dup_token_slice(const char *json, jsmntok_t const *token)
{
    size_t len = (size_t)(token->end - token->start);
    char *out = calloc(len + 1, sizeof(*out));
    if (!out)
        return NULL;
    memcpy(out, json + token->start, len);
    out[len] = '\0';
    return out;
}

static int is_ignored_key(const char *key)
{
    for (size_t i = 0; i < sizeof(k_ignored_keys) / sizeof(k_ignored_keys[0]); ++i) {
        if (strcmp(key, k_ignored_keys[i]) == 0)
            return 1;
    }
    return 0;
}

static int parse_json_object(const char *json, json_object_t *out)
{
    memset(out, 0, sizeof(*out));

    jsmn_parser parser;
    jsmn_init(&parser);
    int token_count = jsmn_parse(&parser, json, strlen(json), NULL, 0);
    if (token_count < 0)
        return 0;

    jsmntok_t *tokens = calloc((size_t)token_count, sizeof(*tokens));
    if (!tokens)
        return 0;

    jsmn_init(&parser);
    int parse_result = jsmn_parse(&parser, json, strlen(json), tokens, (size_t)token_count);
    if (parse_result < 0 || tokens[0].type != JSMN_OBJECT) {
        free(tokens);
        return 0;
    }

    size_t pair_count = 0;
    for (int i = 1; i < parse_result; ++i) {
        if (tokens[i].type == JSMN_STRING) {
            char *key = dup_token_slice(json, &tokens[i]);
            if (!key) {
                free(tokens);
                return 0;
            }

            if (i + 1 >= parse_result) {
                free(key);
                free(tokens);
                return 0;
            }

            if (tokens[i + 1].type == JSMN_STRING || tokens[i + 1].type == JSMN_PRIMITIVE) {
                char *value = dup_token_slice(json, &tokens[i + 1]);
                if (!value) {
                    free(key);
                    free(tokens);
                    return 0;
                }

                if (out->count == out->capacity) {
                    size_t new_cap = out->capacity ? out->capacity * 2 : 8;
                    json_pair_t *new_items = realloc(out->items, new_cap * sizeof(*new_items));
                    if (!new_items) {
                        free(key);
                        free(value);
                        free(tokens);
                        return 0;
                    }
                    out->items = new_items;
                    out->capacity = new_cap;
                }

                out->items[out->count].key = key;
                out->items[out->count].value = value;
                out->count++;
                pair_count++;
                ++i;
            }
        }
    }

    free(tokens);
    return 1;
}

static int json_object_compare(const char *actual_json, const char *expected_json)
{
    json_object_t actual = {0};
    json_object_t expected = {0};

    if (!parse_json_object(actual_json, &actual) || !parse_json_object(expected_json, &expected)) {
        fprintf(stderr, "FAIL: JSON parse error\n");
        free_json_object(&actual);
        free_json_object(&expected);
        return 0;
    }

    json_object_t actual_filtered = {0};
    json_object_t expected_filtered = {0};

    for (size_t i = 0; i < actual.count; ++i) {
        if (is_ignored_key(actual.items[i].key))
            continue;
        if (strcmp(actual.items[i].key, "protocol") == 0 || strcmp(actual.items[i].key, "data_length") == 0)
            continue;
        if (actual_filtered.count == actual_filtered.capacity) {
            size_t new_cap = actual_filtered.capacity ? actual_filtered.capacity * 2 : 8;
            json_pair_t *new_items = realloc(actual_filtered.items, new_cap * sizeof(*new_items));
            if (!new_items) {
                free_json_object(&actual);
                free_json_object(&expected);
                free_json_object(&actual_filtered);
                free_json_object(&expected_filtered);
                return 0;
            }
            actual_filtered.items = new_items;
            actual_filtered.capacity = new_cap;
        }
        actual_filtered.items[actual_filtered.count++] = actual.items[i];
        actual.items[i].key = NULL;
        actual.items[i].value = NULL;
    }

    for (size_t i = 0; i < expected.count; ++i) {
        if (is_ignored_key(expected.items[i].key))
            continue;
        if (expected_filtered.count == expected_filtered.capacity) {
            size_t new_cap = expected_filtered.capacity ? expected_filtered.capacity * 2 : 8;
            json_pair_t *new_items = realloc(expected_filtered.items, new_cap * sizeof(*new_items));
            if (!new_items) {
                free_json_object(&actual);
                free_json_object(&expected);
                free_json_object(&actual_filtered);
                free_json_object(&expected_filtered);
                return 0;
            }
            expected_filtered.items = new_items;
            expected_filtered.capacity = new_cap;
        }
        expected_filtered.items[expected_filtered.count++] = expected.items[i];
        expected.items[i].key = NULL;
        expected.items[i].value = NULL;
    }

    for (size_t i = 0; i < expected_filtered.count; ++i) {
        int found = 0;
        for (size_t j = 0; j < actual_filtered.count; ++j) {
            if (strcmp(expected_filtered.items[i].key, actual_filtered.items[j].key) != 0)
                continue;
            if (strcmp(expected_filtered.items[i].value, actual_filtered.items[j].value) != 0) {
                fprintf(stderr,
                        "FAIL: key '%s' mismatch: actual='%s' expected='%s'\n",
                        expected_filtered.items[i].key,
                        actual_filtered.items[j].value,
                        expected_filtered.items[i].value);
                free_json_object(&actual);
                free_json_object(&expected);
                free_json_object(&actual_filtered);
                free_json_object(&expected_filtered);
                return 0;
            }
            found = 1;
            break;
        }
        if (!found) {
            fprintf(stderr, "FAIL: missing key '%s' in actual output\n", expected_filtered.items[i].key);
            free_json_object(&actual);
            free_json_object(&expected);
            free_json_object(&actual_filtered);
            free_json_object(&expected_filtered);
            return 0;
        }
    }

    free_json_object(&actual);
    free_json_object(&expected);
    free_json_object(&actual_filtered);
    free_json_object(&expected_filtered);
    return 1;
}

static int decode_case(const char *name, const char *hex_input, const char *expected_json)
{
    uint8_t *payload = NULL;
    size_t payload_len = 0;
    m_bus_data_t wmbus_data = {0};
    m_bus_block1_t block1 = {0};

    if (!decode_hex(hex_input, &payload, &payload_len)) {
        fprintf(stderr, "FAIL: %s hex decode failed\n", name);
        return 0;
    }

    if (payload_len < 10) {
        fprintf(stderr, "FAIL: %s payload too short for WMBus header\n", name);
        free(payload);
        return 0;
    }

    memcpy(wmbus_data.data, payload, payload_len);
    wmbus_data.length = (unsigned)payload_len;

    block1.L = payload[0];
    block1.C = payload[1];
    m_bus_manuf_decode((uint32_t)(payload[3] << 8 | payload[2]), block1.M_str);
    block1.A_ID = bcd2int(payload[7]) * 1000000 + bcd2int(payload[6]) * 10000 +
            bcd2int(payload[5]) * 100 + bcd2int(payload[4]);
    block1.A_Version = payload[8];
    block1.A_DevType = payload[9];

    r_device *decoder = decoder_create(&m_bus_mode_s, 0);
    if (!decoder) {
        fprintf(stderr, "FAIL: %s decoder_create() failed\n", name);
        free(payload);
        return 0;
    }

    decoder->output_fn = capture_output;
    m_bus_parse_ci(payload + 10, payload_len > 10 ? payload_len - 10 : 0, 10, &block1.block2);
    if (block1.block2.CI != 0x72 && block1.block2.CI != 0x7A && block1.block2.CI != 0x78) {
        fprintf(stderr, "FAIL: %s parsed unsupported CI 0x%02x\n", name, block1.block2.CI);
        free(payload);
        free(decoder);
        return 0;
    }

    m_bus_output_data(decoder, NULL, &wmbus_data, &block1, "S");
    if (!g_captured_data) {
        fprintf(stderr, "FAIL: %s decoder produced no output\n", name);
        free(payload);
        free(decoder);
        return 0;
    }

    char *actual_json = data_print_jsons_dup(g_captured_data);
    if (!actual_json) {
        fprintf(stderr, "FAIL: %s data_print_jsons_dup() returned NULL\n", name);
        data_free(g_captured_data);
        free(payload);
        free(decoder);
        return 0;
    }

    int ok = json_object_compare(actual_json, expected_json);
    if (!ok) {
        fprintf(stderr, "Actual JSON for %s:\n%s\n", name, actual_json);
        fprintf(stderr, "Expected JSON for %s:\n%s\n", name, expected_json);
    }

    free(actual_json);
    data_free(g_captured_data);
    g_captured_data = NULL;
    free(payload);
    free(decoder);
    return ok;
}

int main(void)
{
    static const char *test1_hex = "24446532516873521a067a5a0000008c04134732000082046c5f3801fd0c11046d0a124b39";
    static const char *test1_json =
            "{\"model\":\"Wireless-MBus\",\"mode\":\"S\",\"M\":\"LSE\",\"id\":52736851,\"version\":26,\"type\":6,\"type_string\":\"Warm Water\",\"C\":68,\"data\":\"24446532516873521a067a5a0000008c04134732000082046c5f3801fd0c11046d0a124b39\",\"mic\":\"CRC\",\"CI\":122,\"AC\":90,\"ST\":0,\"CW\":0,\"inst_volume_m1_8\":\"3.247 m3\",\"inst_date_8\":\"26-08-31\",\"inst_timedate_0\":\"26-09-11T18:10:00\"}";

    static const char *test2_hex = "2d446532516873521a067a5b0000000c13053300004c1328120000426c3f3c02bb560000326cffff046d3b154b39";
    static const char *test2_json =
            "{\"model\":\"Wireless-MBus\",\"mode\":\"S\",\"M\":\"LSE\",\"id\":52736851,\"version\":26,\"type\":6,\"type_string\":\"Warm Water\",\"C\":68,\"data\":\"2d446532516873521a067a5b0000000c13053300004c1328120000426c3f3c02bb560000326cffff046d3b154b39\",\"mic\":\"CRC\",\"CI\":122,\"AC\":91,\"ST\":0,\"CW\":0,\"inst_volume_0\":\"3.305 m3\",\"inst_volume_1\":\"1.228 m3\",\"inst_date_1\":\"25-12-31\",\"err_date_0\":\"invalid\",\"inst_timedate_0\":\"26-09-11T21:59:00\"}";

    if (!decode_case("test1", test1_hex, test1_json))
        return 1;
    if (!decode_case("test2", test2_hex, test2_json))
        return 1;

    puts("wmbus warm water JSON regression: OK");
    return 0;
}
