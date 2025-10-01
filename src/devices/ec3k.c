/** @file
    Decoder for EnergyCounter 3000 (ec3k), tested with .

    Copyright (C) 2025 Michael Dreher @nospam2000

    decoding info taken from https://github.com/EmbedME/ec3k_decoder

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */
 
#include "decoder.h"
#include "r_device.h"
#include "rtl_433.h"
#include "bitbuffer.h"
#include <stdint.h>

//#include <stdio.h>
//#include <stdlib.h>
//#include <errno.h>
//#include <string.h>
//#include <time.h>

// --- Configuration ---
static const uint32_t DECODED_PAKET_LEN_BYTES = 41;
// static const uint32_t ec3_sample_rate = 2000000; // Hz, TODO: adapt to actual sample rate
// #define ec3_sample_rate cfg->samp_rate

// static const uint8_t LEVEL_THRESHOLD = 47;
// static const uint8_t LEVEL_MIN = 20;
// static const uint8_t LEVEL_MAX = 70;
static const uint32_t BITTIME_US = 50;

#define PAKET_MIN_BITS 100
#define PAKET_MAX_BITS ((PAKET_MIN_BITS * 1.3) * 3) // longer bit len and stuffing + repetition? TODO: review

// TODO: BITBUF_COLS is 128 which is the number of bytes per column ; check which limits are in bytes and which are in bits

static int ec3_decode_row(r_device *const decoder, const bitrow_t row, const uint16_t row_bits, const pulse_data_t *pulses);
static int ec3k_decode(r_device *decoder, bitbuffer_t *bitbuffer, const pulse_data_t *pulses);

// copied from bitbuffer.c
static inline uint8_t bit_at(const uint8_t *bytes, unsigned bit)
{
    return (uint8_t)(bytes[bit >> 3] >> (7 - (bit & 7)) & 1);
}

static int ec3_decode_row(r_device *const decoder, const bitrow_t row, const uint16_t row_bits, const pulse_data_t *pulses) {
    const uint32_t ec3_sample_rate = pulses->sample_rate;
    const int BITTIME = (((uint64_t)BITTIME_US * ec3_sample_rate) / 1000000LL);
    const int BITTIME_BOUND_LOWER = (((uint64_t)BITTIME_US * ec3_sample_rate) * 9) / (1000000LL * 10);
    const int BITTIME_BOUND_UPPER = (((uint64_t)BITTIME_US * ec3_sample_rate) * 11) / (1000000LL * 10);

    char bitbuffer[1000]; // TODO: review size and replace with constant; check for overflow
    int bufferpos = 0;
    char lastlevel = 0;      // either 0 or 1
    int lastedge = 0;
    for (size_t col = 0; col < row_bits ; col++)
    {
        const uint8_t level = bit_at((const uint8_t*)row, col);

        // edge detection
        if (level != lastlevel)
        {
            if (lastedge >= BITTIME_BOUND_LOWER)
            {
                // seems to be a bit
                bitbuffer[bufferpos++] = 0; // change after a bit time => 0
            }
            else
            {
                // out of sync
                if (bufferpos > PAKET_MIN_BITS)
                {
                    /*
                    printf("Paket len=%i\n", bufferpos);
                    for (i = 0; i < bufferpos; i++) {
                        printf("%i", bitbuffer[i]);
                    }
                    printf("\n");
                    */
                    unsigned char packetbuffer[100]; // TODO: review size and replace with constant; check for overflow
                    int packetpos = 0;
                    unsigned char packet = 0;
                    char onecount = 0;
                    unsigned char recbyte = 0;
                    char recpos = 0;
                    // printf("Descrambled/unstuffed: \n");
                    for (int i = 17; i < bufferpos; i++)
                    {
                        char out = bitbuffer[i];
                        if (i > 17)
                            out = out ^ bitbuffer[i - 17];
                        if (i > 12)
                            out = out ^ bitbuffer[i - 12];

                        if (out)
                        {
                            onecount++;
                            recbyte = recbyte >> 1 | 0x80;
                            recpos++;
                            if ((recpos == 8) && (packet))
                            {
                                recpos = 0;
                                packetbuffer[packetpos++] = recbyte;
                            }
                            // printf("%i", out);
                        }
                        else
                        {
                            if ((onecount < 5))
                            {
                                // bit unstuffing: 0 after less than 5 ones => normal bit, otherwise skip stuffed 0
                                recbyte = recbyte >> 1;
                                recpos++;
                                if ((recpos == 8) && (packet))
                                {
                                    recpos = 0;
                                    packetbuffer[packetpos++] = recbyte;
                                }
                                // printf("%i", out);
                            }

                            // start and end of packet is marked by 6 ones
                            if (onecount == 6)
                            {
                                if (packet && packetpos == DECODED_PAKET_LEN_BYTES)
                                {
                                    // received ec3k packet

                                    const unsigned int id = (packetbuffer[0] & 0x0f) << (8 + 4) | (packetbuffer[1]) << 4 | (packetbuffer[2]) >> 4;
                                    const unsigned int wcurrent = (packetbuffer[15] & 0x0f) << (8 + 4) | (packetbuffer[16]) << 4 | (packetbuffer[17]) >> 4;
                                    uint64_t energy = ((packetbuffer[33] & 0x0f) << (8 + 4) | (packetbuffer[34]) << 4 | (packetbuffer[35]) >> 4);
                                    energy = energy << 28 | packetbuffer[12] << 20 | packetbuffer[13] << 12 | packetbuffer[14] << 4 | (packetbuffer[15] >> 4);

                                    // convert to common units
                                    const double energy_kwh = energy / (1000.0 * 3600.0); // Ws to kWh
                                    const double power_w = wcurrent / 10.0; // 1/10 W to W

                                    /* clang-format off */
                                    data_t *data = data_make(
                                        "model",            "",             DATA_STRING, "EnergyCounter 3000",
                                        "id",               "",             DATA_INT,    id,
                                        // "wcurrent",         "Power",        DATA_INT,    wcurrent,
                                        "power",            "Power",        DATA_DOUBLE, power_w,
                                        // "energy",           "Energy",       DATA_INT,    energy,
                                        "energy",           "Energy",       DATA_DOUBLE, energy_kwh,
                                        NULL);
                                    /* clang-format on */

                                    decoder_output_data(decoder, data);
                                }
                                packet = !packet;
                                recpos = 0;
                                packetpos = 0;
                            }
                            onecount = 0;
                        }

                        // printf("%i", out);
                    }
                    // printf("\n");
                }
                bufferpos = 0;
            }

            lastedge = 0;
        }

        if (lastedge >= BITTIME_BOUND_UPPER)
        {
            lastedge -= BITTIME;
            bitbuffer[bufferpos++] = 1; // no change for a bit time => 1
        }

        lastedge++;
        lastlevel = level;
    }
    return 1;
}


static int ec3k_decode(r_device *decoder, bitbuffer_t *bitbuffer, const pulse_data_t *pulses)
{
    if (bitbuffer->num_rows != 1
            || bitbuffer->bits_per_row[0] < PAKET_MIN_BITS
            || bitbuffer->bits_per_row[0] > PAKET_MAX_BITS) {
        decoder_logf(decoder, 2, __func__, "bit_per_row %u out of range", bitbuffer->bits_per_row[0]);
        return DECODE_ABORT_EARLY; // Unrecognized data
    }

    // TODO: support multiple rows
    // for (size_t bufferRow = 0; bufferRow < bitbuffer->num_rows; bufferRow++) {
    //     const bitrow_t *row = &bitbuffer->bb[bufferRow];
    //     ec3_decode_row(r_device *decoder, row);
    // }

    ec3_decode_row(decoder, bitbuffer->bb[0], bitbuffer->bits_per_row[0], pulses);

    return 1;
}

/*
 * List of fields that may appear in the output
 *
 * Used to determine what fields will be output in what
 * order for this device when using -F csv.
 *
 */
static char const *const output_fields[] = {
        "model",
        "id",
        "wcurrent",
        "energy",
        // "mic",
        NULL,
};

const r_device ec3k = {
    .name           = "EnergyCounter 3000",
    .modulation     = FSK_PULSE_PCM,
    .short_width    = BITTIME_US,
    .long_width     = BITTIME_US,
    .tolerance      = 2 ,    // %
    .gap_limit      = 3000,  // some distance above long
    .reset_limit    = 5000, // a bit longer than packet gap
    //.sync_pattern   = EC3K_SYNC_PATTERN,
    .decode_fn      = &ec3k_decode,
    .disabled       = 0,
    .fields         = output_fields,
    .verbose        = 3,
};
