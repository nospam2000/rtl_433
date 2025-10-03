/** @file
    Decoder for Voltcraft Energy Count 3000 (ec3k) / Cost Control RT-110 sold by Conrad, tested with
      - Voltcraft ENERGYCOUNT 3000 ENERGY LOGGER (Item No. 12 53 53, https://conrad-rus.ru/images/stories/virtuemart/media/125353-an-01-ml-TCRAFT_ENERGYC_3000_ENER_MESSG_de_en_nl.pdf)
        (don't mix that up with similar products from the same company like "Energy Check 3000", "Energy Monitor 3000" and "Energy Logger 4000")

    Should also work with these devices from other manufacturers that use the same protocol:
      - Technoline Cost Control RT-110
      - Velleman (type NETBESEM4)
      - La Crosse Techology Remote Cost Control Monitor” (type RS3620)

    Copyright (C) 2025 Michael Dreher @nospam2000

    decoding info taken from
      - https://github.com/EmbedME/ec3k_decoder (using rtl_fm)
      - https://github.com/avian2/ec3k (using python and gnuradio)
      
    Some more info can be found here:
      - https://www.sevenwatt.com/main/rfm69-energy-count-3000-elv-cost-control/
      - https://batilanblog.wordpress.com/2015/01/11/getting-data-from-voltcraft-energy-count-3000-on-your-computer/
      - https://web.archive.org/web/20121019130917/http://forum.jeelabs.net:80/comment/4020

    The device transmits every 6 seconds (if there is a change in power consumption) or every 30 minutes (if there is no change).
    It uses BFSK modulation with two frequencies between 30 and 80 kHz apart (e.g. 868.297 and 868.336 MHz).
    The bit rate is 20 kbit/s, so bit time is 50 us.

    The used chip is probably a AX5042 from On Semiconductor (formerly from Axsem), datasheet: https://www.onsemi.com/download/data-sheet/pdf/ax5042-d.pdf
    HDLC mode follows High−Level Data Link Control (HDLC, ISO 13239) protocol. HDLC Mode is the main framing mode of the AX5042.
    HDLC packets are delimited with flag sequences of content 0x7E. In AX5042 the meaning of address and control is user defined.
    The Frame Check Sequence (FCS) can be programmed to be CRC−CCITT, CRC−16 or CRC−32.
    The CRC is appended to the received data. There could be an optional flag byte after the CRC.

    The packet is NRZI encoded, with bit stuffing (a 0 is inserted after 5 consecutive 1 bits).
    The packet is framed by 0x7E (01111110) bytes at start and end.
    The packet length is 41 bytes (328 bits) excluding the two framing bytes.
    The packet contains a CRC-CCITT CRC16 or CRC12
    TODO: check which one is correct
        12-bit CRC (CRC-12/3GPP, polynomial 0x80F, init 0xFFF, no reflection, xorout 0x000, not including syncword at the end).

    The following fields are decoded:
        id -- 16-bit ID of the device
        time_total -- time in seconds since last reset
        time_on -- time in seconds since last reset with non-zero device power
        energy -- total energy in Ws (watt-seconds)
        power_current -- current device power in watts
        power_max -- maximum device power in watts (reset at unknown intervals)
        reset_counter -- total number of transmitter resets
        device_on_flag -- true if device is currently drawing non-zero power

    The CRC is calculated over the whole packet including the leading framing byte 0x7E, but not the crc-value itself and also not the
    trailing framing byte 0x7E. The CRC bytes in the packet are in little-endian order (low byte first). I didn't find the parameters
    for a standard implementation, so I took the implementation from the python code at https://github.com/avian2/ec3k.

    It seems to work with this params, samplerate 1M is very critical:
    rtl_433 -f 868200k -s 1000000 -R 282

    TODO: why does the rowlen depend on the samplerate? Should't it only depend on the bittime? Is this the reason why decoding fails with other samplerates?
    TODO: implement searching for start of packet, currently we just start at bit 0 and expect a full packet
    TODO: check CRC
    TODO: implement other fields

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
#define DECODED_PAKET_LEN_BYTES (41)
 // the number of bits in the decoded packet + some margin
#define max_out_bits ((DECODED_PAKET_LEN_BYTES + 2*50) * 8)

// static const uint8_t LEVEL_THRESHOLD = 47;
// static const uint8_t LEVEL_MIN = 20;
// static const uint8_t LEVEL_MAX = 70;
static const uint32_t BITTIME_US = 50;

// values for 200 kHz sample rate, need to be adapted to actual sample rate
#define PAKET_MIN_BITS (90)
#define PAKET_MAX_BITS (PAKET_MIN_BITS * 5 / 2) // NRZ encoding, stuffing and noise

static int ec3_decode_row(r_device *const decoder, const bitrow_t row, const uint16_t row_bits, const pulse_data_t *pulses);
static int ec3k_decode(r_device *decoder, bitbuffer_t *bitbuffer, const pulse_data_t *pulses);
static uint16_t calc_ec3k_crc(uint8_t *buffer, size_t len);
static uint16_t update_ec3k_crc(uint16_t crc, uint8_t ch);

static inline uint8_t bit_at(const uint8_t *bytes, int bit)
{
    return (uint8_t)(bytes[bit >> 3] >> (7 - (bit & 7)) & 1);
}

static inline uint8_t symbol_at(const uint8_t *bytes, int bit)
{
    // NRZI decoding
    int bit0 = (bit > 0) ? bit_at(bytes, bit - 1) : 0;
    int bit1 = bit_at(bytes, bit);
    return (bit0 == bit1) ? 1 : 0;
}

static inline int min(int a, int b) {
    return a < b ? a : b;
}

static uint32_t unpack_nibbles(const uint8_t* buf, int start_nibble, int num_nibbles)
{
    uint32_t val = 0;
    for (int i = 0; i < num_nibbles; i++)
    {
        val = (val << 4) | ((buf[(start_nibble + i) / 2] >> ((1 - ((start_nibble + i) & 1)) * 4)) & 0x0F);
    }
    return val;
}

static int ec3_decode_row(r_device *const decoder, const bitrow_t row, const uint16_t row_bits, __attribute_maybe_unused__ const pulse_data_t *pulses) {
    int rc = 0;
    // const uint32_t ec3_sample_rate = pulses->sample_rate;

    // const int BITTIME = (((uint64_t)BITTIME_US * ec3_sample_rate) / 1000000LL);
    // const int BITTIME_BOUND_LOWER = (((uint64_t)BITTIME_US * ec3_sample_rate) * 9) / (1000000LL * 10);
    // const int BITTIME_BOUND_UPPER = (((uint64_t)BITTIME_US * ec3_sample_rate) * 11) / (1000000LL * 10);
    // const int BITTIME = 1;
    // const int BITTIME_BOUND_LOWER = 1;
    // const int BITTIME_BOUND_UPPER = 1;

#if 0
    int32_t diffFreq = (int32_t)(pulses->freq2_hz - pulses->freq1_hz + 0.5f);
    if(diffFreq > 20000 && diffFreq < 110000) {
        printf("#f1=%d f2=%d diff=%d ", (int32_t)(pulses->freq1_hz - pulses->centerfreq_hz + 0.5f), (int32_t)(pulses->freq2_hz - pulses->centerfreq_hz + 0.5f), diffFreq);
        printf("#RowLen=%-4i ", row_bits);
        for (int i = 0; i < row_bits; i++) {
            printf("%i", bit_at((const uint8_t*)row, i));
        }
        printf("\n");

        printf("#PulseLen=%-4i ", pulses->num_pulses);
        for (unsigned int i = 0; i < pulses->num_pulses; i++) {
            printf(" +%d -%d", pulses->pulse[i], pulses->gap[i]);
        }
        printf("\n");
#if 0
        printf("#RowLen=%-4i ", row_bits);
        for (int i = 0; i < (row_bits + 7) / 8; i++) {
            printf("%x", row[i]);
        }
        printf("\n");
#endif
    }
#endif

    int bufferpos = 0;
    //for (size_t col = 0; (col < row_bits) && (bufferpos < (max_out_bits - 1)) ; col++)
    unsigned char packetbuffer[100]; // TODO: review size and replace with constant; check for overflow
    int packetpos = 0;
    unsigned char packet = 0;
    char onecount = 0;
    unsigned char recbyte = 0;
    char recpos = 0;
    // printf("Descrambled/unstuffed: \n");

    // TODO: align the implementation with this code: https://github.com/avian2/ec3k/blob/master/ec3k.py
    // TODO: iterate over the input bits to find the start of the packet, check for preamble and sync (01111110 or 10000001)
    // currently we just start at bit 0 and expect a full packet
    for (int i = 17; i < row_bits; i++)
    {
        char out = symbol_at((const uint8_t*)row, bufferpos + i);
        if (i > 17)
            out = out ^ symbol_at((const uint8_t*)row, bufferpos + i - 17);
        if (i > 12)
            out = out ^ symbol_at((const uint8_t*)row, bufferpos + i - 12);

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
                    // decode received ec3k packet
                    uint64_t energy = ((packetbuffer[33] & 0x0f) << (8 + 4) | (packetbuffer[34]) << 4 | (packetbuffer[35]) >> 4);
                    energy = energy << 28 | packetbuffer[12] << 20 | packetbuffer[13] << 12 | packetbuffer[14] << 4 | (packetbuffer[15] >> 4);
                    
                    uint16_t id              = unpack_nibbles(packetbuffer, 1, 4);
                    uint16_t time_total_low  = unpack_nibbles(packetbuffer, 5, 4);
                    uint16_t pad_1           = unpack_nibbles(packetbuffer, 9, 4);
                    uint16_t time_on_low     = unpack_nibbles(packetbuffer, 13, 4);
                    uint32_t pad_2           = unpack_nibbles(packetbuffer, 17, 7);
                    uint32_t energy_low      = unpack_nibbles(packetbuffer, 24, 7);
                    double   power_current   = unpack_nibbles(packetbuffer, 31, 4) / 10.0;
                    double   power_max       = unpack_nibbles(packetbuffer, 35, 4) / 10.0;
                    // unknown? (seems to be used for internal calculations)
                    uint32_t energy_2        = unpack_nibbles(packetbuffer, 39, 6);
                    // 						nibbles[45:59]
                    uint16_t time_total_high = unpack_nibbles(packetbuffer, 59, 3);
                    uint32_t pad_3           = unpack_nibbles(packetbuffer, 62, 5);
                    uint64_t energy_high     = unpack_nibbles(packetbuffer, 67, 4) << 28;
                    uint16_t time_on_high    = unpack_nibbles(packetbuffer, 71, 3);
                    uint8_t  reset_counter   = unpack_nibbles(packetbuffer, 74, 2);
                    uint8_t  flags           = unpack_nibbles(packetbuffer, 76, 1);
                    uint8_t  pad_4           = unpack_nibbles(packetbuffer, 77, 1);
                    uint16_t received_crc    = unpack_nibbles(packetbuffer, 78, 2) | (unpack_nibbles(packetbuffer, 80, 2) << 8); // little-endian
                    uint16_t calculated_crc  = calc_ec3k_crc(packetbuffer, DECODED_PAKET_LEN_BYTES - 2);

                    // convert to common units
                    const double energy_kwh = energy / (1000.0 * 3600.0); // Ws to kWh
                    const double energy_kwh2 = (energy_high | energy_low) / (1000.0 * 3600.0); // Ws to kWh

                    if(pad_1 == 0 && pad_2 == 0 && pad_3 == 0 && pad_4 == 0 && calculated_crc == received_crc) {
                        /* clang-format off */
                        data_t *data = data_make(
                            "model",            "",             DATA_STRING, "Voltcraft Energy Count 3000",
                            "id",               "",             DATA_INT,    id,
                            "power",            "Power",        DATA_DOUBLE, power_current,
                            "energy",           "Energy",       DATA_DOUBLE, energy_kwh,
                            NULL);
                        /* clang-format on */

                        decoder_output_data(decoder, data);
                        rc = 1;
                    }
                    goto exit_decoder;
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

exit_decoder:
    return rc;
}


static int ec3k_decode(r_device *decoder, bitbuffer_t *bitbuffer, const pulse_data_t *pulses)
{
    const int32_t diffFreq = (int32_t)(pulses->freq2_hz - pulses->freq1_hz + 0.5f);
    if (       bitbuffer->num_rows != 1
            || bitbuffer->bits_per_row[0] < (PAKET_MIN_BITS * pulses->sample_rate / 200000) // adapt to sample rate
            || bitbuffer->bits_per_row[0] > (PAKET_MAX_BITS * pulses->sample_rate / 200000) // adapt to sample rate
            || diffFreq < 20000
            || diffFreq > 110000
        )
    {
        decoder_logf(decoder, 2, __func__, "bit_per_row %u out of range or frequency shift %d out of range", bitbuffer->bits_per_row[0], diffFreq);
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

// from the ec3k python implementation at https://github.com/avian2/ec3k
static uint16_t calc_ec3k_crc(uint8_t *buffer, size_t len)
{
    uint16_t crc = 0xffff;
    for(size_t i = 0; i < len; i++) {
        crc = update_ec3k_crc(crc, buffer[i]);
    }
    return crc ^ 0xffff;
}

static uint16_t update_ec3k_crc(uint16_t crc, uint8_t ch)
{
    ch ^= crc & 0xff;
    ch ^= (ch << 4) & 0xff;
    return (((uint16_t)ch << 8) | (crc >> 8)) ^ ((uint16_t)ch >> 4) ^ ((uint16_t)ch << 3);
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
        "power",
        "energy",
        // "mic",
        NULL,
};

const r_device ec3k = {
    .name           = "Voltcraft Energy Count 3000",
    .modulation     = FSK_PULSE_PCM,
    .short_width    = BITTIME_US,
    .long_width     = BITTIME_US,
    .tolerance      = BITTIME_US / 7, // in us ; there can be up to 5 consecutive 0 or 1 pulses and the sync word is 6 bits, so 1/7 should be ok
    .gap_limit      = 3000,  // some distance above long
    .reset_limit    = 5000, // a bit longer than packet gap
    //.sync_pattern   = EC3K_SYNC_PATTERN,
    .decode_fn      = &ec3k_decode,
    .disabled       = 0,
    .fields         = output_fields,
    .verbose        = 3,
    .verbose_bits   = 3,
};
