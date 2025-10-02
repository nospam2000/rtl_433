/** @file
    Decoder for EnergyCounter 3000 (ec3k), tested with
      - Voltcraft ENERGYCOUNT 3000 ENERGY LOGGER 

    Copyright (C) 2025 Michael Dreher @nospam2000

    decoding info taken from https://github.com/EmbedME/ec3k_decoder

    seems to work ok with this params, samplerate is very critical:
    rtl_433 -f 868200k -s 1000000 -R 282

    TODO: why does the rowlen depend on the samplerate? Should't it only depend on the bittime?

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
#define DECODED_PAKET_LEN_BYTES 41

 // the number of bits in the decoded packet + some margin
#define max_out_bits ((DECODED_PAKET_LEN_BYTES + 2*50) * 8)

// static const uint8_t LEVEL_THRESHOLD = 47;
// static const uint8_t LEVEL_MIN = 20;
// static const uint8_t LEVEL_MAX = 70;
static const uint32_t BITTIME_US = 50;

// values for 200 kHz sample rate, need to be adapted to actual sample rate
#define PAKET_MIN_BITS 90
#define PAKET_MAX_BITS (PAKET_MIN_BITS * 5 / 2) // NRZ encoding, stuffing and noise

static int ec3_decode_row(r_device *const decoder, const bitrow_t row, const uint16_t row_bits, const pulse_data_t *pulses);
static int ec3k_decode(r_device *decoder, bitbuffer_t *bitbuffer, const pulse_data_t *pulses);

// copied from bitbuffer.c
static inline uint8_t bit_at(const uint8_t *bytes, unsigned bit)
{
    return (uint8_t)(bytes[bit >> 3] >> (7 - (bit & 7)) & 1);
}

static inline int min(int a, int b) {
    return a < b ? a : b;
}

static int ec3_decode_row(r_device *const decoder, const bitrow_t row, const uint16_t row_bits, const pulse_data_t *pulses) {
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

    char bitbuffer[max_out_bits];
    int bufferpos = 0;
    char lastlevel = 0;      // either 0 or 1
    for (size_t col = 0; (col < row_bits) && (bufferpos < (max_out_bits - 1)) ; col++)
    {
        const uint8_t level = bit_at((const uint8_t*)row, col); // TODO: is the bitorder correct?

        // edge detection
        const int symbol = (level == lastlevel) ? 1 : 0; // no signal change => '1', signal change => '0'
        bitbuffer[bufferpos++] = symbol; 
        if (bufferpos > PAKET_MIN_BITS)
        {
#if 0
            printf("*Len=%-4i ", bufferpos);
            for (int i = 0; i < bufferpos; i++) {
                printf("%i", bitbuffer[i]);
            }
            printf("\n");
#endif
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
                            rc = 1;
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
        }

        lastlevel = level;
    }

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
    .name           = "EnergyCounter 3000",
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
