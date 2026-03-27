/** @file
    Decoder for Heidemann HX Extension.

    Copyright (C) 2026 Michael Dreher <michael(a)5dot1.de>, nospam2000 at github.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

#include "decoder.h"

/**
 *
Decoder for Heidemann HX Extension. The original manufacturer is probably Quhwa.
Original code taken from quhwa.c

The device uses OOK modulation with Pulse Width Coding (PWM):
- Short pulse (332 us) preceeded by a long gap (479 us) encodes a '1' bit.
- Long pulse (676 us) preceeded by a short gap (135 us) encodes a '0' bit.

The frame length is 13 bits, it is repeated 97 times with a gap of 6200 us between the repeats.

The frame contains the following information:
- Battery status (bit 0): 1=battery ok, 0=low battery. This is just guessed and not yet confirmed. I have only seen value 1 here.
- Device-ID (bit 1 to 8): It is randomly generated on each power on
- Melody (bit 9 to 12): which of the 16 melodies the receiver should play.

Decoding can be tested using these parameters:
    rtl_433 -R 0 -X "n=myproto,m=OOK_PWM,s=332,l=676,g=811,r=6200,bits=13" -Y autolevel

Or the full configuration with field extraction:
    rtl_433 -R 0 -X "name=Heidemann-HX-Extension-Cfg,modulation=OOK_PWM,s=332,l=676,g=900,r=7000,unique,repeats>=20,bits=13,get=battery_ok:@0:{1},get=id:@1:{8}:%d,get=melody:@9:{4}:%d" -Y autolevel

Check here for some example captures: https://github.com/merbanan/rtl_433_tests/tree/master/tests/heidemann_hx_extension/01
 */
static int heidemann_hx_extension_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // the message is repeated 97 times, require at least 20 repeated packets of 13 bits
    int r = bitbuffer_find_repeated_row(bitbuffer, 20, 13);
    if (r < 0)
        return DECODE_ABORT_EARLY;

    uint8_t *b = bitbuffer->bb[r];

    // No need to decode/extract values for simple test
    if (!b[0] && !b[1]) {
        decoder_log(decoder, 2, __func__, "DECODE_FAIL_SANITY data all 0x00");
        return DECODE_FAIL_SANITY;
    }

    if (bitbuffer->bits_per_row[r] != 13)
        return DECODE_ABORT_LENGTH;

    uint32_t rawval = (b[0] << 5) | ((b[1] >> 3) & 0x1F); // 13 bits
    uint32_t melody = (rawval & 0x0F); // 4 bits, there are 8 melodies available
    uint32_t id = ((rawval >> 4) & 0xFF); // 8 bits
    uint32_t batt_ok = ((rawval >> 12) & 0x01); // 1 bit

    /* clang-format off */
    data_t *data = data_make(
            "model",      "",        DATA_STRING, "Heidemann-HX-Extension",
            "id",         "ID",      DATA_INT,    id,
            "melody",     "Melody",  DATA_INT,    melody,
            "battery_ok", "Battery", DATA_INT,    batt_ok,
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);

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
        "melody",
        "battery_ok",
        NULL,
};

/*
 * r_device - registers device/callback. see rtl_433_devices.h
 *
 * Timings:
 *
 * short, long, and reset - specify pulse/period timings in [us].
 *     These timings will determine if the received pulses
 *     match, so your callback will fire after demodulation.
 *
 * Modulation:
 *
 * The function used to turn the received signal into bits.
 * See:
 * - pulse_slicer.h for descriptions
 * - r_device.h for the list of defined names
 *
 */
r_device const heidemann_hx_extension = {
        .name        = "Heidemann HX Extension",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 332,  // in us
        .long_width  = 676,  // in us
        .gap_limit   = 900,  // some distance above long 820
        .reset_limit = 7000, // a bit longer than packet gap 6200
        .sync_width  = 0,    // No sync bit used
        .tolerance   = 50,   //  in us
        .disabled    = 0,    // use 0 if there is a MIC, 1 otherwise; this device repeats the frame 97 times, so it is very unlikely to produce false positives and can be enabled by default
        .decode_fn   = &heidemann_hx_extension_decode,
        .fields      = output_fields,
};
