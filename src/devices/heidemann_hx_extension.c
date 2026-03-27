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
- Short pulse (332 us) preceeded by a long gap (479 us) encodes a '0' bit.
- Long pulse (676 us) preceeded by a short gap (135 us) encodes a '1' bit.

The frame length is 13 bits, it is repeated 97 times with a gap of 6200 us between the repeats.

The followinginformation could be part of the message, but is not decoded yet and only a subset can be
contained because of the short message length:
- System-ID / House code (8–16 bits): This is the part that forms the "256 radio channels".
  It is randomly generated or permanently assigned during pairing so that neighbors don't interfere with each other.
- Command / Melody (4 bits): Often transmits the information about which of the 8 or more melodies the receiver should play.
- Checksum (optional): Simple check for error detection.
- Battery status (optional): Some devices might include a bit indicating low battery.

Decoding can be tested using these parameters:
    rtl_433 -R 0 -X "n=myproto,m=OOK_PWM,s=332,l=676,g=811,r=6200,bits=13" -Y autolevel

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

    bitbuffer_invert(bitbuffer);

    if (bitbuffer->bits_per_row[r] != 13
            // || (b[1] & 0x03) != 0x03
            // || (b[2] & 0xC0) != 0xC0
        )
        return DECODE_ABORT_LENGTH;

    uint32_t rawval = (b[0] << 5) | (b[1] & 0x1F); // 13 bits
    uint32_t batt_low = ((rawval >> 12) & 0x01); // 1 bit ???
    uint32_t unknown = ((rawval >> 8) & 0x01); // 1 bit ???
    uint32_t id = ((rawval >> 4) & 0x7F); // 7 bits and bit 4 is always 0
    uint32_t melody = (rawval & 0x0F); // 4 bits ???

    /* clang-format off */
    data_t *data = data_make(
            "model",  "",    DATA_STRING, "Heidemann-HX-Extension",
            "id",     "ID",  DATA_INT, id,
            "melody",     "Melody",  DATA_INT, melody,
            "unknown",       "Unknown",      DATA_INT,    unknown,
            "battery_ok",       "Battery",      DATA_INT,    !batt_low,
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
        "unknown",
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
 * This device is disabled and hidden, it can not be enabled.
 *
 * To enable your device, append it to the list in include/rtl_433_devices.h
 * and sort it into src/CMakeLists.txt or run ./maintainer_update.py
 *
 */
r_device const heidemann_hx_extension = {
        .name        = "Heidemann HX Extension",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 332, // in us
        .long_width  = 676, // in us
        .gap_limit   = 900,            // some distance above long 820
        .reset_limit = 7000,            // a bit longer than packet gap 6200
        .sync_width  = 0,    // No sync bit used
        .tolerance   = 50, //  in us
        .decode_fn   = &heidemann_hx_extension_decode,
        .disabled    = 0, // use 0 if there is a MIC, 1 otherwise
        .fields      = output_fields,
};
