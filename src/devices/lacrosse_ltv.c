/** @file
    LaCrosse Technology View LTV-series Weather Sensors.

    Copyright (C) 2020 Mike Bruski (AJ9X) <michael.bruski@gmail.com>
    Copyright (C) 2025 Cory Snider <corhere@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
LaCrosse Technology View LTV-TH3 & LTV-TH2 Thermo/Hygro Sensor.

LaCrosse Color Forecast Station (model S84060) utilizes the remote
Thermo/Hygro LTV-TH3 and LTV-WR1 multi sensor (wind spd/dir and rain).
LaCrosse Color Forecast Station (model C84343) utilizes the remote
Thermo/Hygro LTV-TH2.

Product pages:
https://www.lacrossetechnology.com/products/S84060
https://www.lacrossetechnology.com/products/ltv-th3
https://www.lacrossetechnology.com/products/C84343
https://www.lacrossetechnology.com/products/ltv-th2

Specifications:
- Outdoor Temperature Range: -40 C to 60 C
- Outdoor Humidity Range: 10 to 99 %RH
- Update Interval: Every 30 Seconds

No internal inspection of the sensors was performed so can only
speculate that the remote sensors utilize a HopeRF CMT2119A ISM
transmitter chip which is tuned to 915Mhz.

Again, no inspection of the S84060 or C84343 console was performed but
it probably utilizes a HopeRF CMT2219A ISM receiver chip.  An application
note is available that provides further info into the capabilities of the
CMT2119A and CMT2219A.

(http://www.cmostek.com/download/CMT2119A_v0.95.pdf)
(http://www.cmostek.com/download/CMT2219A.pdf)
(http://www.cmostek.com/download/AN138%20CMT2219A%20Configuration%20Guideline.pdf)

Protocol Specification:

Data bits are NRZ encoded.  Logical 1 and 0 bits are 104us in
length for the LTV-TH3 and 107us for the LTV-TH2.

LTV-TH3
    SYNC:32h ID:24h ?:4b SEQ:3b ?:1b TEMP:12d HUM:12d CHK:8h END:

    CHK is CRC-8 poly 0x31 init 0x00 over 7 bytes following SYN

LTV-TH2
    SYNC:32h ID:24h ?:4b SEQ:3b ?:1b TEMP:12d HUM:12d CHK:8h END:

Sequence# 2 & 6
    CHK is CRC-8 poly 0x31 init 0x00 over 7 bytes following SYN
Sequence# 0,1,3,4,5 & 7
    CHK is CRC-8 poly 0x31 init 0xac over 7 bytes following SYN

LTV-TH2i
    SYNC:32h ID:24h BATTLOW:1b RETRANS:1b ?:2b SEQ:3b ?:1b TEMP:12d HUM:12d CHK:8h TRAILER:96h

    CHK is CRC-8 poly 0x31 init (0x00 or 0xb2) over 7 bytes following SYN
    The CRC init value for a given packet is seemingly arbitrary, though it
    appears that the same init value is used for at most three packets in a row
    before switching.

    Pressing the TX button under the battery cover causes the sensor to
    immediately retransmit the most recent packet with the RETRANS flag set.

    TRAILER is 0xd2d2d2d2d200000000000000...

*/

/*

 - LTV-TH2/TH3/TH2i
    SYNC:32h ID:24h BATTLOW:1b RETRANS:1b ?:2b SEQ:3b ?:1b | TEMP:12d HUM:12d CHK:8h (4 bytes) TRAILER:96h
    crc init=0x00, 0xac, or 0xb2
    known PIDs:
        TH2:
            13efb7
        TH2i:
            136c21
        TH3:
            110851
 - LTV-WR1/LTV-WSDR1
    SYN:32h ID:24h ?:4b SEQ:3d ?:1b | WSPD:12d WDIR:12d RAIN1:12d RAIN2:12d CHK:8h (7 bytes)
    crc init=0x00
    known PIDs:
        WR1:
            19047a
        WSDR1:
            a0552d
 - LTV-WSDTH0x (Breeze Pro)
     SYNC:32h ID:24h ?:4b SEQ:3d ?:1b | TEMP:12d HUM:12d WSPD:12d WDIR:12d CHK:8h (7 bytes) END:32h
     crc init=0x00
     known PIDs:
        0b3829
        0b76ed
 - LTV-R1
     PRE:32h SYNC:32h ID:24h ?:4b SEQ:3d ?:1b | RAIN:24h CRC:8h (4 bytes) CHK?:8h TRAILER:96h
    crc init=0x00
    known PIDs:
        380322
 - LTV-R3
    PRE:58h SYNC:32h ID:24h ?:4b SEQ:3d ?:1b | RAIN:24h RAIN:24h CRC:8h (7 bytes) TRAILER:56h
    crc init=0x00
    known PIDs:
        71061d
        70f6a2
        72b9f2 (EU)
 - LTV-W1/LTV-W2
    ID:24h BATTLOW:1b STARTUP:1b ?:2b SEQ:3h ?:1b | 8h8h8h WIND:12d 12h CRC:8h (7 bytes) TRAILER 8h8h8h8h8h8h8h8h
      NB: same format as Breeze Pro, with aa... placeholders for missing fields
    crc init=0x00
    known PIDs:
        W1:
            0fb220

*/

#include <stdbool.h>
#include "decoder.h"

static int lacrosse_ltv_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble_pattern[] = {0xd2, 0xaa, 0x2d, 0xd4};

    // Not all LTV sensors transmit a trailer, and I don't trust the all-zeroes
    // trailers to be anything more than an artifact of the hardware not
    // disabling the transmit carrier quickly enough.

    // aaaaaaaa d2aa2dd4 (preamble + sync -- 8 bytes) + fixed header (4 bytes) + 3 (or 6) data bytes + 1 byte CRC (+ trailer)
    // = 16 or 19 bytes total

    int abort_length = 0, abort_early = 0, abort_sanity = 0, fail_mic = 0, success = 0;

    int r;
    for (r = 0; r < bitbuffer->num_rows; ++r) {
        if (bitbuffer->bits_per_row[r] < 16 * 8) {
            decoder_logf(decoder, 1, __func__, "[%d] Packet too short: %d bits", r, bitbuffer->bits_per_row[r]);
            abort_length++;
            continue;
        }
        else if (bitbuffer->bits_per_row[0] > 290) {
            decoder_logf(decoder, 1, __func__, "[%d] Packet too long: %d bits", r, bitbuffer->bits_per_row[r]);
            abort_length++;
            continue;
        }
        else {
            decoder_logf(decoder, 1, __func__, "[%d] Packet length: %d", r, bitbuffer->bits_per_row[r]);
        }

        int offset = bitbuffer_search(bitbuffer, r, 0,
                preamble_pattern, sizeof(preamble_pattern) * 8);

        if (offset >= bitbuffer->bits_per_row[r]) {
            decoder_logf(decoder, 1, __func__, "[%d] Sync word not found", r);
            abort_early++;
            continue;
        }

        offset += sizeof(preamble_pattern) * 8;
        if (offset + 8 * 8 >= bitbuffer->bits_per_row[r]) {
            decoder_logf(decoder, 1, __func__, "[%d] Not enough bits for data: %d bits", r, bitbuffer->bits_per_row[r] - offset);
            abort_early++;
            continue;
        }
        
        uint8_t b[11] = {0};
        // Make sure we don't read past the end of the bitbuffer row.
        unsigned extract_len = bitbuffer->bits_per_row[r] - offset;
        if (extract_len > sizeof(b) * 8)
            extract_len = sizeof(b) * 8;
        bitbuffer_extract_bytes(bitbuffer, r, offset, b, extract_len);

        // For some as-yet unknown reason, the LTV-TH2 and LTV-TH2i use a
        // particular nonzero CRC-8 init value that depends on the model
        // for the majority of packets -- but not all.
        bool crc7_th2_ok  = !crc8(b, 8, 0x31, 0xac);
        bool crc7_th2i_ok = !crc8(b, 8, 0x31, 0xb2);
        bool crc7_ok      = crc7_th2_ok || crc7_th2i_ok || !crc8(b, 8, 0x31, 0x00);
        bool crc10_ok    = !crc8(b, 11, 0x31, 0x00);

        if (!crc7_ok && !crc10_ok) {
            decoder_logf(decoder, 1, __func__, "[%d] CRC failed!", r);
            fail_mic++;
            continue;
        }

        // Extract fixed header fields that are common to all models.
        uint32_t id  = (b[0] << 16) | (b[1] << 8) | b[2];
        int seq      = (b[3] & 0x0e) >> 1;
        int flags    = (b[3] & 0xf1); // mask off seq bits

        // Now we play guess the model!
        // The secret probably is in the ID. However, without a significant
        // sample of IDs, we can only speculate. The MSB of the ID seems pretty
        // consistent across units where we have multiple examples and different
        // in different models. If the manufacturer has assigned the most
        // significant byte of the ID to identify the model, that would allow
        // for 2^32 unique units of 2^8 models. That sounds reasonable enough.
        // Let's work off that assumption until proven otherwise.

#define DATA_MAKE_COMMON_HEADER \
            "id",               "Sensor ID",        DATA_FORMAT, "%06x", DATA_INT, id, \
            "seq",              "Sequence",         DATA_INT,     seq, \
            "flags",            "unknown",          DATA_COND,    flags, DATA_INT, flags
#define DATA_MAKE_COMMON_FOOTER \
            "mic",              "Integrity",        DATA_STRING,  "CRC", \
            NULL

        if (crc7_ok) switch (id >> 16) {
            case 0x11: // LTV-TH3
            case 0x13: // LTV-TH2/TH2i
            {
                int raw_temp = b[4] << 4 | ((b[5] & 0xf0) >> 4);
                int humidity = ((b[5] & 0x0f) << 8) | b[6];
        
                bool batt_low = (flags & 0x80);
                bool retrans  = (flags & 0x40);
                flags &= 0x31; // mask off known bits
        
                // base and/or scale adjustments
                float temp_c = (raw_temp - 400) * 0.1f;
        
                if (humidity < 0 || humidity > 100 || temp_c < -50 || temp_c > 70) {
                    abort_sanity++;
                    continue;
                }

                char const *model_suffix = "2/2i/3";
                switch (id >> 16) {
                    case 0x13:
                        if (crc7_th2i_ok) {
                            model_suffix = "2i";
                        } else if (crc7_th2_ok) {
                            model_suffix = "2";
                        } else {
                            model_suffix = "2/2i";
                        }
                        break;
                    case 0x11:
                        model_suffix = "3";
                        break;
                }

                /* clang-format off */
                data_t *data = data_make(
                    "model", "", DATA_FORMAT, "La Crosse LTV-TH%s", DATA_STRING, model_suffix,
                    DATA_MAKE_COMMON_HEADER,
                    "battery_ok",       "Battery",          DATA_INT,    !batt_low,
                    "retransmit",       "Retransmit",       DATA_COND,   retrans,   DATA_INT, retrans,
                    "temperature_C",    "Temperature",      DATA_FORMAT, "%.1f C",  DATA_DOUBLE, temp_c,
                    "humidity",         "Humidity",         DATA_FORMAT, "%u %%", DATA_INT, humidity,
                    DATA_MAKE_COMMON_FOOTER
                );
                /* clang-format on */

                decoder_output_data(decoder, data);
                success++;
                continue;
            }
            break;

            case 0x38: // LTV-R1
            {
                // Note that the rain zero value is 00aa00 with a known byte order of HH??LL.
                // We just prepend the middle byte and assume whitening. Let's hope we get feedback someday.
                bool batt_low  = (flags & 0x80);
                bool startup   = (flags & 0x40);
                flags     &= 0x31; // mask off known bits
                int raw_rain = ((b[5] ^ 0xaa) << 16) | (b[4] << 8) | (b[6]);
                float rain_in = raw_rain * 0.01f;
            
                /* clang-format off */
                data_t *data = data_make(
                    "model",            "",                 DATA_STRING, "La Crosse LTV-R1",
                    DATA_MAKE_COMMON_HEADER,
                    "battery_ok",       "Battery",          DATA_INT,    !batt_low,
                    "startup",          "Startup",          DATA_COND,   startup,   DATA_INT,    startup,
                    "rain_in",          "Total Rain",       DATA_FORMAT, "%.2f in", DATA_DOUBLE, rain_in,
                    DATA_MAKE_COMMON_FOOTER);
                /* clang-format on */

                decoder_output_data(decoder, data);
                success++;
                continue;
            }

            // TODO: dump output of other 7-byte crc7_ok packets
        }


        // LTV-R3, TFA 30.3802.02
        // and (wild guess) id 73xxxx.
        if (crc10_ok && (id & 0xfc0000) == 0x700000) { 
            bool batt_low  = (flags & 0x80);
            bool startup   = (flags & 0x40);
            flags     &= 0x31; // mask off known bits
            int raw_rain1 = ((b[5] ^ 0xaa) << 16) | (b[4] << 8) | (b[6]);
            int raw_rain2 = ((b[8] ^ 0xaa) << 16) | (b[7] << 8) | (b[9]);
            float rain_in = raw_rain1 * 0.01f;
            float rain2_in = raw_rain2 * 0.01f;
        
            /* clang-format off */
            data_t *data = data_make(
                "model",            "",                 DATA_STRING, "La Crosse LTV-R3",
                DATA_MAKE_COMMON_HEADER,
                "battery_ok",       "Battery",          DATA_INT,    !batt_low,
                "startup",          "Startup",          DATA_COND,   startup,   DATA_INT,    startup,
                "rain_in",          "Total Rain",       DATA_FORMAT, "%.2f in", DATA_DOUBLE, rain_in,
                "rain2_in",         "Total Rain2",      DATA_FORMAT, "%.2f in", DATA_DOUBLE, rain2_in,
                DATA_MAKE_COMMON_FOOTER);
            /* clang-format on */
        
            decoder_output_data(decoder, data);
            success++;
            continue;
        }

        // Most other 10-byte packets encode four 12-bit values.
        if (crc10_ok) {
            int v1 = b[4] << 4 | ((b[5] & 0xf0) >> 4);
            int v2 = ((b[5] & 0x0f) << 8) | b[6];
            int v3 = b[7] << 4 | ((b[8] & 0xf0) >> 4);
            int v4 = ((b[8] & 0x0f) << 8) | b[9];

            switch (id >> 16) {
                case 0x19: // LTV-WR1
                case 0xa0: // LTV-WSDR1
                {
                    float speed_kmh = v1 * 0.1f;
                    int direction = v2;
                    float rain1_in = v3 * 0.01f;
                    float rain2_in = v4 * 0.01f;

                    if (direction < 0 || direction > 360 || speed_kmh < 0 || speed_kmh > 200) {
                        abort_sanity++;
                        continue;
                    }

                    char const *model_suffix = (id >> 16) == 0x19 ? "WR1" : "WSDR1";

                    /* clang-format off */
                    data_t *data = data_make(
                        "model",            "",                 DATA_FORMAT, "La Crosse LTV-%s", DATA_STRING, model_suffix,
                        DATA_MAKE_COMMON_HEADER,
                        "wind_avg_km_h",    "Wind speed",       DATA_FORMAT, "%.1f km/h",  DATA_DOUBLE, speed_kmh,
                        "wind_dir_deg",     "Wind direction",   DATA_INT,    direction,
                        "rain_in",         "Total Rain",      DATA_FORMAT, "%.2f in", DATA_DOUBLE, rain1_in,
                        "rain2_in",         "Total Rain2",      DATA_FORMAT, "%.2f in", DATA_DOUBLE, rain2_in,
                        DATA_MAKE_COMMON_FOOTER);
                    /* clang-format on */

                    decoder_output_data(decoder, data);
                    success++;
                    continue;
                }
                break;

                case 0x0b: // LTV-WSDTH0x (Breeze Pro)
                {
                    int raw_temp = v1;
                    int humidity = v2;
                    int raw_speed = v3;
                    int direction = v4;

                    // base and/or scale adjustments
                    float temp_c = (raw_temp - 400) * 0.1f;
                    float speed_kmh = raw_speed * 0.1f;

                    if (humidity < 0 || humidity > 100
                            || temp_c < -40 || temp_c > 70
                            || direction < 0 || direction > 360
                            || speed_kmh < 0 || speed_kmh > 200) {
                        abort_sanity++;
                        continue;
                    }

                    /* clang-format off */
                    data_t *data = data_make(
                        "model",            "",                 DATA_STRING, "La Crosse Breeze Pro",
                        DATA_MAKE_COMMON_HEADER,
                        "temperature_C",    "Temperature",      DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_c,
                        "humidity",         "Humidity",         DATA_FORMAT, "%u %%", DATA_INT, humidity,
                        "wind_avg_km_h",    "Wind speed",       DATA_FORMAT, "%.1f km/h",  DATA_DOUBLE, speed_kmh,
                        "wind_dir_deg",     "Wind direction",   DATA_INT,    direction,
                        DATA_MAKE_COMMON_FOOTER);
                    /* clang-format on */

                    decoder_output_data(decoder, data);
                    success++;
                    continue;
                }
                break;

                case 0x0f: // LTV-W1/LTV-W2
                {
                    bool batt_low  = (flags & 0x80);
                    bool startup   = (flags & 0x40);
                    flags     &= 0x31; // mask off known bits
                    int raw_speed  = v3;
                    float speed_kmh = raw_speed * 0.1f;

                    if (speed_kmh > 200) {
                        abort_sanity++;
                        continue;
                    }

                    /* clang-format off */
                    data_t *data = data_make(
                        "model",            "",                 DATA_STRING, "La Crosse LTV-W1/W2",
                        DATA_MAKE_COMMON_HEADER,
                        "battery_ok",       "Battery",          DATA_INT,    !batt_low,
                        "startup",          "Startup",          DATA_COND,   startup,   DATA_INT,    startup,
                        "wind_avg_km_h",    "Wind speed",       DATA_FORMAT, "%.1f km/h",  DATA_DOUBLE, speed_kmh,
                        DATA_MAKE_COMMON_FOOTER);
                    /* clang-format on */

                    decoder_output_data(decoder, data);
                    success++;
                    continue;
                }
                break;

                default: // Unrecognized ID
                {
                    /* clang-format off */
                    data_t *data = data_make(
                        "model",            "",                 DATA_STRING, "Unknown La Crosse LTV Sensor",
                        DATA_MAKE_COMMON_HEADER,
                        "v1",               "Value 1",         DATA_INT,     v1,
                        "v2",               "Value 2",         DATA_INT,     v2,
                        "v3",               "Value 3",         DATA_INT,     v3,
                        "v4",               "Value 4",         DATA_INT,     v4,
                        DATA_MAKE_COMMON_FOOTER);
                    /* clang-format on */

                    decoder_output_data(decoder, data);
                    success++;
                    continue;
                }
                break;
            }
        }
    }

    if (success)
        return success;
    // Report the failure for the furthest progress made.
    if (abort_sanity)
        return DECODE_FAIL_SANITY;
    if (fail_mic)
        return DECODE_FAIL_MIC;
    if (abort_early)
        return DECODE_ABORT_EARLY;
    if (abort_length)
        return DECODE_ABORT_LENGTH;
    return 0;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "battery_ok",
        "retransmit",
        "seq",
        "flags",
        "temperature_C",
        "humidity",
        "rain_in",
        "rain2_in",
        "wind_avg_km_h",
        "wind_dir_deg",
        "startup",
        "v1",
        "v2",
        "v3",
        "v4",
        "mic",
        NULL,
};

// flex decoder n=TH3, m=FSK_PCM, s=104, l=104, r=9600
// flex decoder n=TH2, m=FSK_PCM, s=107, l=107, r=5900
// TH3 parameters should be good enough for both sensors
r_device const lacrosse_ltv = {
        .name        = "LaCrosse Technology View LTV Sensors",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 104,
        .long_width  = 104,
        .reset_limit = 9600,
        .decode_fn   = &lacrosse_ltv_decode,
        .fields      = output_fields,
};
