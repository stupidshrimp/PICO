#include "m8n.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

M8N::M8N(Stream &uart) : latitude(0.0), longitude(0.0),
                         speed(0.0), course(0.0), altitude(0.0),
                         fix_quality(0), satellites_in_use(0), has_valid_fix(false),
                         uart(uart), nmeaBufferIndex(0), nmeaDiscarding(false), rmcDataActive(false) {
    timestamp[0] = '\0';
    date[0] = '\0';
    nmeaBuffer[0] = '\0';
}

// Computes the UBX Fletcher-8 checksum over a frame's class/id/length/payload
// bytes (everything between the two sync chars and the checksum) and writes
// CK_A/CK_B into the final two bytes of the frame.
static void ubxAppendChecksum(uint8_t *frame, size_t frameLen) {
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (size_t i = 2; i < frameLen - 2; ++i) {
        ckA = static_cast<uint8_t>(ckA + frame[i]);
        ckB = static_cast<uint8_t>(ckB + ckA);
    }
    frame[frameLen - 2] = ckA;
    frame[frameLen - 1] = ckB;
}

void M8N::begin(uint32_t baud) {
    // UBX-CFG-PRT: UART1, 8N1, inProto=UBX+NMEA (0x03), outProto=NMEA only (0x02).
    // The baudRate field is filled from the caller so the module's UART speed
    // stays in sync with the open serial port (the diagnostic build can override
    // baud); the checksum is therefore computed at runtime.
    uint8_t cfgPrt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x01, 0x00, 0x00, 0x00,              // portID=1, reserved, txReady
        0xC0, 0x08, 0x00, 0x00,              // mode: 8N1
        0x00, 0x00, 0x00, 0x00,              // baudRate (filled below)
        0x03, 0x00,                          // inProtoMask: UBX+NMEA
        0x02, 0x00,                          // outProtoMask: NMEA only
        0x00, 0x00, 0x00, 0x00,              // flags, reserved
        0x00, 0x00                           // checksum (filled below)
    };
    cfgPrt[14] = static_cast<uint8_t>(baud & 0xFF);
    cfgPrt[15] = static_cast<uint8_t>((baud >> 8) & 0xFF);
    cfgPrt[16] = static_cast<uint8_t>((baud >> 16) & 0xFF);
    cfgPrt[17] = static_cast<uint8_t>((baud >> 24) & 0xFF);
    ubxAppendChecksum(cfgPrt, sizeof(cfgPrt));

    // UBX-CFG-MSG (8-byte payload): enable GGA on UART1 at 1 Hz.
    static const uint8_t cfgMsgGga[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x00,                          // NMEA-GGA
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, // rate=1 on UART1 only
        0x00, 0x28
    };
    // UBX-CFG-MSG (8-byte payload): enable RMC on UART1 at 1 Hz.
    static const uint8_t cfgMsgRmc[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x04,                          // NMEA-RMC
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, // rate=1 on UART1 only
        0x04, 0x44
    };
    // UBX-CFG-RATE: measRate=1000ms, navRate=1, timeRef=GPS.
    // CFG-MSG rate bytes are "once per nav solution", so this must be sent
    // before CFG-MSG to guarantee 1 Hz output regardless of any saved nav rate.
    static const uint8_t cfgRate[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0xE8, 0x03,                          // measRate: 1000 ms
        0x01, 0x00,                          // navRate: 1 solution per measurement
        0x01, 0x00,                          // timeRef: GPS time
        0x01, 0x39
    };

    uart.write(cfgPrt, sizeof(cfgPrt));
    uart.flush();
    delay(100);
    uart.write(cfgRate, sizeof(cfgRate));
    uart.write(cfgMsgGga, sizeof(cfgMsgGga));
    uart.write(cfgMsgRmc, sizeof(cfgMsgRmc));
    uart.flush();
    delay(100);
}

void M8N::saveConfig() {
    // UBX-CFG-CFG: clearMask=0, loadMask=0, deviceMask=0x03 (battery-backed RAM
    // + flash). saveMask=0x0000001F sets only the documented config-section bits
    // -- ioPort(0), msgConf(1), infMsg(2), navConf(3), rxmConf(4) -- which cover
    // the CFG-PRT (baud), CFG-MSG, and CFG-RATE changes we persist. Setting the
    // reserved bits via 0xFFFF risks a NAK on M8, leaving the 9600 setting
    // volatile and reverting on power cycle. Checksum filled at runtime.
    uint8_t cfgCfg[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0x00, 0x00, 0x00, 0x00,              // clearMask
        0x1F, 0x00, 0x00, 0x00,              // saveMask: documented sections 0-4
        0x00, 0x00, 0x00, 0x00,              // loadMask
        0x03,                                // deviceMask: BBR + flash
        0x00, 0x00                           // checksum (filled below)
    };
    ubxAppendChecksum(cfgCfg, sizeof(cfgCfg));
    uart.write(cfgCfg, sizeof(cfgCfg));
    uart.flush();
    delay(100);
}

void M8N::gatherData() {
    while (uart.available()) {
        char c = static_cast<char>(uart.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (!nmeaDiscarding) {
                nmeaBuffer[nmeaBufferIndex] = '\0';
                if (nmeaBufferIndex > 0) {
                    parseNMEA(nmeaBuffer);
                }
            }
            nmeaBufferIndex = 0;
            nmeaBuffer[0] = '\0';
            nmeaDiscarding = false;
        } else if (!nmeaDiscarding) {
            if (nmeaBufferIndex < (NMEA_BUFFER_SIZE - 1)) {
                nmeaBuffer[nmeaBufferIndex++] = c;
            } else {
                // Drop over-length/corrupt sentences rather than fragmenting heap with dynamic strings.
                nmeaBufferIndex = 0;
                nmeaBuffer[0] = '\0';
                nmeaDiscarding = true;
            }
        }
    }
}

void M8N::parseNMEA(char *sentence) {
    if (!validateAndStripChecksum(sentence)) {
        return;
    }

    // Check the sentence type and call the appropriate parser.
    if (strncmp(sentence, "$GNRMC", 6) == 0 || strncmp(sentence, "$GPRMC", 6) == 0) {
        parseRMC(sentence);
    } else if (strncmp(sentence, "$GNGGA", 6) == 0 || strncmp(sentence, "$GPGGA", 6) == 0) {
        parseGGA(sentence);
    }
}

bool M8N::validateAndStripChecksum(char *sentence) {
    if (sentence == nullptr || sentence[0] != '$') {
        return false;
    }

    char *checksumMarker = strchr(sentence, '*');
    if (checksumMarker == nullptr || checksumMarker[1] == '\0' || checksumMarker[2] == '\0') {
        return false;
    }

    uint8_t calculated = 0;
    for (char *cursor = sentence + 1; cursor < checksumMarker; ++cursor) {
        calculated ^= static_cast<uint8_t>(*cursor);
    }

    char checksumText[3] = {checksumMarker[1], checksumMarker[2], '\0'};
    char *end = nullptr;
    unsigned long expected = strtoul(checksumText, &end, 16);
    if (end == checksumText || *end != '\0' || expected > 0xFFUL) {
        return false;
    }

    if (calculated != static_cast<uint8_t>(expected)) {
        return false;
    }

    *checksumMarker = '\0';
    return true;
}

size_t M8N::splitFields(char *sentence, char *fields[], size_t maxFields) {
    size_t fieldCount = 0;
    char *fieldStart = sentence;

    while (fieldCount < maxFields) {
        fields[fieldCount++] = fieldStart;
        char *comma = strchr(fieldStart, ',');
        if (comma == nullptr) {
            break;
        }
        *comma = '\0';
        fieldStart = comma + 1;
    }

    return fieldCount;
}

void M8N::parseRMC(char *sentence) {
    char *parts[NMEA_MAX_FIELDS] = {nullptr};
    size_t partCount = splitFields(sentence, parts, NMEA_MAX_FIELDS);
    if (partCount <= 9) {
        return;
    }

    // Check that data is valid (parts[2] should be "A"). RMC is the best
    // source for speed/course, but some M8N configurations stream GGA without
    // RMC. Do not make GPS lock depend on RMC when GGA already reports a fix.
    if (parts[2][0] == 'A' && parts[2][1] == '\0') {
        rmcDataActive = true;
        const double parsedLatitude = convertToDecimal(parts[3], parts[4][0]);
        const double parsedLongitude = convertToDecimal(parts[5], parts[6][0]);
        const bool currentCoordinatesValid =
            (parsedLatitude != 0.0 || parsedLongitude != 0.0);
        if (currentCoordinatesValid) {
            latitude = parsedLatitude;
            longitude = parsedLongitude;
        } else {
            latitude = 0.0;
            longitude = 0.0;
        }
        speed = (parts[7][0] != '\0') ? atof(parts[7]) : 0.0;
        course = (parts[8][0] != '\0') ? atof(parts[8]) : 0.0;
        has_valid_fix = currentCoordinatesValid &&
                        (fix_quality > 0 || satellites_in_use == 0) &&
                        (satellites_in_use == 0 || satellites_in_use >= MIN_SATELLITES_FOR_FIX);

        // Process timestamp and date (parts[1] HHMMSS.SS, parts[9] DDMMYY).
        if (strlen(parts[1]) >= 6 && strlen(parts[9]) >= 6) {
            timestamp[0] = parts[1][0];
            timestamp[1] = parts[1][1];
            timestamp[2] = ':';
            timestamp[3] = parts[1][2];
            timestamp[4] = parts[1][3];
            timestamp[5] = ':';
            timestamp[6] = parts[1][4];
            timestamp[7] = parts[1][5];
            timestamp[8] = '\0';

            date[0] = '2';
            date[1] = '0';
            date[2] = parts[9][4];
            date[3] = parts[9][5];
            date[4] = '-';
            date[5] = parts[9][2];
            date[6] = parts[9][3];
            date[7] = '-';
            date[8] = parts[9][0];
            date[9] = parts[9][1];
            date[10] = '\0';
        } else {
            timestamp[0] = '\0';
            date[0] = '\0';
        }
    } else {
        rmcDataActive = false;
        // Some receivers can report an inactive/void RMC sentence while GGA
        // still contains a valid fix. Keep the GGA-derived lock state instead
        // of forcing telemetry coordinates back to zero between GGA updates.
        has_valid_fix = (latitude != 0.0 || longitude != 0.0) &&
                        fix_quality > 0 &&
                        satellites_in_use >= MIN_SATELLITES_FOR_FIX;
    }
}

void M8N::parseGGA(char *sentence) {
    char *parts[NMEA_MAX_FIELDS] = {nullptr};
    size_t partCount = splitFields(sentence, parts, NMEA_MAX_FIELDS);
    if (partCount <= 9) {
        return;
    }

    if (parts[6][0] != '\0') {
        fix_quality = atoi(parts[6]);
    }
    if (parts[7][0] != '\0') {
        satellites_in_use = atoi(parts[7]);
    }

    // GGA carries the current fix coordinates. Parse them here as well as in
    // RMC so the flight controller still sends non-zero GPS telemetry when a
    // receiver is configured to output GGA but not RMC. Require coordinates in
    // the current sentence so stale coordinates are not reused after a bad GGA.
    const bool currentFixReported =
        fix_quality > 0 && satellites_in_use >= MIN_SATELLITES_FOR_FIX;
    const double parsedLatitude = convertToDecimal(parts[2], parts[3][0]);
    const double parsedLongitude = convertToDecimal(parts[4], parts[5][0]);
    const bool currentCoordinatesValid =
        (parsedLatitude != 0.0 || parsedLongitude != 0.0);
    if (currentFixReported && currentCoordinatesValid) {
        latitude = parsedLatitude;
        longitude = parsedLongitude;
    } else if (!currentCoordinatesValid) {
        latitude = 0.0;
        longitude = 0.0;
    }

    // Altitude (in meters) is typically in parts[9]; convert to feet.
    if (parts[9][0] != '\0') {
        double altitude_m = atof(parts[9]);
        altitude = altitude_m * 3.28084;
    }

    has_valid_fix = currentFixReported && currentCoordinatesValid;
}

double M8N::convertToDecimal(const char *raw_value, char direction) {
    if (raw_value == nullptr || raw_value[0] == '\0' || direction == '\0') {
        return 0.0;
    }

    int degreeDigits = 0;
    if (direction == 'N' || direction == 'S') {
        degreeDigits = 2;
    } else if (direction == 'E' || direction == 'W') {
        degreeDigits = 3;
    } else {
        return 0.0;
    }

    if (strlen(raw_value) < static_cast<size_t>(degreeDigits)) {
        return 0.0;
    }

    char degreeBuffer[4] = {0};
    memcpy(degreeBuffer, raw_value, degreeDigits);
    int degrees = atoi(degreeBuffer);
    double minutes = atof(raw_value + degreeDigits);
    double decimal = degrees + minutes / 60.0;
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

void M8N::getCoordinates(double &lat, double &lon) {
  lat = (isnan(latitude)) ? 0.0 : latitude;
  lon = (isnan(longitude)) ? 0.0 : longitude;
}
