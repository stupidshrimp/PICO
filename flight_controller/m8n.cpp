#include "m8n.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

M8N::M8N(Stream &uart) : latitude(0.0), longitude(0.0),
                         speed(0.0), course(0.0), altitude(0.0),
                         fix_quality(0), satellites_in_use(0), has_valid_fix(false),
                         valid_sentence_count(0), checksum_error_count(0),
                         unsupported_sentence_count(0), ubx_nav_pvt_count(0),
                         ubx_checksum_error_count(0), last_valid_sentence_ms(0),
                         uart(uart), nmeaBufferIndex(0), nmeaDiscarding(false), rmcDataActive(false),
                         ubxState(0), ubxMessageClass(0), ubxMessageId(0),
                         ubxPayloadLength(0), ubxPayloadIndex(0),
                         ubxChecksumA(0), ubxChecksumB(0) {
    timestamp[0] = '\0';
    date[0] = '\0';
    nmeaBuffer[0] = '\0';
}

void M8N::resetParser(bool resetData) {
    nmeaBufferIndex = 0;
    nmeaBuffer[0] = '\0';
    nmeaDiscarding = false;
    resetUbxParser();

    if (!resetData) {
        return;
    }

    latitude = 0.0;
    longitude = 0.0;
    speed = 0.0;
    course = 0.0;
    altitude = 0.0;
    fix_quality = 0;
    satellites_in_use = 0;
    has_valid_fix = false;
    valid_sentence_count = 0;
    checksum_error_count = 0;
    unsupported_sentence_count = 0;
    ubx_nav_pvt_count = 0;
    ubx_checksum_error_count = 0;
    last_valid_sentence_ms = 0;
    rmcDataActive = false;
    timestamp[0] = '\0';
    date[0] = '\0';
}

void M8N::gatherData() {
    while (uart.available()) {
        uint8_t rawByte = static_cast<uint8_t>(uart.read());
        processUbxByte(rawByte);

        char c = static_cast<char>(rawByte);
        if (c == '$') {
            nmeaBufferIndex = 0;
            nmeaBuffer[nmeaBufferIndex++] = c;
            nmeaDiscarding = false;
            continue;
        }

        // Ignore UBX/binary/noise bytes until a real NMEA sentence starts.
        if (nmeaBufferIndex == 0) {
            continue;
        }

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
    if (sentence == nullptr || strlen(sentence) < 6) {
        ++checksum_error_count;
        return;
    }

    if (!validateAndStripChecksum(sentence)) {
        ++checksum_error_count;
        return;
    }

    // Check the sentence type and call the appropriate parser. Accept talker IDs
    // commonly emitted by u-blox M8N modules in GPS-only and multi-GNSS modes.
    if (strncmp(sentence + 3, "RMC", 3) == 0) {
        ++valid_sentence_count;
        last_valid_sentence_ms = millis();
        parseRMC(sentence);
    } else if (strncmp(sentence + 3, "GGA", 3) == 0) {
        ++valid_sentence_count;
        last_valid_sentence_ms = millis();
        parseGGA(sentence);
    } else if (strncmp(sentence + 3, "GNS", 3) == 0) {
        ++valid_sentence_count;
        last_valid_sentence_ms = millis();
        parseGNS(sentence);
    } else if (strncmp(sentence + 3, "GLL", 3) == 0) {
        ++valid_sentence_count;
        last_valid_sentence_ms = millis();
        parseGLL(sentence);
    } else {
        ++unsupported_sentence_count;
    }
}


void M8N::resetUbxParser() {
    ubxState = 0;
    ubxMessageClass = 0;
    ubxMessageId = 0;
    ubxPayloadLength = 0;
    ubxPayloadIndex = 0;
    ubxChecksumA = 0;
    ubxChecksumB = 0;
}

void M8N::processUbxByte(uint8_t byte) {
    auto updateChecksum = [this](uint8_t value) {
        ubxChecksumA = static_cast<uint8_t>(ubxChecksumA + value);
        ubxChecksumB = static_cast<uint8_t>(ubxChecksumB + ubxChecksumA);
    };

    switch (ubxState) {
        case 0: // sync char 1
            ubxState = (byte == 0xB5) ? 1 : 0;
            break;

        case 1: // sync char 2
            if (byte == 0x62) {
                ubxState = 2;
                ubxChecksumA = 0;
                ubxChecksumB = 0;
            } else {
                ubxState = (byte == 0xB5) ? 1 : 0;
            }
            break;

        case 2: // class
            ubxMessageClass = byte;
            updateChecksum(byte);
            ubxState = 3;
            break;

        case 3: // id
            ubxMessageId = byte;
            updateChecksum(byte);
            ubxState = 4;
            break;

        case 4: // length LSB
            ubxPayloadLength = byte;
            updateChecksum(byte);
            ubxState = 5;
            break;

        case 5: // length MSB
            ubxPayloadLength |= static_cast<uint16_t>(byte) << 8;
            updateChecksum(byte);
            ubxPayloadIndex = 0;
            if (ubxPayloadLength > UBX_MAX_PAYLOAD_SIZE) {
                resetUbxParser();
            } else {
                ubxState = (ubxPayloadLength == 0) ? 7 : 6;
            }
            break;

        case 6: // payload
            ubxPayload[ubxPayloadIndex++] = byte;
            updateChecksum(byte);
            if (ubxPayloadIndex >= ubxPayloadLength) {
                ubxState = 7;
            }
            break;

        case 7: // checksum A
            if (byte == ubxChecksumA) {
                ubxState = 8;
            } else {
                ++ubx_checksum_error_count;
                resetUbxParser();
            }
            break;

        case 8: // checksum B
            if (byte == ubxChecksumB) {
                parseUbxMessage(ubxMessageClass, ubxMessageId, ubxPayload, ubxPayloadLength);
            } else {
                ++ubx_checksum_error_count;
            }
            resetUbxParser();
            break;

        default:
            resetUbxParser();
            break;
    }
}

static int32_t readI32LE(const uint8_t *payload, uint16_t offset) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(payload[offset])) |
        (static_cast<uint32_t>(payload[offset + 1]) << 8) |
        (static_cast<uint32_t>(payload[offset + 2]) << 16) |
        (static_cast<uint32_t>(payload[offset + 3]) << 24));
}

void M8N::parseUbxMessage(uint8_t messageClass, uint8_t messageId, const uint8_t *payload, uint16_t length) {
    // NAV-PVT gives fix status, coordinates, satellite count, speed, course, and altitude
    // in one checksum-protected binary message. Some M8N modules are configured for UBX
    // output only, so accepting this message prevents a blue-fix module from reporting
    // zero GPS coordinates just because NMEA is disabled.
    if (messageClass != 0x01 || messageId != 0x07 || length < 92) {
        return;
    }

    const uint8_t fixType = payload[20];
    const uint8_t flags = payload[21];
    const uint8_t numSv = payload[23];
    const bool gnssFixOk = (flags & 0x01) != 0;
    const bool fixUsable = gnssFixOk && fixType >= 2 && numSv >= MIN_SATELLITES_FOR_FIX;

    satellites_in_use = numSv;
    fix_quality = fixUsable ? static_cast<int>(fixType) : 0;

    const int32_t lonRaw = readI32LE(payload, 24);
    const int32_t latRaw = readI32LE(payload, 28);
    const bool currentCoordinatesValid = (latRaw != 0 || lonRaw != 0);

    if (fixUsable && currentCoordinatesValid) {
        longitude = static_cast<double>(lonRaw) / 10000000.0;
        latitude = static_cast<double>(latRaw) / 10000000.0;
    } else if (!currentCoordinatesValid) {
        longitude = 0.0;
        latitude = 0.0;
    }

    const int32_t hMslMm = readI32LE(payload, 36);
    altitude = (static_cast<double>(hMslMm) / 1000.0) * 3.28084;

    const int32_t groundSpeedMms = readI32LE(payload, 60);
    speed = (static_cast<double>(groundSpeedMms) / 1000.0) * 1.943844492;

    const int32_t headMotRaw = readI32LE(payload, 64);
    course = static_cast<double>(headMotRaw) / 100000.0;
    if (course < 0.0) {
        course += 360.0;
    }

    has_valid_fix = fixUsable && currentCoordinatesValid;
    ++ubx_nav_pvt_count;
    last_valid_sentence_ms = millis();
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


void M8N::parseGNS(char *sentence) {
    char *parts[NMEA_MAX_FIELDS] = {nullptr};
    size_t partCount = splitFields(sentence, parts, NMEA_MAX_FIELDS);
    if (partCount <= 9) {
        return;
    }

    if (parts[7][0] != '\0') {
        satellites_in_use = atoi(parts[7]);
    }

    bool modeReportsFix = false;
    for (const char *mode = parts[6]; mode != nullptr && *mode != '\0'; ++mode) {
        if (*mode != 'N') {
            modeReportsFix = true;
            break;
        }
    }
    fix_quality = modeReportsFix ? 1 : 0;

    const bool currentFixReported =
        modeReportsFix && satellites_in_use >= MIN_SATELLITES_FOR_FIX;
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

    if (parts[9][0] != '\0') {
        double altitude_m = atof(parts[9]);
        altitude = altitude_m * 3.28084;
    }

    has_valid_fix = currentFixReported && currentCoordinatesValid;
}


void M8N::parseGLL(char *sentence) {
    char *parts[NMEA_MAX_FIELDS] = {nullptr};
    size_t partCount = splitFields(sentence, parts, NMEA_MAX_FIELDS);
    if (partCount <= 6) {
        return;
    }

    const bool statusValid = parts[6][0] == 'A' && parts[6][1] == '\0';
    const double parsedLatitude = convertToDecimal(parts[1], parts[2][0]);
    const double parsedLongitude = convertToDecimal(parts[3], parts[4][0]);
    const bool currentCoordinatesValid =
        (parsedLatitude != 0.0 || parsedLongitude != 0.0);

    if (statusValid && currentCoordinatesValid) {
        latitude = parsedLatitude;
        longitude = parsedLongitude;
    } else if (!currentCoordinatesValid) {
        latitude = 0.0;
        longitude = 0.0;
    }

    has_valid_fix = statusValid && currentCoordinatesValid &&
                    (satellites_in_use == 0 || satellites_in_use >= MIN_SATELLITES_FOR_FIX);
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
