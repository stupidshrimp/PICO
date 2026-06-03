#include "m8n.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

M8N::M8N(Stream &uart) : latitude(0.0), longitude(0.0),
                         speed(0.0), course(0.0), altitude(0.0),
                         fix_quality(0), satellites_in_use(0),
                         uart(uart), nmeaBufferIndex(0), nmeaDiscarding(false) {
    timestamp[0] = '\0';
    date[0] = '\0';
    nmeaBuffer[0] = '\0';
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
    // Check the sentence type and call the appropriate parser.
    if (strncmp(sentence, "$GNRMC", 6) == 0 || strncmp(sentence, "$GPRMC", 6) == 0) {
        parseRMC(sentence);
    } else if (strncmp(sentence, "$GNGGA", 6) == 0 || strncmp(sentence, "$GPGGA", 6) == 0) {
        parseGGA(sentence);
    }
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

    // Check that data is valid (parts[2] should be "A").
    if (parts[2][0] == 'A' && parts[2][1] == '\0') {
        latitude = convertToDecimal(parts[3], parts[4][0]);
        longitude = convertToDecimal(parts[5], parts[6][0]);
        speed = (parts[7][0] != '\0') ? atof(parts[7]) : 0.0;
        course = (parts[8][0] != '\0') ? atof(parts[8]) : 0.0;

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
    }
}

void M8N::parseGGA(char *sentence) {
    char *parts[NMEA_MAX_FIELDS] = {nullptr};
    size_t partCount = splitFields(sentence, parts, NMEA_MAX_FIELDS);
    if (partCount <= 9) {
        return;
    }

    // Altitude (in meters) is typically in parts[9]; convert to feet.
    if (parts[9][0] != '\0') {
        double altitude_m = atof(parts[9]);
        altitude = altitude_m * 3.28084;
    }
    if (parts[6][0] != '\0') {
        fix_quality = atoi(parts[6]);
    }
    if (parts[7][0] != '\0') {
        satellites_in_use = atoi(parts[7]);
    }
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
