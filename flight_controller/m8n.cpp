#include "m8n.h"
#include <math.h>

M8N::M8N(Stream &uart) : uart(uart), latitude(0.0), longitude(0.0),
                         speed(0.0), course(0.0), altitude(0.0),
                         fix_quality(0), satellites_in_use(0) {
    nmeaBuffer.reserve(120);
}

void M8N::gatherData() {
    while (uart.available()) {
        char c = static_cast<char>(uart.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            String rawData = nmeaBuffer;
            nmeaBuffer = "";
            rawData.trim();
            if (rawData.length() > 0) {
                parseNMEA(rawData);
            }
        } else {
            if (nmeaBuffer.length() < 120) {
                nmeaBuffer += c;
            } else {
                nmeaBuffer = "";
            }
        }
    }
}

void M8N::parseNMEA(const String &sentence) {
    // Check the sentence type and call the appropriate parser
    if (sentence.startsWith("$GNRMC") || sentence.startsWith("$GPRMC")) {
        parseRMC(sentence);
    } else if (sentence.startsWith("$GNGGA") || sentence.startsWith("$GPGGA")) {
        parseGGA(sentence);
    }
}

void M8N::parseRMC(const String &sentence) {
    // Split the sentence by commas.
    const int maxParts = 20;
    String parts[maxParts];
    int partIndex = 0;
    int start = 0;
    int commaIndex = sentence.indexOf(',');
    while (commaIndex != -1 && partIndex < maxParts) {
        parts[partIndex++] = sentence.substring(start, commaIndex);
        start = commaIndex + 1;
        commaIndex = sentence.indexOf(',', start);
    }
    if (partIndex < maxParts) {
        parts[partIndex++] = sentence.substring(start);
    }

    // Check that data is valid (parts[2] should be "A")
    if (parts[2] == "A") {
        // Convert latitude and longitude to decimal degrees
        latitude = convertToDecimal(parts[3], parts[4]);
        longitude = convertToDecimal(parts[5], parts[6]);

        // Speed is provided in parts[7]
        if (parts[7].length() > 0) {
            speed = parts[7].toDouble();
        } else {
            speed = 0.0;
        }

        // Course over ground is provided in parts[8]
        if (parts[8].length() > 0) {
            course = parts[8].toDouble();
        } else {
            course = 0.0;
        }

        // Process timestamp and date (parts[1] and parts[9])
        String raw_time = parts[1];  // Expected format HHMMSS.SS
        String raw_date = parts[9];  // Expected format DDMMYY
        if (raw_time.length() >= 6 && raw_date.length() >= 6) {
            timestamp = raw_time.substring(0, 2) + ":" + raw_time.substring(2, 4) + ":" + raw_time.substring(4, 6);
            int day = raw_date.substring(0, 2).toInt();
            int month = raw_date.substring(2, 4).toInt();
            int year = 2000 + raw_date.substring(4, 6).toInt();
            char dateBuffer[11];
            snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", year, month, day);
            date = String(dateBuffer);
        } else {
            timestamp = "";
            date = "";
        }
    }
}

void M8N::parseGGA(const String &sentence) {
    const int maxParts = 20;
    String parts[maxParts];
    int partIndex = 0;
    int start = 0;
    int commaIndex = sentence.indexOf(',');
    while (commaIndex != -1 && partIndex < maxParts) {
        parts[partIndex++] = sentence.substring(start, commaIndex);
        start = commaIndex + 1;
        commaIndex = sentence.indexOf(',', start);
    }
    if (partIndex < maxParts) {
        parts[partIndex++] = sentence.substring(start);
    }

    // Altitude (in meters) is typically in parts[9]; convert to feet.
    if (parts[9].length() > 0) {
        double altitude_m = parts[9].toDouble();
        altitude = altitude_m * 3.28084;
    }
    if (parts[6].length() > 0) {
        fix_quality = parts[6].toInt();
    }
    if (parts[7].length() > 0) {
        satellites_in_use = parts[7].toInt();
    }
}

double M8N::convertToDecimal(const String &raw_value, const String &direction) {
    if (raw_value.length() == 0 || direction.length() == 0) {
        return 0.0;  // Alternatively, you might return NAN
    }

    int degrees = 0;
    double minutes = 0.0;
    if (direction == "N" || direction == "S") {
        // Latitude: degrees are first two digits
        if (raw_value.length() < 2) return 0.0;
        degrees = raw_value.substring(0, 2).toInt();
        minutes = raw_value.substring(2).toDouble();
    } else if (direction == "E" || direction == "W") {
        // Longitude: degrees are first three digits
        if (raw_value.length() < 3) return 0.0;
        degrees = raw_value.substring(0, 3).toInt();
        minutes = raw_value.substring(3).toDouble();
    }
    double decimal = degrees + minutes / 60.0;
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }
    return decimal;
}

void M8N::getCoordinates(double &lat, double &lon) {
  lat = (isnan(latitude)) ? 0.0 : latitude;
  lon = (isnan(longitude)) ? 0.0 : longitude;
}