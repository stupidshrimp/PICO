#ifndef M8N_H
#define M8N_H

#include <Arduino.h>

class M8N {
public:
    // Constructor: takes a reference to a Stream (e.g., Serial1, etc.)
    M8N(Stream &uart);

    // Call this method repeatedly (e.g. in loop()) to process incoming data
    void gatherData();

    // Clears partial NMEA input and, optionally, all parsed GPS state/counters.
    void resetParser(bool resetData = false);

    // Returns current coordinates via reference parameters
    void getCoordinates(double &latitude, double &longitude);

    // Public GPS data members
    double latitude;         // Decimal degrees
    double longitude;        // Decimal degrees
    double speed;            // As provided in the RMC sentence (typically knots)
    double course;           // Ground course in degrees
    double altitude;         // In feet (from GGA sentence)
    char timestamp[9];       // Formatted as "HH:MM:SS"
    char date[11];           // Formatted as "YYYY-MM-DD"
    int fix_quality;         // From GGA sentence
    int satellites_in_use;   // From GGA sentence
    bool has_valid_fix;      // True when NMEA/UBX data passed checksum and fix gates.
    uint32_t valid_sentence_count;      // Number of checksum-valid supported NMEA sentences parsed.
    uint32_t checksum_error_count;      // Number of complete NMEA sentences rejected by checksum/format.
    uint32_t unsupported_sentence_count;// Number of checksum-valid NMEA sentences not used by this parser.
    uint32_t ubx_nav_pvt_count;         // Number of checksum-valid UBX NAV-PVT messages parsed.
    uint32_t ubx_checksum_error_count;  // Number of UBX messages rejected by checksum.
    uint32_t last_valid_sentence_ms;    // millis() when the last checksum-valid supported sentence arrived.

private:
    static constexpr size_t NMEA_BUFFER_SIZE = 121;
    static constexpr size_t NMEA_MAX_FIELDS = 20;
    static constexpr size_t UBX_MAX_PAYLOAD_SIZE = 100;
    static constexpr int MIN_SATELLITES_FOR_FIX = 4;

    Stream &uart;

    // Parses a complete NMEA sentence. The buffer may be modified during parsing.
    void parseNMEA(char *sentence);

    // Incrementally decodes UBX binary messages that some M8N modules emit instead of NMEA.
    void processUbxByte(uint8_t byte);
    void resetUbxParser();
    void parseUbxMessage(uint8_t messageClass, uint8_t messageId, const uint8_t *payload, uint16_t length);

    // Verifies and strips the NMEA checksum suffix in-place before field parsing.
    static bool validateAndStripChecksum(char *sentence);

    // Parses RMC sentences for latitude, longitude, speed, timestamp, and date.
    void parseRMC(char *sentence);

    // Parses GGA sentences for altitude, fix quality, and satellites in use.
    void parseGGA(char *sentence);

    // Parses GNS sentences for combined multi-GNSS fix, coordinates, altitude, and satellites.
    void parseGNS(char *sentence);

    // Parses GLL sentences for geographic position when richer sentences are disabled.
    void parseGLL(char *sentence);

    // Splits a comma-separated NMEA sentence into field pointers in-place.
    static size_t splitFields(char *sentence, char *fields[], size_t maxFields);

    // Helper: converts raw NMEA coordinate (e.g., "4807.038") and direction ('N' or 'S') to decimal degrees.
    static double convertToDecimal(const char *raw_value, char direction);

    char nmeaBuffer[NMEA_BUFFER_SIZE];
    size_t nmeaBufferIndex;
    bool nmeaDiscarding;
    bool rmcDataActive;
    uint8_t ubxState;
    uint8_t ubxMessageClass;
    uint8_t ubxMessageId;
    uint16_t ubxPayloadLength;
    uint16_t ubxPayloadIndex;
    uint8_t ubxChecksumA;
    uint8_t ubxChecksumB;
    uint8_t ubxPayload[UBX_MAX_PAYLOAD_SIZE];
};

#endif // M8N_H
