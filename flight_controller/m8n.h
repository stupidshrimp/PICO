#ifndef M8N_H
#define M8N_H

#include <Arduino.h>

class M8N {
public:
    // Constructor: takes a reference to a Stream (e.g., Serial1, etc.)
    M8N(Stream &uart);

    // Call this method repeatedly (e.g. in loop()) to process incoming data
    void gatherData();

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
    bool has_valid_fix;      // True when RMC/GGA data passed checksum and fix gates.

private:
    static constexpr size_t NMEA_BUFFER_SIZE = 121;
    static constexpr size_t NMEA_MAX_FIELDS = 20;
    static constexpr int MIN_SATELLITES_FOR_FIX = 4;

    Stream &uart;

    // Parses a complete NMEA sentence. The buffer may be modified during parsing.
    void parseNMEA(char *sentence);

    // Verifies and strips the NMEA checksum suffix in-place before field parsing.
    static bool validateAndStripChecksum(char *sentence);

    // Parses RMC sentences for latitude, longitude, speed, timestamp, and date.
    void parseRMC(char *sentence);

    // Parses GGA sentences for altitude, fix quality, and satellites in use.
    void parseGGA(char *sentence);

    // Splits a comma-separated NMEA sentence into field pointers in-place.
    static size_t splitFields(char *sentence, char *fields[], size_t maxFields);

    // Helper: converts raw NMEA coordinate (e.g., "4807.038") and direction ('N' or 'S') to decimal degrees.
    static double convertToDecimal(const char *raw_value, char direction);

    char nmeaBuffer[NMEA_BUFFER_SIZE];
    size_t nmeaBufferIndex;
    bool nmeaDiscarding;
    bool rmcDataActive;
};

#endif // M8N_H
