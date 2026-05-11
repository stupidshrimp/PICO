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
    String timestamp;        // Formatted as "HH:MM:SS"
    String date;             // Formatted as "YYYY-MM-DD"
    int fix_quality;         // From GGA sentence
    int satellites_in_use;   // From GGA sentence

private:
    Stream &uart;

    // Parses a complete NMEA sentence
    void parseNMEA(const String &sentence);

    // Parses RMC sentences for latitude, longitude, speed, timestamp, and date
    void parseRMC(const String &sentence);

    // Parses GGA sentences for altitude, fix quality, and satellites in use
    void parseGGA(const String &sentence);

    // Helper: converts raw NMEA coordinate (e.g., "4807.038") and direction ("N" or "S") to decimal degrees
    static double convertToDecimal(const String &raw_value, const String &direction);

    String nmeaBuffer;
};

#endif // M8N_H
