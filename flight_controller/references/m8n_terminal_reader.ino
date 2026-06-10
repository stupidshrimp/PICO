/*************************************************************************************************************
 * M8N GPS Terminal Reader
 *
 * Upload this standalone sketch to the flight-controller board when you want to verify that the u-blox M8N
 * GPS is actually sending data on the GPS UART. It echoes every raw NMEA sentence to the USB Serial Monitor
 * and also prints a compact parsed summary for GGA/RMC sentences so you can check whether latitude/longitude
 * are truly zero, missing, or waiting on a fix.
 *
 * Board wiring used by the main flight-controller sketch:
 *   - GPS RX into board pin PC7 / USART6 RX
 *   - GPS TX from board pin PC6 / USART6 TX
 *   - GPS baud: 9600
 *   - USB terminal baud: 115200
 *
 * Open the Arduino Serial Monitor/terminal at 115200 baud after upload. If wiring and baud are correct you
 * should see RAW lines immediately, even before a GPS fix. FIX, SATS, LAT, and LON will only become meaningful
 * once the module has a valid sky view and starts reporting active GGA/RMC data.
 ************************************************************************************************************/

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#if !defined(USART6)
HardwareSerial Serial6(PC7, PC6);
#endif

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t GPS_BAUD = 9600;
static constexpr size_t NMEA_BUFFER_SIZE = 121;
static constexpr size_t NMEA_MAX_FIELDS = 20;
static constexpr uint32_t STATUS_PERIOD_MS = 1000;
static constexpr uint32_t NO_DATA_WARNING_MS = 3000;

char nmeaBuffer[NMEA_BUFFER_SIZE];
size_t nmeaBufferIndex = 0;
bool nmeaDiscarding = false;

uint32_t gpsByteCount = 0;
uint32_t nmeaSentenceCount = 0;
uint32_t validChecksumCount = 0;
uint32_t checksumFailCount = 0;
uint32_t overLengthSentenceCount = 0;
uint32_t lastByteMillis = 0;
uint32_t lastStatusMillis = 0;

bool validateAndStripChecksum(char *sentence) {
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
  const unsigned long expected = strtoul(checksumText, &end, 16);
  if (end == checksumText || *end != '\0' || expected > 0xFFUL) {
    return false;
  }

  if (calculated != static_cast<uint8_t>(expected)) {
    return false;
  }

  *checksumMarker = '\0';
  return true;
}

size_t splitFields(char *sentence, char *fields[], size_t maxFields) {
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

double convertNmeaCoordinateToDecimal(const char *rawValue, char direction) {
  if (rawValue == nullptr || rawValue[0] == '\0') {
    return 0.0;
  }

  char *end = nullptr;
  const double nmeaValue = strtod(rawValue, &end);
  if (end == rawValue) {
    return 0.0;
  }

  const int degrees = static_cast<int>(nmeaValue / 100.0);
  const double minutes = nmeaValue - (static_cast<double>(degrees) * 100.0);
  double decimal = static_cast<double>(degrees) + (minutes / 60.0);

  if (direction == 'S' || direction == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

void printParsedGga(char *fields[], size_t fieldCount) {
  if (fieldCount <= 9) {
    Serial.println(F("PARSED GGA: incomplete sentence"));
    return;
  }

  const int fixQuality = atoi(fields[6]);
  const int satellites = atoi(fields[7]);
  const double latitude = convertNmeaCoordinateToDecimal(fields[2], fields[3][0]);
  const double longitude = convertNmeaCoordinateToDecimal(fields[4], fields[5][0]);
  const double altitudeMeters = strtod(fields[9], nullptr);

  Serial.print(F("PARSED GGA: fix="));
  Serial.print(fixQuality);
  Serial.print(F(" sats="));
  Serial.print(satellites);
  Serial.print(F(" lat="));
  Serial.print(latitude, 8);
  Serial.print(F(" lon="));
  Serial.print(longitude, 8);
  Serial.print(F(" alt_m="));
  Serial.print(altitudeMeters, 2);
  Serial.print(F(" raw_lat="));
  Serial.print(fields[2]);
  Serial.print(fields[3]);
  Serial.print(F(" raw_lon="));
  Serial.print(fields[4]);
  Serial.println(fields[5]);
}

void printParsedRmc(char *fields[], size_t fieldCount) {
  if (fieldCount <= 9) {
    Serial.println(F("PARSED RMC: incomplete sentence"));
    return;
  }

  const bool activeFix = fields[2][0] == 'A';
  const double latitude = convertNmeaCoordinateToDecimal(fields[3], fields[4][0]);
  const double longitude = convertNmeaCoordinateToDecimal(fields[5], fields[6][0]);

  Serial.print(F("PARSED RMC: status="));
  Serial.print(activeFix ? F("A(active)") : F("V(void)"));
  Serial.print(F(" lat="));
  Serial.print(latitude, 8);
  Serial.print(F(" lon="));
  Serial.print(longitude, 8);
  Serial.print(F(" speed_knots="));
  Serial.print(fields[7]);
  Serial.print(F(" course_deg="));
  Serial.print(fields[8]);
  Serial.print(F(" utc="));
  Serial.print(fields[1]);
  Serial.print(F(" date="));
  Serial.println(fields[9]);
}

void parseAndPrintSentence(char *sentence) {
  Serial.print(F("RAW: "));
  Serial.println(sentence);
  ++nmeaSentenceCount;

  if (!validateAndStripChecksum(sentence)) {
    ++checksumFailCount;
    Serial.println(F("PARSED: checksum failed or missing; verify baud/wiring/noise if this repeats"));
    return;
  }

  ++validChecksumCount;

  char *fields[NMEA_MAX_FIELDS] = {nullptr};
  const size_t fieldCount = splitFields(sentence, fields, NMEA_MAX_FIELDS);

  if (strncmp(fields[0], "$GNGGA", 6) == 0 || strncmp(fields[0], "$GPGGA", 6) == 0) {
    printParsedGga(fields, fieldCount);
  } else if (strncmp(fields[0], "$GNRMC", 6) == 0 || strncmp(fields[0], "$GPRMC", 6) == 0) {
    printParsedRmc(fields, fieldCount);
  } else {
    Serial.print(F("PARSED: valid "));
    Serial.print(fields[0]);
    Serial.println(F(" sentence"));
  }
}

void printStatusHeartbeat() {
  const uint32_t now = millis();
  if (now - lastStatusMillis < STATUS_PERIOD_MS) {
    return;
  }
  lastStatusMillis = now;

  Serial.print(F("STATUS: bytes="));
  Serial.print(gpsByteCount);
  Serial.print(F(" sentences="));
  Serial.print(nmeaSentenceCount);
  Serial.print(F(" checksum_ok="));
  Serial.print(validChecksumCount);
  Serial.print(F(" checksum_fail="));
  Serial.print(checksumFailCount);
  Serial.print(F(" overlength="));
  Serial.print(overLengthSentenceCount);
  Serial.print(F(" ms_since_last_byte="));
  Serial.println(now - lastByteMillis);

  if (gpsByteCount == 0 || now - lastByteMillis > NO_DATA_WARNING_MS) {
    Serial.println(F("WARNING: no recent GPS bytes. Check M8N power, ground, TX/RX crossing, USART6 pins, and 9600 baud."));
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  Serial6.begin(GPS_BAUD);
  lastByteMillis = millis();

  Serial.println();
  Serial.println(F("M8N GPS Terminal Reader started"));
  Serial.println(F("USB terminal: 115200 baud"));
  Serial.println(F("GPS UART: USART6 / Serial6 at 9600 baud"));
  Serial.println(F("Waiting for raw NMEA data..."));
}

void loop() {
  while (Serial6.available() > 0) {
    const char c = static_cast<char>(Serial6.read());
    ++gpsByteCount;
    lastByteMillis = millis();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (!nmeaDiscarding) {
        nmeaBuffer[nmeaBufferIndex] = '\0';
        if (nmeaBufferIndex > 0) {
          parseAndPrintSentence(nmeaBuffer);
        }
      }
      nmeaBufferIndex = 0;
      nmeaBuffer[0] = '\0';
      nmeaDiscarding = false;
      continue;
    }

    if (!nmeaDiscarding) {
      if (nmeaBufferIndex < NMEA_BUFFER_SIZE - 1) {
        nmeaBuffer[nmeaBufferIndex++] = c;
      } else {
        ++overLengthSentenceCount;
        nmeaBufferIndex = 0;
        nmeaBuffer[0] = '\0';
        nmeaDiscarding = true;
      }
    }
  }

  printStatusHeartbeat();
}
