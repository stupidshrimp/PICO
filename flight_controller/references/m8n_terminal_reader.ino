/*************************************************************************************************************
 * M8N GPS Terminal Reader
 *
 * Upload this standalone sketch to the flight-controller board when you want to verify that the u-blox M8N
 * GPS is actually sending data on the GPS UART. It echoes every raw NMEA sentence to the USB Serial Monitor
 * and also prints a compact parsed summary for GGA/RMC sentences so you can check whether latitude/longitude
 * are truly zero, missing, or waiting on a fix.
 *
 * Board wiring used by the main flight-controller sketch:
 *   - GPS TX into board pin PC7 / USART6 RX
 *   - GPS RX from board pin PC6 / USART6 TX
 *   - GPS baud: 9600
 *   - USB terminal baud: 115200
 *
 * Open the Arduino Serial Monitor/terminal at 115200 baud after upload. If wiring and baud are correct you
 * should see RAW lines immediately, even before a GPS fix. FIX, SATS, LAT, and LON will only become meaningful
 * once the module has a valid sky view and starts reporting active GGA/RMC data.
 *
 * Extra diagnostics in this sketch:
 *   - Prints whether board PC7/USART6 RX is sitting HIGH or LOW before the UART is started. A healthy idle UART
 *     line is normally HIGH; a stuck LOW line can point to a short, swapped wiring, or an unpowered module.
 *   - Sends periodic u-blox/NMEA polls ("PING") so the module has a reason to answer even if automatic NMEA
 *     output is disabled. Any byte received after a ping proves that PC7 is seeing signal from the GPS TX pin.
 *   - Prints short HEX samples for non-NMEA/binary traffic so you can tell the difference between no signal,
 *     baud mismatch/noise, and a real UBX response.
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
static constexpr uint32_t PING_PERIOD_MS = 5000;
static constexpr uint32_t POST_PING_WINDOW_MS = 1500;
static constexpr size_t HEX_SAMPLE_BYTES = 16;
static constexpr uint8_t UBX_SYNC_1 = 0xB5;
static constexpr uint8_t UBX_SYNC_2 = 0x62;
static constexpr uint16_t UBX_MAX_PAYLOAD_LENGTH = 512;

char nmeaBuffer[NMEA_BUFFER_SIZE];
size_t nmeaBufferIndex = 0;
bool nmeaDiscarding = false;
bool nmeaReceiving = false;

uint32_t gpsByteCount = 0;
uint32_t nmeaSentenceCount = 0;
uint32_t validChecksumCount = 0;
uint32_t checksumFailCount = 0;
uint32_t overLengthSentenceCount = 0;
uint32_t binaryByteCount = 0;
uint32_t ubxFrameCount = 0;
uint32_t ubxChecksumFailCount = 0;
uint32_t monVerResponseCount = 0;
uint32_t pingCount = 0;
uint32_t bytesAtLastPing = 0;
uint32_t lastPingMillis = 0;
uint32_t lastByteMillis = 0;
uint32_t lastStatusMillis = 0;
uint8_t hexSample[HEX_SAMPLE_BYTES];
size_t hexSampleCount = 0;

enum class UbxParseState {
  Sync1,
  Sync2,
  Class,
  Id,
  Length1,
  Length2,
  Payload,
  ChecksumA,
  ChecksumB
};

UbxParseState ubxState = UbxParseState::Sync1;
uint8_t ubxClass = 0;
uint8_t ubxId = 0;
uint16_t ubxPayloadLength = 0;
uint16_t ubxPayloadIndex = 0;
uint8_t ubxChecksumA = 0;
uint8_t ubxChecksumB = 0;
uint8_t ubxReceivedChecksumA = 0;

void addUbxChecksumByte(uint8_t value) {
  ubxChecksumA = static_cast<uint8_t>(ubxChecksumA + value);
  ubxChecksumB = static_cast<uint8_t>(ubxChecksumB + ubxChecksumA);
}

void resetUbxParser() {
  ubxState = UbxParseState::Sync1;
  ubxClass = 0;
  ubxId = 0;
  ubxPayloadLength = 0;
  ubxPayloadIndex = 0;
  ubxChecksumA = 0;
  ubxChecksumB = 0;
  ubxReceivedChecksumA = 0;
}

void updateUbxParser(uint8_t value) {
  switch (ubxState) {
    case UbxParseState::Sync1:
      ubxState = (value == UBX_SYNC_1) ? UbxParseState::Sync2 : UbxParseState::Sync1;
      break;
    case UbxParseState::Sync2:
      if (value == UBX_SYNC_2) {
        ubxState = UbxParseState::Class;
        ubxChecksumA = 0;
        ubxChecksumB = 0;
      } else {
        ubxState = (value == UBX_SYNC_1) ? UbxParseState::Sync2 : UbxParseState::Sync1;
      }
      break;
    case UbxParseState::Class:
      ubxClass = value;
      addUbxChecksumByte(value);
      ubxState = UbxParseState::Id;
      break;
    case UbxParseState::Id:
      ubxId = value;
      addUbxChecksumByte(value);
      ubxState = UbxParseState::Length1;
      break;
    case UbxParseState::Length1:
      ubxPayloadLength = value;
      addUbxChecksumByte(value);
      ubxState = UbxParseState::Length2;
      break;
    case UbxParseState::Length2:
      ubxPayloadLength |= static_cast<uint16_t>(value) << 8;
      addUbxChecksumByte(value);
      ubxPayloadIndex = 0;
      if (ubxPayloadLength > UBX_MAX_PAYLOAD_LENGTH) {
        ++ubxChecksumFailCount;
        resetUbxParser();
      } else {
        ubxState = (ubxPayloadLength == 0) ? UbxParseState::ChecksumA : UbxParseState::Payload;
      }
      break;
    case UbxParseState::Payload:
      addUbxChecksumByte(value);
      ++ubxPayloadIndex;
      if (ubxPayloadIndex >= ubxPayloadLength) {
        ubxState = UbxParseState::ChecksumA;
      }
      break;
    case UbxParseState::ChecksumA:
      ubxReceivedChecksumA = value;
      ubxState = UbxParseState::ChecksumB;
      break;
    case UbxParseState::ChecksumB:
      if (ubxReceivedChecksumA == ubxChecksumA && value == ubxChecksumB) {
        ++ubxFrameCount;
        Serial.print(F("UBX: class=0x"));
        if (ubxClass < 0x10) {
          Serial.print('0');
        }
        Serial.print(ubxClass, HEX);
        Serial.print(F(" id=0x"));
        if (ubxId < 0x10) {
          Serial.print('0');
        }
        Serial.print(ubxId, HEX);
        Serial.print(F(" payload_len="));
        Serial.println(ubxPayloadLength);
        if (ubxClass == 0x0A && ubxId == 0x04) {
          ++monVerResponseCount;
          Serial.println(F("PING RESULT: M8N answered UBX-MON-VER poll on this UART."));
        }
      } else {
        ++ubxChecksumFailCount;
      }
      resetUbxParser();
      break;
  }
}

void printHexSample(const char *reason) {
  if (hexSampleCount == 0) {
    return;
  }

  Serial.print(F("SIGNAL "));
  Serial.print(reason);
  Serial.print(F(": "));
  for (size_t i = 0; i < hexSampleCount; ++i) {
    if (hexSample[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(hexSample[i], HEX);
    Serial.print(' ');
  }
  Serial.print(F(" |ascii| "));
  for (size_t i = 0; i < hexSampleCount; ++i) {
    const char c = static_cast<char>(hexSample[i]);
    Serial.print((c >= 32 && c <= 126) ? c : '.');
  }
  Serial.println();
  hexSampleCount = 0;
}

void recordHexSample(uint8_t value) {
  if (hexSampleCount < HEX_SAMPLE_BYTES) {
    hexSample[hexSampleCount++] = value;
  }
  if (hexSampleCount >= HEX_SAMPLE_BYTES) {
    printHexSample("non-NMEA bytes");
  }
}

void sendGpsPing() {
  static const uint8_t ubxMonVerPoll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
  static const char nmeaPubxPositionPoll[] = "$PUBX,00*33\r\n";

  Serial6.write(ubxMonVerPoll, sizeof(ubxMonVerPoll));
  Serial6.print(nmeaPubxPositionPoll);
  Serial6.flush();

  ++pingCount;
  bytesAtLastPing = gpsByteCount;
  lastPingMillis = millis();
  Serial.print(F("PING: sent UBX-MON-VER poll and PUBX position poll #"));
  Serial.println(pingCount);
}

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
  Serial.print(F(" binary_bytes="));
  Serial.print(binaryByteCount);
  Serial.print(F(" ubx_ok="));
  Serial.print(ubxFrameCount);
  Serial.print(F(" ubx_fail="));
  Serial.print(ubxChecksumFailCount);
  Serial.print(F(" mon_ver_answers="));
  Serial.print(monVerResponseCount);
  Serial.print(F(" ms_since_last_byte="));
  Serial.println(now - lastByteMillis);

  if (lastPingMillis != 0 && now - lastPingMillis >= POST_PING_WINDOW_MS && now - lastPingMillis < POST_PING_WINDOW_MS + STATUS_PERIOD_MS) {
    Serial.print(F("PING RESULT: bytes_after_last_ping="));
    Serial.print(gpsByteCount - bytesAtLastPing);
    Serial.println((gpsByteCount > bytesAtLastPing) ? F(" (RX pin saw activity)") : F(" (no reply/activity seen)"));
  }

  if (gpsByteCount == 0 || now - lastByteMillis > NO_DATA_WARNING_MS) {
    Serial.println(F("WARNING: no recent GPS bytes. Check M8N power, ground, GPS TX -> board PC7/USART6 RX, GPS RX <- board PC6/USART6 TX, and 9600 baud."));
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  pinMode(PC7, INPUT);
  const int rxIdleLevel = digitalRead(PC7);

  Serial6.begin(GPS_BAUD);
  lastByteMillis = millis();

  Serial.println();
  Serial.println(F("M8N GPS Terminal Reader started"));
  Serial.println(F("USB terminal: 115200 baud"));
  Serial.println(F("GPS UART: USART6 / Serial6 at 9600 baud"));
  Serial.print(F("PC7/USART6 RX idle level before UART start: "));
  Serial.println(rxIdleLevel == HIGH ? F("HIGH (normal UART idle if GPS TX is connected/powered)") : F("LOW (possible short, swapped wire, or unpowered GPS)"));
  Serial.println(F("Waiting for raw NMEA data; also sending a ping every 5 seconds..."));
  sendGpsPing();
}

void loop() {
  while (Serial6.available() > 0) {
    const int rawByte = Serial6.read();
    if (rawByte < 0) {
      continue;
    }
    const uint8_t value = static_cast<uint8_t>(rawByte);
    const char c = static_cast<char>(value);
    ++gpsByteCount;
    lastByteMillis = millis();
    updateUbxParser(value);

    if (c == '$') {
      printHexSample("before NMEA sentence");
      nmeaBufferIndex = 0;
      nmeaDiscarding = false;
      nmeaReceiving = true;
    } else if (!nmeaReceiving && c != '\r' && c != '\n') {
      ++binaryByteCount;
      recordHexSample(value);
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (nmeaReceiving && !nmeaDiscarding) {
        nmeaBuffer[nmeaBufferIndex] = '\0';
        if (nmeaBufferIndex > 0) {
          parseAndPrintSentence(nmeaBuffer);
        }
      }
      nmeaBufferIndex = 0;
      nmeaBuffer[0] = '\0';
      nmeaDiscarding = false;
      nmeaReceiving = false;
      continue;
    }

    if (nmeaReceiving && !nmeaDiscarding) {
      if (nmeaBufferIndex < NMEA_BUFFER_SIZE - 1) {
        nmeaBuffer[nmeaBufferIndex++] = c;
      } else {
        ++overLengthSentenceCount;
        nmeaBufferIndex = 0;
        nmeaBuffer[0] = '\0';
        nmeaDiscarding = true;
        nmeaReceiving = false;
      }
    }
  }

  const uint32_t now = millis();
  if (now - lastPingMillis >= PING_PERIOD_MS) {
    printHexSample("before ping");
    sendGpsPing();
  }

  printStatusHeartbeat();
}
