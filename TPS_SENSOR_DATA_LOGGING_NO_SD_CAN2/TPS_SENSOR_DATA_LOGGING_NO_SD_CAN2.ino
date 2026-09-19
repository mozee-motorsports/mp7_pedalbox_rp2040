/*TPS Sensor Data Logging + Software CAN TX Code (full diagnostic output)
Below is the wiring diagram

ARDUINO UNO R3
─────────────────────────────────────────────
5V   ─────────────┬── TPS1 +5V
                  ├── TPS2 +5V
                  └── MCP2562 VDD (pin 3) + VIO (pin 5)

GND  ─────────────┬── TPS1 GND
                  ├── TPS2 GND

                  ├── MCP2562 Vss (pin 2)
                  └── MCP2562 STBY (pin 8)

A0   ───────────────── TPS1 SIGNAL
A1   ───────────────── TPS2 SIGNAL

D8   ───────────────── MCP2562 TxD (pin 1)
D2   ◄──────────────── MCP2562 RxD (pin 4)

MCP2562 CANH (pin 7) / CANL (pin 6) ── physical CAN bus
(120-ohm termination resistor across CANH/CANL)

SD card wiring (currently disabled in code, kept here for reference):
D10  ───────────────── SD CS
D11  ───────────────── SD MOSI
D12  ───────────────── SD MISO
D13  ───────────────── SD SCK

Every sample cycle, this prints:
  - raw ADC counts, voltages, and percentages for both TPS sensors
  - the TPS agreement check
  - the raw CAN payload bytes being sent
  - whether the frame was ACKed
  - the frame decoded back from what was captured on RX (ID/DLC/Data/CRC)
so you can verify the whole pipeline end-to-end from one Serial Monitor view.
*/

#include <SPI.h>
// #include "SdFat.h"   // SD card disabled for now

// -----------------------------------------------------------------------
// TPS sensor config
// -----------------------------------------------------------------------
const int TPS1_PIN = A0;
const int TPS2_PIN = A1;

const int SD_CS_PIN = 10;

const float ADC_REFERENCE = 5.0;     //Max voltage of the sensor

// Individual calibration values for each sensor - change these as needed
const float TPS1_MIN_VOLTAGE = 0.288;
const float TPS1_MAX_VOLTAGE = 1.334;

const float TPS2_MIN_VOLTAGE = 3.128;
const float TPS2_MAX_VOLTAGE = 1.979;

const float MAX_DIFFERENCE = 5.0;

// Sampling rate in Hz
const float SAMPLE_RATE_HZ = 50.0;

// SdFat sd;              // SD card disabled for now
// File32 logFile;        // SD card disabled for now

unsigned long nextSampleTime = 0;
unsigned long lastFlushTime = 0;

// 50 Hz hardware timer
// Timer1 generates an interrupt every 20 ms
// ============================================================

volatile bool sampleReady = false;

ISR(TIMER1_COMPA_vect)
{
  sampleReady = true;
}


float voltageToPercent(float voltage, float minVoltage, float maxVoltage)
{
  float percent =
    (voltage - minVoltage) /
    (maxVoltage - minVoltage) * 100.0;

  return constrain(percent, 0.0, 100.0);
}

void setupTimer1()
{
  noInterrupts();

  // Reset Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Arduino Uno = 16 MHz
  // Timer1 prescaler = 64
  //
  // Timer frequency:
  // 16,000,000 / 64 = 250,000 Hz
  //
  // Calculate the number of timer counts needed
  // for the desired sampling rate.

  float timerFrequency = 16000000.0 / 64.0;

  OCR1A = (uint16_t)((timerFrequency / SAMPLE_RATE_HZ) - 1.0);

  // CTC mode
  TCCR1B |= (1 << WGM12);

  // Prescaler = 64
  TCCR1B |= (1 << CS11) | (1 << CS10);

  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}

// -----------------------------------------------------------------------
// Software (bit-banged) CAN TX + loopback-style RX capture/decode for
// diagnostics. No error frames, no arbitration/retransmit, standard
// 11-bit IDs only. BIT_RATE must match whatever is listening on the bus.
// -----------------------------------------------------------------------
#define CAN_TX_PIN 8
#define CAN_RX_PIN 2

#define CAN_BIT_RATE_BPS 20000UL
#define CAN_BIT_PERIOD_US (1000000UL / CAN_BIT_RATE_BPS)

#define DOMINANT  0
#define RECESSIVE 1

#define MAX_STUFFED_BITS 140

uint8_t stuffedBits[MAX_STUFFED_BITS];
uint8_t receivedBits[MAX_STUFFED_BITS];
int stuffedBitCount = 0;

void canSetBus(uint8_t bitValue)
{
  digitalWrite(CAN_TX_PIN, bitValue == DOMINANT ? LOW : HIGH);
}

uint8_t canReadBus()
{
  return digitalRead(CAN_RX_PIN) == LOW ? DOMINANT : RECESSIVE;
}

void canPinsInit()
{
  pinMode(CAN_TX_PIN, OUTPUT);
  pinMode(CAN_RX_PIN, INPUT);
  canSetBus(RECESSIVE); // idle bus = recessive
}

// ---- Encoder-side bit stuffing ----
uint8_t encLastBit = 0xFF;
int encRunLength = 0;

void appendStuffedBit(uint8_t bit)
{
  stuffedBits[stuffedBitCount++] = bit;

  if (bit == encLastBit)
  {
    encRunLength++;
  }
  else
  {
    encLastBit = bit;
    encRunLength = 1;
  }

  if (encRunLength == 5)
  {
    uint8_t stuffBit = !bit;
    stuffedBits[stuffedBitCount++] = stuffBit;
    encLastBit = stuffBit;
    encRunLength = 1;
  }
}

uint16_t crcRegister;

void crcInit()
{
  crcRegister = 0;
}

void crcAddBit(uint8_t bit)
{
  uint8_t crcNext = bit ^ ((crcRegister >> 14) & 0x1);
  crcRegister = (crcRegister << 1) & 0x7FFF;
  if (crcNext)
  {
    crcRegister ^= 0x4599;
  }
}

int buildFrame(uint16_t id11, uint8_t *data, uint8_t dlc)
{
  if (dlc > 8) dlc = 8;

  stuffedBitCount = 0;
  encLastBit = 0xFF;
  encRunLength = 0;
  crcInit();

  #define ADD_RAW_BIT(b) do { crcAddBit(b); appendStuffedBit(b); } while (0)

  ADD_RAW_BIT(DOMINANT); // SOF

  for (int i = 10; i >= 0; i--) ADD_RAW_BIT((id11 >> i) & 0x1); // ID

  ADD_RAW_BIT(DOMINANT); // RTR
  ADD_RAW_BIT(DOMINANT); // IDE
  ADD_RAW_BIT(DOMINANT); // r0

  for (int i = 3; i >= 0; i--) ADD_RAW_BIT((dlc >> i) & 0x1); // DLC

  for (int b = 0; b < dlc; b++)
    for (int i = 7; i >= 0; i--)
      ADD_RAW_BIT((data[b] >> i) & 0x1); // Data

  #undef ADD_RAW_BIT

  for (int i = 14; i >= 0; i--) appendStuffedBit((crcRegister >> i) & 0x1); // CRC (stuffed)

  stuffedBits[stuffedBitCount++] = RECESSIVE; // CRC delimiter
  stuffedBits[stuffedBitCount++] = RECESSIVE; // ACK slot
  stuffedBits[stuffedBitCount++] = RECESSIVE; // ACK delimiter
  for (int i = 0; i < 7; i++) stuffedBits[stuffedBitCount++] = RECESSIVE; // EOF
  for (int i = 0; i < 3; i++) stuffedBits[stuffedBitCount++] = RECESSIVE; // IFS

  return stuffedBitCount;
}

bool transmitAndCapture(int ackBitIndex)
{
  unsigned long nextBitTime = micros();
  bool ackReceived = false;

  for (int i = 0; i < stuffedBitCount; i++)
  {
    while ((long)(micros() - nextBitTime) < 0) { /* busy wait */ }

    canSetBus(stuffedBits[i]);

    delayMicroseconds(2); // let RX settle before sampling
    receivedBits[i] = canReadBus();

    if (i == ackBitIndex)
    {
      delayMicroseconds(CAN_BIT_PERIOD_US / 2);
      if (canReadBus() == DOMINANT) ackReceived = true;
    }

    nextBitTime += CAN_BIT_PERIOD_US;
  }

  canSetBus(RECESSIVE);
  return ackReceived;
}

// ---- Decoder-side destuffing/parsing ----
uint8_t decLastBit;
int decRunLength;
int decPhysIndex;

uint8_t nextRawBit(uint8_t *bits)
{
  if (decRunLength == 5)
  {
    uint8_t stuffBit = bits[decPhysIndex++]; // discard the stuff bit
    decLastBit = stuffBit;
    decRunLength = 1;
  }

  uint8_t rawBit = bits[decPhysIndex++];

  if (rawBit == decLastBit)
  {
    decRunLength++;
  }
  else
  {
    decLastBit = rawBit;
    decRunLength = 1;
  }

  return rawBit;
}

void decodeAndPrint(uint8_t *bits)
{
  decLastBit = 0xFF;
  decRunLength = 0;
  decPhysIndex = 0;

  uint8_t rawFrame[100];
  int rawCount = 0;

  for (int i = 0; i < 19; i++) rawFrame[rawCount++] = nextRawBit(bits); // SOF+ID+RTR+IDE+r0+DLC

  uint16_t id = 0;
  for (int k = 0; k < 11; k++) id = (id << 1) | rawFrame[1 + k];

  uint8_t dlc = 0;
  for (int k = 0; k < 4; k++) dlc = (dlc << 1) | rawFrame[15 + k];
  if (dlc > 8) dlc = 8;

  int dataBits = dlc * 8;

  for (int i = 0; i < dataBits + 15; i++) rawFrame[rawCount++] = nextRawBit(bits); // Data+CRC

  uint8_t data[8];
  for (int b = 0; b < dlc; b++)
  {
    uint8_t byteVal = 0;
    for (int k = 0; k < 8; k++) byteVal = (byteVal << 1) | rawFrame[19 + b * 8 + k];
    data[b] = byteVal;
  }

  uint16_t recvCrc = 0;
  for (int k = 0; k < 15; k++) recvCrc = (recvCrc << 1) | rawFrame[19 + dataBits + k];

  crcInit();
  for (int i = 0; i < 19 + dataBits; i++) crcAddBit(rawFrame[i]);
  bool crcOk = (crcRegister == recvCrc);

  Serial.print("  [Decoded RX] ID: 0x");
  Serial.print(id, HEX);
  Serial.print("  DLC: ");
  Serial.print(dlc);
  Serial.print("  Data:");
  for (int b = 0; b < dlc; b++)
  {
    Serial.print(" ");
    if (data[b] < 0x10) Serial.print("0");
    Serial.print(data[b], HEX);
  }
  Serial.print("  CRC: ");
  Serial.println(crcOk ? "OK" : "MISMATCH");
}

bool canSendAndVerify(uint16_t id11, uint8_t *data, uint8_t dlc)
{
  int totalBits = buildFrame(id11, data, dlc);
  int ackBitIndex = totalBits - (3 + 7 + 1 + 1);

  bool acked = transmitAndCapture(ackBitIndex);

  decodeAndPrint(receivedBits);

  return acked;
}

// CAN ID used for the TPS data frame - change if it conflicts with something
// else on your test bus
#define TPS_CAN_ID 0x123

void setup()
{
  Serial.begin(115200);

  pinMode(TPS1_PIN, INPUT);
  pinMode(TPS2_PIN, INPUT);

  canPinsInit();

  /*
  // SD card disabled for now
  // Initialize SD card
  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(8)))
  {
    Serial.println("SD card initialization FAILED!");
    while (1);
  }

  Serial.println("SD card initialized.");

  // Open/create log file
  logFile = sd.open("TPS_LOG.CSV", O_WRONLY | O_CREAT | O_APPEND);

  if (!logFile)
  {
    Serial.println("Failed to open TPS_LOG.CSV");
    while (1);
  }

  // Write header only if the file is empty
  if (logFile.size() == 0)
  {
    logFile.println(
      "Time_ms,TPS1_Raw,TPS1_Voltage,TPS1_Percent,"
      "TPS2_Raw,TPS2_Voltage,TPS2_Percent,"
      "Difference_Percent,Agree"
    );

    logFile.flush();
  }
  */

  Serial.println("=== TPS + CAN diagnostic output ===");
  Serial.println();

  setupTimer1();
  lastFlushTime = millis();
}

void loop()
{
  // ==========================================================
  // 50 Hz sampling
  // Timer1 sets sampleReady every 20 ms
  // ==========================================================

  if (sampleReady)
  {
    // Clear the flag
    noInterrupts();
    sampleReady = false;
    interrupts();

    unsigned long timeMs = millis();

    // Read TPS sensors
    int tps1Raw = analogRead(TPS1_PIN);
    int tps2Raw = analogRead(TPS2_PIN);

    // Convert ADC readings to voltage
    float tps1Voltage =
      tps1Raw * (ADC_REFERENCE / 1023.0);

    float tps2Voltage =
      tps2Raw * (ADC_REFERENCE / 1023.0);

    // Convert voltage to throttle percentage (each sensor uses its own calibration)
    float tps1Percent =
      voltageToPercent(tps1Voltage, TPS1_MIN_VOLTAGE, TPS1_MAX_VOLTAGE);

    float tps2Percent =
      voltageToPercent(tps2Voltage, TPS2_MIN_VOLTAGE, TPS2_MAX_VOLTAGE);

    // Compare TPS sensors
    float difference =
      abs(tps1Percent - tps2Percent);

    bool agree = difference <= MAX_DIFFERENCE;

    // -------------------------
    // Build CAN payload: pack TPS1_Percent and TPS2_Percent as two
    // floats (4 bytes each = 8 bytes total, fits in one CAN frame)
    // -------------------------
    uint8_t canData[8];
    memcpy(&canData[0], &tps1Percent, 4);
    memcpy(&canData[4], &tps2Percent, 4);

    bool canAcked = canSendAndVerify(TPS_CAN_ID, canData, 8);

    // -------------------------
    // FULL diagnostic Serial output
    // -------------------------
    Serial.print("[t=");
    Serial.print(timeMs);
    Serial.println(" ms]");

    Serial.print("  TPS1 -> Raw: ");
    Serial.print(tps1Raw);
    Serial.print("  Voltage: ");
    Serial.print(tps1Voltage, 3);
    Serial.print(" V  Percent: ");
    Serial.print(tps1Percent, 2);
    Serial.println(" %");

    Serial.print("  TPS2 -> Raw: ");
    Serial.print(tps2Raw);
    Serial.print("  Voltage: ");
    Serial.print(tps2Voltage, 3);
    Serial.print(" V  Percent: ");
    Serial.print(tps2Percent, 2);
    Serial.println(" %");

    Serial.print("  Difference: ");
    Serial.print(difference, 2);
    Serial.print(" %  Agree: ");
    Serial.println(agree ? "YES" : "NO");

    Serial.print("  [CAN TX] ID: 0x");
    Serial.print(TPS_CAN_ID, HEX);
    Serial.print("  Payload:");
    for (int b = 0; b < 8; b++)
    {
      Serial.print(" ");
      if (canData[b] < 0x10) Serial.print("0");
      Serial.print(canData[b], HEX);
    }
    Serial.print("  ACK: ");
    Serial.println(canAcked ? "YES" : "NO (expected with no other node)");

    // decodeAndPrint() already printed the "[Decoded RX] ..." line above

    Serial.println();

    // -------------------------
    // SD card output (disabled for now)
    // -------------------------
    /*
    logFile.print(timeMs);
    logFile.print(",");

    logFile.print(tps1Raw);
    logFile.print(",");
    logFile.print(tps1Voltage, 3);
    logFile.print(",");
    logFile.print(tps1Percent, 2);
    logFile.print(",");

    logFile.print(tps2Raw);
    logFile.print(",");
    logFile.print(tps2Voltage, 3);
    logFile.print(",");
    logFile.print(tps2Percent, 2);
    logFile.print(",");

    logFile.print(difference, 2);
    logFile.print(",");

    logFile.println(agree ? "GOOD" : "BAD");
    */
  }

  /*
  // Flush SD roughly once per second (disabled for now)
  if (millis() - lastFlushTime >= 1000)
  {
    logFile.flush();
    lastFlushTime = millis();
  }
  */
}
