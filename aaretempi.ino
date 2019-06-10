/*******************************************************************************
   Copyright (c) 2018 by Tobi Oetiker
   https://github.com/oetiker/aaretempi

   This code has been tested on an Adafruit Feather 32U4. It reads
   temperatures of DS18B20 sensors and report the temperature
   to the things network.

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.
   
   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   This will require 0.5 mA in sleep mode ... so it will use pretty little energy.

 *******************************************************************************/


#define PROBE_INTERVAL 300
#define SEND_DIFF 0.2
#define CFG_eu868 1
#define CFG_sx1276_radio 1
// #define SHOW_DEBUG

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LowPower.h>

#define VBATPIN A9

#define ONE_WIRE_BUS 12


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// This should also be in little endian format (LSB), see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xD6, 0x30, 0x29, 0xD1, 0x22, 0x65, 0xF2, 0x00 };

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This EUI must be in little-endian format (LSB), so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0xFA, 0x3D, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}


// This key should be in big endian format (MSB) (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x21, 0x40, 0xE3, 0x4D, 0x33, 0x2C, 0x29, 0xBF, 0x35, 0x78, 0x74, 0xCF, 0x8D, 0x20, 0x7C, 0x07  };

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

static osjob_t sendjob;

// Pin mapping

const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {6, 7, LMIC_UNUSED_PIN },
};

int sendDone = 1;

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
    case EV_BEACON_FOUND:
    case EV_BEACON_MISSED:
    case EV_BEACON_TRACKED:
    case EV_JOINING:
      break;
    case EV_JOINED:
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
#ifdef SHOW_DEBUG
      Serial.print(" joined ... ");
#endif
      break;
    case EV_RFU1:
    case EV_JOIN_FAILED:
    case EV_REJOIN_FAILED:
      break;
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW);
#ifdef SHOW_DEBUG
      Serial.println("done.");
#endif
      sendDone = 1;
      break;
    case EV_LOST_TSYNC:
    case EV_RESET:
    case EV_RXCOMPLETE:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
    default:
      break;
  }
}

void floatToBuffer(uint8_t *buffer, float value, int pos) {
  int16_t valueInt = value * 100;
  buffer[pos] = valueInt >> 8;
  buffer[pos + 1] = valueInt;
}

void initLoRaWAN () {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // LMIC init
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
  // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
  LMIC_setDrTxpow(DR_SF7,14);
}

void deepSleep() {
#ifdef SHOW_DEBUG
    Serial.print("sleep ... ");
    delay(PROBE_INTERVAL*1000);
#else
    int sleep = 0;
    while(true){
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      if (sleep > PROBE_INTERVAL) {
        break;
      }
      sleep += 8;
    }
#endif
}

float correctTemperature(int dev, float RawValue) {
  float ReferenceLow = 5.32;
  float ReferenceHigh = 48.02;
  float RawHigh;
  float RawLow;
  if (dev == 0) {
    RawLow = 5;
    RawHigh = 47.88;
  }
  if (dev == 1) {
    RawLow = 5.88;
    RawHigh = 48.81;
  }

  float RawRange = RawHigh - RawLow;
  float ReferenceRange = ReferenceHigh - ReferenceLow;
  float CorrectedValue = (((RawValue - RawLow) * ReferenceRange) / RawRange) + ReferenceLow;
  return CorrectedValue;
}


void checkAndSend(){
    static float t0prev = 0;
    static float t1prev = 0;
    sensors.requestTemperatures();
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(100);
    // digitalWrite(LED_BUILTIN, LOW);      
    delay(500);
    float t0 = correctTemperature(0, sensors.getTempCByIndex(0));
    float t1 = correctTemperature(1, sensors.getTempCByIndex(1));
    if (fabs(t0-t0prev) > SEND_DIFF || fabs(t1-t1prev) > SEND_DIFF){
      t0prev = t0;
      t1prev = t1;
      digitalWrite(LED_BUILTIN, HIGH);
      sendDone = 0;
      do_send(&sendjob,t0,t1);
    }
#ifdef SHOW_DEBUG
    else {
      Serial.println("No Temp Change ...");
    }
#endif
}

void do_send(osjob_t* j,float t0,float t1) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
#ifdef SHOW_DEBUG
    Serial.println("OP_TXRXPEND, not sending");
#endif
  } else {
    uint8_t buffer[6]; // reserve 6 bytes in memory
    floatToBuffer(buffer, t0, 0);
    floatToBuffer(buffer, t1, 2);
    // Prepare upstream data transmission at the next possible time.
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    floatToBuffer(buffer, measuredvbat, 4);
#ifdef SHOW_DEBUG
    Serial.print("Sending Data ...");
#endif
    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
   }
}

void setup() {
  delay(2000);
#ifdef SHOW_DEBUG
  Serial.begin(9600);
  while (! Serial);
#else
  // Disable USB clock 
  USBCON |= _BV(FRZCLK);
  // Disable USB PLL
  PLLCSR &= ~_BV(PLLE); 
  // Disable USB
  USBCON &= ~_BV(USBE);
#endif
#ifdef SHOW_DEBUG
  Serial.print("Starting Sensors ... ");
#endif
  sensors.begin();
#ifdef SHOW_DEBUG
  Serial.println("done.");
#endif
#ifdef SHOW_DEBUG
  Serial.print("Starting LoRaWAN ... ");
#endif
  initLoRaWAN();
  // Reset the MAC state. Session and pending data transfers will be discarded.
}


int initialRound = 1;

void loop() {
  if (sendDone){
    if (!initialRound){
       deepSleep();
    }
    initialRound = 0;
    checkAndSend();
  }
  else {
    os_runloop_once();
  }
}
