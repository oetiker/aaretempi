#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LoraMessage.h>
//#define AA_DEBUG
//#define AA_FAKE

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 300 * 1000;

// Data wire is plugged into GPIO5 on the CubeCell
#define ONE_WIRE_BUS GPIO5

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/* OTAA para*/
#ifdef AA_FAKE
uint8_t devEui[] = { 0x00, 0xBF, 0xAA, 0xD2, 0xEF, 0x26, 0x30, 0xDC };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0x00, 0x03, 0xE0, 0x3B };
uint8_t appKey[] = { 0x69, 0x3A, 0x56, 0x16, 0x60, 0x00, 0xC2, 0xEB, 0x76, 0xC5, 0xBA, 0xEA, 0xBC, 0x46, 0xA9, 0xA9 };
#else /* HOT */
uint8_t devEui[] = { 0x00, 0x6A, 0x00, 0xDA, 0xA0, 0x4F, 0x06, 0x8B };
uint8_t appEui[] = { 0x70, 0x00, 0xD5, 0x00, 0xD0, 0x01, 0x3D, 0xFA };
uint8_t appKey[] = { 0x00, 0x17, 0xCF, 0xF2, 0x00, 0x67, 0x13, 0x24, 0xFF, 0xB8, 0x14, 0x76, 0x41, 0xC0, 0xC3, 0x09 };
#endif

/* ABP para*/
uint8_t nwkSKey[] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
uint8_t appSKey[] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
uint32_t devAddr =  ( uint32_t )0x00;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;



/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 1;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;




void getTemp(int count, LoraMessage &message) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(500);
  sensors.begin();
  delay(500);
  sensors.requestTemperatures();
  int i;
  for (i = 0; i < count; i++) {
    float temp = sensors.getTempCByIndex(i);
#ifdef AA_DEBUG
    Serial.print(F("Temp "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.println(temp);
#endif
    message.addTemperature(temp);
  }
  digitalWrite(Vext, HIGH);
}



void setup() {
  boardInitMcu();
#ifdef AA_DEBUG
  Serial.begin(115200);
#endif
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();
}

void loop()
{
#ifdef AA_DEBUG
  //Serial.print(F("."));
#endif
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#ifdef AA_DEBUG
        printDevParam();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
#ifdef AA_DEBUG
        Serial.println(F("Sending"));
#endif
        LoraMessage message;
        getTemp(2, message);
        float voltage = getBatteryVoltage() / 1000.0;
#ifdef AA_DEBUG
        Serial.print(voltage);
        Serial.println(F("V"));
#endif
        message.addTemperature(voltage);
        appDataSize = message.getLength();
        memcpy(appData, message.getBytes(), appDataSize);
#ifdef AA_DEBUG
        Serial.print(F("send start ..."));
#endif
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
#ifdef AA_DEBUG
        Serial.println(F("Schedule next Cycle"));
#endif
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
#ifdef AA_DEBUG
        //       Serial.println(F("Sleeping"));
#endif
        LoRaWAN.sleep();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
