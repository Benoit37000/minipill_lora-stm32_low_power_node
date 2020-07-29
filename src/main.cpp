
 // Pin mapping
 /* MiniPill LoRa v1.1 mapping - LoRa module RFM95W
 PA4,  // SPI1_NSS   NSS - RFM95W
 PA5,  // SPI1_SCK.  SCK - RFM95W
 PA6,  // SPI1_MISO. MISO - RFM95W
 PA7,  // SPI1_MOSI. MOSI - RFM95W

 PA10, // USART1_RX. DIO0 - RFM95W
 PB4,  //            DIO1 - RFM95W
 PB5,  //            DIO2 - RFM95W

 PA9,  // USART1_TX. RST - RFM95W

 VCC - 3V3
 GND - GND
 */

// 2020-07-12  LMIC_setClockError
// ttn12



/*******************************************************************************
* Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
*
* Permission is hereby granted, free of charge, to anyone
* obtaining a copy of this document and accompanying files,
* to do whatever they want with them without any restriction,
* including, but not limited to, copying, modification and redistribution.
* NO WARRANTY OF ANY KIND IS PROVIDED.
*
* This example sends a valid LoRaWAN packet with payload "Hello,
* world!", using frequency and encryption settings matching those of
* the The Things Network.
*
* This uses OTAA (Over-the-air activation), where where a DevEUI and
* application key is configured, which are used in an over-the-air
* activation procedure where a DevAddr and session keys are
* assigned/generated for use with all further communication.
*
* Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
* g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
* violated by this sketch when left running for longer)!

* To use this sketch, first register your application and device with
* the things network, to set or generate an AppEUI, DevEUI and AppKey.
* Multiple devices can use the same AppEUI, but each device has its own
* DevEUI and AppKey.
*
* Do not forget to define the radio type correctly in config.h.
*
*******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "STM32LowPower.h"

void do_send(osjob_t* j);

// for debugging redirect to hardware Serial2
// Tx on PA2
#define Serial Serial2
HardwareSerial Serial2(USART2);   // or HardWareSerial Serial2 (PA3, PA2);
int i;

// include security credentials OTAA, check secconfig_example.h for more information
#include "secconfig.h"
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

 // Schedule TX every this many seconds (might become longer due to duty
 // cycle limitations).
 const unsigned TX_INTERVAL = 60;

 // Pin mapping for the MiniPill LoRa
 const lmic_pinmap lmic_pins =
 {
     .nss = PA4,
     .rxtx = LMIC_UNUSED_PIN,
     .rst = PA9,
     .dio = {PA10, PB4, PB5},
 };

 void onEvent (ev_t ev) {
     Serial.print(os_getTime());
     Serial.print(": ");
     switch(ev) {
         case EV_SCAN_TIMEOUT:
             Serial.println(F("EV_SCAN_TIMEOUT"));
             break;
         case EV_BEACON_FOUND:
             Serial.println(F("EV_BEACON_FOUND"));
             break;
         case EV_BEACON_MISSED:
             Serial.println(F("EV_BEACON_MISSED"));
             break;
         case EV_BEACON_TRACKED:
             Serial.println(F("EV_BEACON_TRACKED"));
             break;
         case EV_JOINING:
             Serial.println(F("EV_JOINING"));
             break;
         case EV_JOINED:
             Serial.println(F("EV_JOINED"));

             // Disable link check validation (automatically enabled
             // during join, but not supported by TTN at this time).
             LMIC_setLinkCheckMode(0);
             break;
         case EV_RFU1:
             Serial.println(F("EV_RFU1"));
             break;
         case EV_JOIN_FAILED:
             Serial.println(F("EV_JOIN_FAILED"));
             break;
         case EV_REJOIN_FAILED:
             Serial.println(F("EV_REJOIN_FAILED"));
             break;
             break;
         case EV_TXCOMPLETE:
             Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
             if (LMIC.txrxFlags & TXRX_ACK)
               Serial.println(F("Received ack"));
             if (LMIC.dataLen)
             {
               Serial.print(F("Received "));
               Serial.print(LMIC.dataLen);
               Serial.println(F(" bytes of payload"));
               // print data
               for (int i = 0; i < LMIC.dataLen; i++)
               {
                 Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
               }
               Serial.println();
             }
             // Schedule next transmission
             //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);

             // Low power next transmission
             Serial.println("going to sleep");
             LowPower.deepSleep(60000);
             Serial.println("queing next job");
             LowPower.begin(); // reset low power parameters, seems to work -> 0.6 uA
             do_send(&sendjob);
             break;
         case EV_LOST_TSYNC:
             Serial.println(F("EV_LOST_TSYNC"));
             break;
         case EV_RESET:
             Serial.println(F("EV_RESET"));
             break;
         case EV_RXCOMPLETE:
             // data received in ping slot
             Serial.println(F("EV_RXCOMPLETE"));
             break;
         case EV_LINK_DEAD:
             Serial.println(F("EV_LINK_DEAD"));
             break;
         case EV_LINK_ALIVE:
             Serial.println(F("EV_LINK_ALIVE"));
             break;
          default:
             Serial.println(F("Unknown event"));
             break;
     }
 }

 void do_send(osjob_t* j){
     // Check if there is not a current TX/RX job running
     if (LMIC.opmode & OP_TXRXPEND) {
         Serial.println(F("OP_TXRXPEND, not sending"));
     } else {
         // Prepare upstream data transmission at the next possible time.
         LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
         Serial.println(F("Packet queued"));
     }
     // Next TX is scheduled after TX_COMPLETE event.
 }

 void setup() {
     Serial.begin(9600);

     delay(5000);
     Serial.println(F("Starting"));

     #ifdef VCC_ENABLE
     // For Pinoccio Scout boards
     pinMode(VCC_ENABLE, OUTPUT);
     digitalWrite(VCC_ENABLE, HIGH);
     delay(1000);
     #endif

     // LMIC init
     os_init();
     // Reset the MAC state. Session and pending data transfers will be discarded.
     LMIC_reset();

     // to incrise the size of the RX window.
     LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

     // Configure low power
     LowPower.begin();

     // Start job (sending automatically starts OTAA too)
     do_send(&sendjob);
 }

 void loop()
 {
    Serial.print("loop ");
    Serial.println(i);
    i++;
    os_runloop_once();
 }
