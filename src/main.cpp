/*
  main.cpp
  Main code where the control takes place
  @author  Leo Korbee (c), Leo.Korbee@xs4all.nl
  @website iot-lab.org
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed beforme me on this code or libraries.
  Please share you thoughts and comments or you projects with me through e-mail
  or comment on my website

  Please remove all comments on Serial.print() commands to get debug
  information on hardware serial port (9600 baud). Rember that this wil take
  about 2uA in Sleep Mode

  Hardware information at the end of this file.

  @version 2020-07-12
  Added LMIC_setClockError due to inaccurate timing on LMIC

  @version 2020-10-08
  Added code so you can use ABP instead of OTAA. Please enable DISABLE_JOIN
  in the config.h in the lmic library and change ABP config in secconfig.h
*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "STM32LowPower.h"
#include "STM32IntRef.h"
#include "BME280.h"

void do_send(osjob_t* j);

// for debugging redirect to hardware Serial2
// Tx on PA2
//#define Serial Serial2
//HardwareSerial Serial2(USART2);   // or HardWareSerial Serial2 (PA3, PA2);

/* A BME280 object using SPI, chip select pin PA1 */
BME280 bme(SPI,PA1);

// include security credentials OTAA, check secconfig_example.h for more information
#include "secconfig.h"

#ifdef DISABLE_JOIN
// These callbacks are only used in over-the-air activation, so they are
// left empty here. We cannot leave them out completely otherwise the linker will complain.
// DISABLE_JOIN is set in config.h for using ABP
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#else
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif

static osjob_t sendjob;

// Sleep this many microseconds. Notice that the sending and waiting for downlink
// will extend the time between send packets. You have to extract this time
#define SLEEP_INTERVAL 300000

// Pin mapping for the MiniPill LoRa with the RFM95 LoRa chip
const lmic_pinmap lmic_pins =
{
  .nss = PA4,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = PA9,
  .dio = {PA10, PB4, PB5},
};

// event called by os on events
void onEvent (ev_t ev)
{
  //   Serial.print(os_getTime());
  //   Serial.print(": ");
     switch(ev) {
         case EV_SCAN_TIMEOUT:
    //         Serial.println(F("EV_SCAN_TIMEOUT"));
             break;
         case EV_BEACON_FOUND:
    //         Serial.println(F("EV_BEACON_FOUND"));
             break;
         case EV_BEACON_MISSED:
    //         Serial.println(F("EV_BEACON_MISSED"));
             break;
         case EV_BEACON_TRACKED:
    //         Serial.println(F("EV_BEACON_TRACKED"));
             break;
         case EV_JOINING:
    //         Serial.println(F("EV_JOINING"));
             break;
         case EV_JOINED:
    //         Serial.println(F("EV_JOINED"));

             // Disable link check validation (automatically enabled
             // during join, but not supported by TTN at this time).
             LMIC_setLinkCheckMode(0);
             break;
         case EV_RFU1:
    //         Serial.println(F("EV_RFU1"));
             break;
         case EV_JOIN_FAILED:
    //         Serial.println(F("EV_JOIN_FAILED"));
             break;
         case EV_REJOIN_FAILED:
    //         Serial.println(F("EV_REJOIN_FAILED"));
             break;
             break;
         case EV_TXCOMPLETE:
    //         Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
             if (LMIC.txrxFlags & TXRX_ACK)
    //           Serial.println(F("Received ack"));
             if (LMIC.dataLen)
             {
    //           Serial.print(F("Received "));
    //           Serial.print(LMIC.dataLen);
    //           Serial.println(F(" bytes of payload"));
               // print data
               for (int i = 0; i < LMIC.dataLen; i++)
               {
    //             Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
               }
    //           Serial.println();
             }
    //         Serial.println("going to sleep");
             // reset low power parameters, seems to work -> 0.6 uA without BME280
             LowPower.begin();
             delay(100);
             // set PA6 to analog to reduce power due to currect flow on DIO on BME280
             pinMode(PA6, INPUT_ANALOG);
             // take SLEEP_INTERVAL time to sleep
             LowPower.deepSleep(SLEEP_INTERVAL);
    //         Serial.println("queing next job");
            // next transmission
             do_send(&sendjob);
             break;
         case EV_LOST_TSYNC:
    //         Serial.println(F("EV_LOST_TSYNC"));
             break;
         case EV_RESET:
      //       Serial.println(F("EV_RESET"));
             break;
         case EV_RXCOMPLETE:
             // data received in ping slot
    //         Serial.println(F("EV_RXCOMPLETE"));
             break;
         case EV_LINK_DEAD:
    //         Serial.println(F("EV_LINK_DEAD"));
             break;
         case EV_LINK_ALIVE:
    //         Serial.println(F("EV_LINK_ALIVE"));
             break;
          default:
    //         Serial.println(F("Unknown event"));
             break;
     }
}

//
void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
//         Serial.println(F("OP_TXRXPEND, not sending"));
  } else
  {
    // Prepare upstream data transmission at the next possible time.
    uint8_t dataLength = 8;
    uint8_t data[dataLength];

    // read vcc and add to bytebuffer
    int32_t vcc = IntRef.readVref();
    data[0] = (vcc >> 8) & 0xff;
    data[1] = (vcc & 0xff);

    // reading data from BME sensor
    bme.readSensor();
    float tempFloat = bme.getTemperature_C();
    float humFloat = bme.getHumidity_RH();
    float pressFloat = bme.getPressure_Pa();

    // from float to uint16_t
    uint16_t tempInt = 100 * tempFloat;
    uint16_t humInt = 100 * humFloat;
    // pressure is already given in 100 x mBar = hPa
    uint16_t pressInt = pressFloat/10;

    // move BME data into bytebuffer
    data[2] = (tempInt >> 8) & 0xff;
    data[3] = tempInt & 0xff;

    data[4] = (humInt >> 8) & 0xff;
    data[5] = humInt & 0xff;

    data[6] = (pressInt >> 8) & 0xff;
    data[7] = pressInt & 0xff;

    // set forced mode to be shure it will use minimal power and send it to sleep
    bme.setForcedMode();
    bme.goToSleep();
    // set data in queue
    LMIC_setTxData2(1, data, sizeof(data), 0);
  //       Serial.println(F("Packet queued"));
  }
}

void setup()
{
  //   Serial.begin(9600);
  // delay at startup for debugging reasons
  delay(5000);
  //   Serial.println(F("Starting"));
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // to incrise the size of the RX window.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  // Set static session parameters when using ABP.
  // Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef DISABLE_JOIN
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  // These settings are needed for correct communication. By removing them you
  // get empty downlink messages
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  #endif

  // begin communication with BME280 and set to default sampling, iirc, and standby settings
  if (bme.begin() < 0)
  {
    //   Serial.println("Error communicating with BME280 sensor, please check wiring");
    while(1){}
  }
  // Configure low power at startup
  LowPower.begin();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop()
{
  // run the os_loop to check if a job is available
  os_runloop_once();
}

/* MiniPill LoRa v1.1 mapping - LoRa module RFM95W and BME280 sensor
  PA1  //            NSS           - BME280
  PA4  // SPI1_NSS   NSS  - RFM95W
  PA5  // SPI1_SCK   SCK  - RFM95W - BME280
  PA6  // SPI1_MISO  MISO - RFM95W - BME280
  PA7  // SPI1_MOSI  MOSI - RFM95W - BME280

  PA10 // USART1_RX  DIO0 - RFM95W
  PB4  //            DIO1 - RFM95W
  PB5  //            DIO2 - RFM95W

  PA9  // USART1_TX  RST  - RFM95W

  VCC - 3V3
  GND - GND
*/
