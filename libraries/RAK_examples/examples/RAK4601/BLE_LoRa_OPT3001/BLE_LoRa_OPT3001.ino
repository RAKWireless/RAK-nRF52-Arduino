/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules
 Pick one up today in the adafruit shop!
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!
 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>
#include <ClosedCube_OPT3001.h> // Click here to get the library: http://librarymanager/All#OPT3001
ClosedCube_OPT3001 opt3001;
#define OPT3001_ADDRESS 0x44
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={0x15,0x09,0x00,0x00,0x00,0x00,0x00,0x00};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={0x15,0x09,0x00,0x00,0x00,0x00,0x00,0x00};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xb3,0x8b,0x0d,0xef,0x40,0x38,0x31,0x63,0x1f,0x81,0x34,0x8e,0xeb,0xe6,0x9d,0x66 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t lora_data[64];
uint32_t lora_count = 0;
static osjob_t sendjob;

typedef enum {
  LORA_UNCONFIRMED,
  LORA_CONFIRMED  
};
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {27, 28, 29},
};

void onEvent (ev_t ev) {
    //Serial.print(os_getTime());
    //Serial.print(": ");
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
            Serial.println(F("LoRa Start Joining..."));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack\r\n"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            //Serial.println(F("EV_TXSTART"));
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        opt3001_get();
        LMIC_setTxData2(1, lora_data, lora_count, LORA_UNCONFIRMED);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  /* opt3001 init */
  opt3001.begin(OPT3001_ADDRESS);
  Serial.print("OPT3001 Manufacturer ID");
  Serial.println(opt3001.readManufacturerID());
  Serial.print("OPT3001 Device ID");
  Serial.println(opt3001.readDeviceID());

  configureSensor();
  printResult("High-Limit", opt3001.readHighLimit());
  printResult("Low-Limit", opt3001.readLowLimit());


  /* LoRa init */
  Serial.println("LoRa init");
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  #if defined(CFG_us915)
    LMIC_selectSubBand(1);
  #endif

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);


  /* BLE init */
  Serial.println("BLE init");
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // Start Central Scan
  Bluefruit.Scanner.setInterval(3200, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.start(6000);  // in units of 10 ms,  0 = Don't stop scanning

  Serial.println("BLE scan 60 seconds...");
}

void scan_callback(ble_gap_evt_adv_report_t* report)
{
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));

  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.print(" ");

  Serial.print(report->rssi);
  Serial.print("  ");

  Serial.printBuffer(report->data.p_data, report->data.len, '-');
  Serial.println();

  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForUuid(report, BLEUART_UUID_SERVICE) )
  {
    Serial.println("                       BLE UART service detected");
  }

  Serial.println();

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

void loop() 
{
  os_runloop_once();
}

void configureSensor() {
  OPT3001_Config newConfig;
  
  newConfig.RangeNumber = B1100;  
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;

  OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);
  if (errorConfig != NO_ERROR)
    printError("OPT3001 configuration", errorConfig);
  else {
    OPT3001_Config sensorConfig = opt3001.readConfig();
    Serial.println("OPT3001 Current Config:");
    Serial.println("------------------------------");
    
    Serial.print("Conversion ready (R):");
    Serial.println(sensorConfig.ConversionReady,HEX);

    Serial.print("Conversion time (R/W):");
    Serial.println(sensorConfig.ConvertionTime, HEX);

    Serial.print("Fault count field (R/W):");
    Serial.println(sensorConfig.FaultCount, HEX);

    Serial.print("Flag high field (R-only):");
    Serial.println(sensorConfig.FlagHigh, HEX);

    Serial.print("Flag low field (R-only):");
    Serial.println(sensorConfig.FlagLow, HEX);

    Serial.print("Latch field (R/W):");
    Serial.println(sensorConfig.Latch, HEX);

    Serial.print("Mask exponent field (R/W):");
    Serial.println(sensorConfig.MaskExponent, HEX);

    Serial.print("Mode of conversion operation (R/W):");
    Serial.println(sensorConfig.ModeOfConversionOperation, HEX);

    Serial.print("Polarity field (R/W):");
    Serial.println(sensorConfig.Polarity, HEX);

    Serial.print("Overflow flag (R-only):");
    Serial.println(sensorConfig.OverflowFlag, HEX);

    Serial.print("Range number (R/W):");
    Serial.println(sensorConfig.RangeNumber, HEX);

    Serial.println("------------------------------");
  }
  
}

void opt3001_get()
{
  uint8_t cnt = 0;
  OPT3001 result = opt3001.readResult();
  if (result.error == NO_ERROR) {
    Serial.print("OPT3001");
    Serial.print(": ");
    Serial.print(result.lux);
    Serial.println(" lux");

    uint16_t luminosity = result.lux;
    // Luminosity
    lora_data[cnt++] = 0x01;
    lora_data[cnt++] = LPP_LUMINOSITY;
    lora_data[cnt++] = luminosity>>8;
    lora_data[cnt++] = luminosity;
  }
  else {
    printError("OPT3001",result.error);
  }

    lora_count = cnt;   // lora data len
}

void printResult(String text, OPT3001 result) {
  if (result.error == NO_ERROR) {
    Serial.print(text);
    Serial.print(": ");
    Serial.print(result.lux);
    Serial.println(" lux");
  }
  else {
    printError(text,result.error);
  }
}
void printError(String text, OPT3001_ErrorCode error) {
  Serial.print(text);
  Serial.print(": [ERROR] Code #");
  Serial.println(error);
}