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
#include "BlueDot_BME680.h" // Click here to get the library: http://librarymanager/All#BlueDot_BME680
BlueDot_BME680 bme680 = BlueDot_BME680();

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
        bme680_get();
        LMIC_setTxData2(1, lora_data, lora_count, LORA_UNCONFIRMED);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  /* bme680 init */
  bme680_init();

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


void bme680_init(){
  //*********************************************************************
  //*************BASIC SETUP - READ BEFORE GOING ON!*********************
    
  //Set the I2C address of your breakout board  
  
  //0x76:       Alternative I2C Address (SDO pin connected to GND)
  //0x77:       Default I2C Address (SDO pin unconnected)
    Wire.begin();     
    bme680.parameter.I2CAddress = 0x76;                  //Choose I2C Address


  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Now the device will be set to forced mode
  //This is the default setup
  //Please note that unlike the BME280, BME680 does not have a normal mode

  //0b01:     In forced mode a single measured is performed and the device returns automatically to sleep mode
  
    bme680.parameter.sensorMode = 0b01;                   //Default sensor mode


  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Great! Now set up the internal IIR Filter
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
  //In short, a high factor value means less noise, but measurements are also less responsive
  //You can play with these values and check the results!
  //In doubt just leave on default

  //0b000:      factor 0 (filter off)
  //0b001:      factor 1
  //0b010:      factor 3
  //0b011:      factor 7
  //0b100:      factor 15 (default value)
  //0b101:      factor 31
  //0b110:      factor 63
  //0b111:      factor 127 (maximum value)

    bme680.parameter.IIRfilter = 0b100;                                   //Setting IIR Filter coefficient to 15 (default)

  
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************

  //Next you'll define the oversampling factor for the humidity measurements
  //Again, higher values mean less noise, but slower responses
  //If you don't want to measure humidity, set the oversampling to zero

  //0b000:      factor 0 (Disable humidity measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

    bme680.parameter.humidOversampling = 0b101;                           //Setting Humidity Oversampling to factor 16 (default) 


  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Now define the oversampling factor for the temperature measurements
  //You know now, higher values lead to less noise but slower measurements
  
  //0b000:      factor 0 (Disable temperature measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

    bme680.parameter.tempOversampling = 0b101;                            //Setting Temperature Oversampling factor to 16 (default)
    

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //Finally, define the oversampling factor for the pressure measurements
  //For altitude measurements a higher factor provides more stable values
  //On doubt, just leave it on default
  
  //0b000:      factor 0 (Disable pressure measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

    bme680.parameter.pressOversampling = 0b101;                           //Setting Pressure Oversampling to factor 16 (default) 
  

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
  
  //For precise altitude measurements please put in the current pressure corrected for the sea level
  //On doubt, just leave the standard pressure as default (1013.25 hPa)
  
    bme680.parameter.pressureSeaLevel = 1013.25;                          //default value of 1013.25 hPa

  //Now write here the current average temperature outside (yes, the outside temperature!)
  //You can either use the value in Celsius or in Fahrenheit, but only one of them (comment out the other value)
  //In order to calculate the altitude, this temperature is converted by the library into Kelvin
  //For slightly less precise altitude measurements, just leave the standard temperature as default (15°C)
  //Remember, leave one of the values here commented, and change the other one!
  //If both values are left commented, the default temperature of 15°C will be used
  //But if both values are left uncommented, then the value in Celsius will be used    
  
    bme680.parameter.tempOutsideCelsius = 15;                            //default value of 15°C
  //bme680.parameter.tempOutsideFahrenheit = 59;                         //default value of 59°F
  
  
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE!************************
   
  //Here we need to set the target temperature of the gas sensor hot plate
  //According to the datasheet, the target temperature is typically set between 200°C and 400°C
  //The default value is 320°C
  
    bme680.parameter.target_temp = 320;


  //*********************************************************************
  //*************ADVANCED SETUP IS OVER - LET'S CHECK THE CHIP ID!*******

  if (bme680.init() != 0x61)
  {    
    //If the Arduino fails to identify the BME680, program stops and shows the Troubleshooting guide
      
    
    Serial.println(F("Ops! BME680 could not be found!"));
    Serial.println(F("Please check your connections."));
    Serial.println();
    Serial.println(F("Troubleshooting Guide"));
    Serial.println(F("*************************************************************"));
    Serial.println(F("1. Let's check the basics: Are the VCC and GND pins connected correctly? If the BME680 is getting really hot, then the wires are crossed."));
    Serial.println();
    Serial.println(F("2. Are you using the I2C mode? Did you connect the SDI pin from your BME680 to the SDA line from the Arduino?"));
    Serial.println();
    Serial.println(F("3. And did you connect the SCK pin from the BME680 to the SCL line from your Arduino?"));
    Serial.println();
    Serial.println(F("4. Are you using the alternative I2C Address(0x76)? Did you remember to connect the SDO pin to GND?"));
    Serial.println();
    Serial.println(F("5. If you are using the default I2C Address (0x77), did you remember to leave the SDO pin unconnected?"));
    Serial.println();
    Serial.println(F("6. Are you using the SPI mode? Did you connect the Chip Select (CS) pin to the pin 10 of your Arduino (or to wherever pin you choosed)?"));
    Serial.println();
    Serial.println(F("7. Did you connect the SDI pin from the BME680 to the MOSI pin from your Arduino?"));
    Serial.println();
    Serial.println(F("8. Did you connect the SDO pin from the BME680 to the MISO pin from your Arduino?"));
    Serial.println();
    Serial.println(F("9. And finally, did you connect the SCK pin from the BME680 to the SCK pin from your Arduino?"));
    Serial.println();
    
    while(1);
  }   
    else
  {
    Serial.println(F("BME680 detected!"));
  }
  Serial.println();
}

#define LPP_TEMPERATURE         103     // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY   104     // 1 byte, 0.5% unsigned
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned

void bme680_get(){
  bme680.writeCTRLMeas();

  uint8_t cnt = 0;
  float Temperature = bme680.readTempC();
  float Humidity = bme680.readHumidity();
  float Pressure = bme680.readPressure();
  float AltitudeMeter = bme680.readAltitudeMeter();
//  sprintf((char *)lora_data, "%.2fC,%.2f%%", bme680.readTempC(), bme680.readHumidity());
//  lora_count = strlen((char *)lora_data);

  Serial.print(F("Temperature in Celsius:\t\t")); 
  Serial.println(bme680.readTempC());

  Serial.print(F("Humidity in %:\t\t\t")); 
  Serial.println(bme680.readHumidity());

  Serial.print(F("Pressure in hPa:\t\t")); 
  Serial.println(bme680.readPressure());

  Serial.print(F("Altitude in Meters:\t\t"));
  Serial.println(bme680.readAltitudeMeter());

  // Temperature
  lora_data[cnt++] = 0x01;
  lora_data[cnt++] = LPP_TEMPERATURE;
  lora_data[cnt++] = (int16_t)(Temperature*10)>>8;
  lora_data[cnt++] = (int16_t)(Temperature*10);

  // Humidity
  lora_data[cnt++] = 0x02;
  lora_data[cnt++] = LPP_RELATIVE_HUMIDITY;
  lora_data[cnt++] = Humidity*2;

  // Pressure
  lora_data[cnt++] = 0x03;
  lora_data[cnt++] = LPP_BAROMETRIC_PRESSURE;
  lora_data[cnt++] = (int16_t)(Pressure*10)>>8;
  lora_data[cnt++] = (int16_t)(Pressure*10);

  lora_count = cnt;   // lora data len
  
  Serial.println();
}