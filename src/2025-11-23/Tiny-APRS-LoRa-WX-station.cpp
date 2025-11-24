/*
Tiny WX station by Patrick EGLOFF TK5EP
Test software for an Low power WX station LoRa with Arduino Pro mini with SX1278 LoRa module
version 2025-02-27

Arduino mini 3V3 8MHz and SX1278 LoRa module.
BME280/SHT31 on pin A4 SDA & A5 SCL

The LoRa library uses default pins SS=10, RST=9, DIO0=2
Change these pins according to your board & wiring
LoRa.setPins(10, 4, 2); // pins for the PCB of uni Pau

In the readVddc() function, adjust the value in line :
result = 1125300L / result;
to match the internal ref voltage that can vary a bit and to read a perfect voltage value.

Updates :
2025-11-23 Some cosmetic changes, low battery routine modification, moved SHT31 into a function readSHT().
2025-02-27 Added SHT31 sensor, due to constant BM2280 humidity sensor failure
*/

#include <SPI.h>
#include <LoRa.h>
#include <forcedBMX280.h> // https://github.com/soylentOrange/Forced-BMX280
#include <LowPower.h>     // https://github.com/LowPowerLab/LowPower
#include <Wire.h>
#include <SHT31.h>

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/
// change these values to your needs
const String CALLSIGN   = "TK5EP-11";  // callsign with SSID

/*
// QRA
const String LATITUDE   = "4156.94N";  // APRS latitude coordinates. Go on my map to find them htpp://egloff.eu/qralocator
const String LONGITUDE  = "00845.25E"; // APRS longitude coordinates
const int ALTITUDE      = 60;          // altitude in meters
*/

// Rucher 
const String LATITUDE   = "4158.46N";  // APRS latitude coordinates. Go on my map to find them htpp://egloff.eu/qralocator
const String LONGITUDE  = "00849.35E"; // APRS longitude coordinates
const int ALTITUDE      = 40;          // altitude in meters


// LoRa parameters
const long TXFREQUENCY  = 433775000;   // Tx frequency in Hz
const int FREQ_ERR      = -17000;      // Freq error in Hz. Start with 0, and measure real frequency and correct this value.
const byte TXPOWER      = 20;          // in dBm, output power on the SX1278 module, max 20 dBm
#define LoRa_SS         10 // Lora module interface
#define LoRa_RST        4
#define LoRa_DIO0       2

// other parameters
const int TXPERIOD      = 600;         // in seconds, interval between 2 transmissions
float LOWBATTERYVOLTAGE = 3.60;        // for 18650 LiIon battery 3.6V voltage threshold. Under this voltage, there will be no transmission in order to save the battery.
#define WITH_SEALEVELCORRECTION       // if we want a pressure correction for the given altitude.
// enable only one of the following sensors
#define SHT31_ENABLE                  // uncomment if we want a SHT31 sensor
//#define BME280_ENABLE               // uncomment if we want a BME280 sensor

// do not change these variables
String SOFTWARE_DATE = "2025-11-23";
float temperature;                    // current temperature
float pressure;                       // current pressure
float humidity;                       // current humidity
int tempAPRS  = 0;
int humiAPRS  = 0;
int pressAPRS = 0;
char buffer[15];
float batteryvoltage = 0;
String packet ="";
String status ="";

// check if only one sensor has been selected
#ifdef BME280_ENABLE
  #undef SHT31_ENABLE
  #define I2C_ADDRESS   0x76          // SHT31 i2C default address
  ForcedBME280Float climateSensor = ForcedBME280Float(Wire,I2C_ADDRESS);  // init object for BME280
#endif
#ifdef SHT31_ENABLE
  #undef BME280_ENABLE
  #define SHT31_ADDRESS   0x44          // SHT31 i2C default address
  SHT31 sht(SHT31_ADDRESS);            // SHT31 init
#endif  


/*******************************
 _____                 _   _                 
|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 

*******************************/

/************************************************************************
 *   Calculates the pressure at sea level (in hPa) from the specified
 *   @param  altitude      Altitude in meters
 *   @param  p0            Measured atmospheric pressure in hPa
 *   @returns              the pressure at sea level (in hPa) from the specified altitude
 *************************************************************************/
float seaLevelForAltitude(float altitude, float p0) {
  return p0 / pow(1.0 - (altitude / 44330.0), 5.255);
}

/************************************************************************
 *   Reads the BME280 sensor and converts to APRS compatible values
 *************************************************************************/
#ifdef BME280_ENABLE
void readBME() {
      climateSensor.takeForcedMeasurement();
      temperature = climateSensor.getTemperatureCelsiusAsFloat(true);
      pressure    = climateSensor.getPressureAsFloat();
      humidity    = climateSensor.getRelativeHumidityAsFloat();
   
      #ifdef WITH_SEALEVELCORRECTION
          pressure = seaLevelForAltitude(ALTITUDE, pressure);
      #endif
      tempAPRS  = int((temperature * 1.8) + 32);
      humiAPRS  = int(round(humidity));
      pressAPRS = int(pressure * 10);
}
#endif

/************************************************************************
 *   Reads the SHT31 sensor and converts to APRS compatible values
 *************************************************************************/
#ifdef SHT31_ENABLE
void readSHT() {
          if (sht.isConnected()) { 
          sht.read();
          temperature = sht.getTemperature();
          humidity  = sht.getHumidity();
          tempAPRS  = int((temperature * 1.8) + 32);
          humiAPRS  = int(round(humidity));
        }
}
#endif

/************************************************************************
 *   Print all datas
 *************************************************************************/
void printDatas() {
      Serial.print(F("Voltage = "));
      Serial.print(batteryvoltage);
      Serial.println(F(" V"));
      Serial.print(F("Temperature: "));
      Serial.print(temperature);
      Serial.println(F(" °C"));
      Serial.print("Humidity : ");
      Serial.print(humidity);
      Serial.println(F(" %"));
      #ifdef BME280_ENABLE
        Serial.print(F("Pressure: "));
        Serial.print(pressure);
        Serial.println(F(" hPa"));
      #endif  
}

void printSetup() {
      Serial.print(F("Callsign : "));
      Serial.println(CALLSIGN);
      Serial.print(F("Latitude : "));
      Serial.println(LATITUDE);
      Serial.print(F("Longitude: "));
      Serial.println(LONGITUDE);
      Serial.print(F("Altitude : "));
      Serial.print(ALTITUDE);
      Serial.println(F(" m"));
      Serial.print(F("TX freq  : "));
      Serial.print(TXFREQUENCY);
      Serial.println(F(" Hz"));
      Serial.print(F("TX offset: "));
      Serial.print(FREQ_ERR);
      Serial.println(F(" Hz"));      
      Serial.print(F("Power    : "));
      Serial.print(TXPOWER);
      Serial.println(F(" dBm"));
      Serial.print(F("TX period: "));
      Serial.print(TXPERIOD);
      Serial.println(F(" s"));
      Serial.print(F("Low battery threshold: "));    
      Serial.print(LOWBATTERYVOLTAGE);
      Serial.println(F(" V"));
      Serial.println(F(""));
}
/************************************************************************
 *   Build the WX datas packet
 *************************************************************************/  
String buildPacket(){
  #ifdef BME280_ENABLE
      // if humidity = 100%, transmit h00 as requested by APRS protocol
      if ( humiAPRS == 100 ) {
        sprintf(buffer, "t%03dh00b%05d",
                        tempAPRS,
                        pressAPRS);
      }
      // otherwise send measured values
      else {
      sprintf(buffer, "t%03dh%02db%05d",
                        tempAPRS,
                        humiAPRS,
                        pressAPRS);
      }
  #endif
  #ifdef SHT31_ENABLE
      if ( humiAPRS == 100 ) sprintf(buffer, "t%03dh00b", tempAPRS);
      else sprintf(buffer, "t%03dh%02db.....",tempAPRS,humiAPRS);
  #endif
                    
      String packet = CALLSIGN + ">APEP02,WIDE1-1:!" + LATITUDE + "/" + LONGITUDE + "_.../...g..." + buffer + "(Bat=" + batteryvoltage + "V)";
      return packet;
}

/************************************************************************
 *   Build the WX status packet
 *************************************************************************/
String buildStatus(){
  String datas = CALLSIGN + ">APEP02,WIDE1-1:>Tiny LoRa APRS WX station by TK5EP vers. " + SOFTWARE_DATE;
  return datas;
}

/************************************************************************
*   Send the packet load
************************************************************************/
void sendPacket(String packet) {
      while (LoRa.beginPacket() == 0) {
        Serial.print(F("waiting for radio ... "));
        delay(100);
      }
      Serial.print(F("Sending packet: "));

      // send packet
      LoRa.beginPacket();
      LoRa.write('<');
      LoRa.write(0xFF);
      LoRa.write(0x01);
      
      Serial.println(packet);
      LoRa.write((const uint8_t *)packet.c_str(), packet.length());
      LoRa.endPacket();
}

/************************************************************************
* Measures the supply voltage of Arduino. Doesn't need any external hardware
* https://forum.arduino.cc/t/can-an-arduino-measure-the-voltage-of-its-own-power-source/669954/4
*************************************************************************/

long readVcc() { 
    long result;
      // Read 1.1V reference against AVcc
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
      delay(2); // Wait for Vref to settle
      ADCSRA |= _BV(ADSC); // Convert
      while (bit_is_set(ADCSRA,ADSC));
      result = ADCL;
      result |= ADCH<<8;
      result = 1100000L / result; // Back-calculate AVcc in mV
      //Serial.print("Voltage = ");
      //Serial.print(result/1000.00);
      //Serial.println(" V");
	    //Serial.flush();
      return result;
}

/****************************************************************************
* function to put the 328P into sleep
* called by longSleep (time_in_seconds)
* time is approximative, depends on the 328P clock accuracy and wake times
* TK5EP oct 2025
*****************************************************************************/
 void longSleep( uint16_t sleepInSeconds )
{
  Serial.print(F("Sleeping for "));
  Serial.print(sleepInSeconds);
  Serial.println(F(" seconds"));
  Serial.flush();
  if ( sleepInSeconds & 0x01 ) LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  if ( sleepInSeconds & 0x02 ) LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  if ( sleepInSeconds & 0x04 ) LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
 
  while ( sleepInSeconds & 0xFFF8 ) {
    sleepInSeconds -= 8;
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  };
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/
void setup() {
  Serial.begin(115200);
  while (!Serial) { 
    delay(10);
  }
  Wire.begin(); // needed for climatesensor

  Serial.print(F("Tiny LoRa APRS WX station by TK5EP v. "));
  Serial.println(SOFTWARE_DATE);

  #ifdef BME280_ENABLE
    // try to initiate the BME280
    while (climateSensor.begin()) {
      Serial.println(F("Waiting for BME280..."));
      delay(1000);
    }
    Serial.println(F("BME280 init succeeded."));
  #endif

  #ifdef SHT31_ENABLE
    //Wire.begin();
    Wire.setClock(100000);
    sht.begin();
    uint16_t sht_status = sht.isConnected();
    if (!sht_status) Serial.println(F("No SHT detected"));
    else Serial.println(F("SHT31 detected"));
  #endif

  // initiate the SX1278
  LoRa.setPins(LoRa_SS, LoRa_RST, LoRa_DIO0); // 10,4,22
  if (!LoRa.begin(TXFREQUENCY+FREQ_ERR)) {
    Serial.println(F("Starting LoRa failed!"));
    while (1);
  }
  else {
    Serial.println(F("LoRa init succeeded."));
  }

  // print the global configuration
    printSetup();

  // LoRa module setup
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(TXPOWER);

  // build the APRS status load
  status = buildStatus();
  // send the status packet
  sendPacket(status);
  // go to sleep before running loop to let time to digis to repeat packet
  longSleep(5);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/
void loop() {
    // read battery voltage
    batteryvoltage = readVcc()/1000.00;
    // read the battery voltage. If under the threshold, go into power save mode. No measure, no transmitting.
    if (batteryvoltage >= LOWBATTERYVOLTAGE)
    {
      // read sensor
      #ifdef BME280_ENABLE
        readBME();
      #endif  
      #ifdef SHT31_ENABLE
        readSHT();
       #endif   
      // display datas
      printDatas();
      // build datas packet and send it
      packet = buildPacket();
      sendPacket(packet);
      Serial.flush();
      // put the LoRa module in sleep mode
      LoRa.sleep(); 
      // put the 328P in power down mode for the TXPERIOD delay
      longSleep(TXPERIOD);
    }
    else
    {
      Serial.print(F("Voltage = "));
      Serial.print(batteryvoltage);
      Serial.println(F(" V"));
      Serial.println(F("Low battery detected! Going into sleep. No measures nor transmitting. Waiting voltage to raise."));
      Serial.flush();
      LoRa.sleep();
      //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      // sleep for 30 min. Wait until the voltage is higher then LOWBATTERYVOLTAGE
      longSleep(1800);
    }

 } // end loop