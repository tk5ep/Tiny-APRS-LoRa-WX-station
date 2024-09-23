/*
Tiny WX station
Test software for an Low power WX station LoRa with Arduino Pro mini with SX1278
2024-01-16 TK5EP

Arduino mini 3V3 8MHz and SX1278 LoRa module.
BME280 on pin A4 SDA & A5 SCL
*/
#include <SPI.h>
#include "LoRa.h"
#include <forcedBMX280.h> // https://github.com/soylentOrange/Forced-BMX280
#include <LowPower.h>     //https://github.com/LowPowerLab/LowPower

#define WITH_SEALEVELCORRECTION // if we want a pressure correction for the given altitude.

const String CALLSIGN  = "TK5EP-12";  // callsign with SSID
const String STATUS    = "Tiny LoRa APRS WX station by TK5EP"; // Status text to be sent with Status transmission
const String LATITUDE  = "4156.94N";  // APRS latitude coordinates. Go on my map to find them htpp://egloff.eu/qralocator
const String LONGITUDE = "00845.25E"; // APRS longitude coordinates
const int ALTITUDE     = 60;          // altitude in meters
const long TXFREQUENCY = 433775000;   // Tx frequency in Hz
const byte TXPOWER     = 20;          // in dBm, output power on the SX1278 module, max 20 dBm
const int TXPERIOD     = 300;         // in seconds, interval between 2 transmissions

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
float temperature;    // current temperature
float pressure;       // current pressure
float humidity;       // current humidity
int tempAPRS  = 0;
int humiAPRS  = 0;
int pressAPRS = 0;
char buffer[15];
float batteryvoltage = 0;
byte counter=0;
String packet ="";
String status ="";

// init object for BME280
ForcedBME280Float climateSensor = ForcedBME280Float();

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
void readBME() {
    climateSensor.takeForcedMeasurement();
      temperature = climateSensor.getTemperatureCelsiusAsFloat(true);
      pressure    = climateSensor.getPressureAsFloat();
      humidity    = climateSensor.getRelativeHumidityAsFloat();
      
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
      Serial.print("Humidity : ");
      Serial.print(humidity);
      Serial.println(" %");
      Serial.print("Pressure: ");
      Serial.print(pressure);
      Serial.println(" hPa");

      #ifdef WITH_SEALEVELCORRECTION
          pressure = seaLevelForAltitude(ALTITUDE, pressure);
      #endif
      tempAPRS  = int((temperature * 1.8) + 32);
      humiAPRS  = int(round(humidity));
      pressAPRS = int(pressure * 10);
}

/************************************************************************
 *   Builds the packet content
 *************************************************************************/  
String buildPacket(){
      sprintf(buffer, "t%03dh%02db%05d",
                        tempAPRS,
                        humiAPRS,
                        pressAPRS
                        );
      String packet = CALLSIGN + ">APEP02,WIDE1-1:!" + LATITUDE + "/" + LONGITUDE + "_.../...g..." + buffer + "(Bat=" + batteryvoltage + "V)";
      return packet;
}

/************************************************************************
 *   Builds the WX DATAS packet
 *************************************************************************/
String buildStatus(){
  String datas = CALLSIGN + ">APEP02,WIDE1-1:>" + STATUS;
  return datas;
}

/************************************************************************
 *   Builds thE WX STATUS load
************************************************************************/
void sendPacket(String packet) {
      while (LoRa.beginPacket() == 0) {
        Serial.print("waiting for radio ... ");
        delay(100);
      }
      Serial.print("Sending packet: ");

      // send packet
      LoRa.beginPacket();
      LoRa.write('<');
      LoRa.write(0xFF);
      LoRa.write(0x01);
      
      Serial.println(packet);
      //String newPacket = "TK5EP-12>APWXEP,WIDE1-1:>WX station by TK5EP";
      LoRa.write((const uint8_t *)packet.c_str(), packet.length());
      LoRa.endPacket();
}

/************************************************************************
*   Measures the supply voltage of Arduino. Doesn't need any external hardware
*************************************************************************/
// https://forum.arduino.cc/t/can-an-arduino-measure-the-voltage-of-its-own-power-source/669954/4
long readVcc() { 
    long result;
      // Read 1.1V reference against AVcc
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
      delay(2); // Wait for Vref to settle
      ADCSRA |= _BV(ADSC); // Convert
      while (bit_is_set(ADCSRA,ADSC));
      result = ADCL;
      result |= ADCH<<8;
      result = 1126400L / result; // Back-calculate AVcc in mV
      Serial.print("Voltage = ");
      Serial.print(result/1000.00);
      Serial.println(" V");
	  Serial.flush();
      return result;
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
  // try to initiate the BME280
  while (climateSensor.begin()) {
  Serial.println("Waiting for sensor...");
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(1000);
  }
  // initiate the SX1278
  if (!LoRa.begin(TXFREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa init succeeded.");
  //LoRa.setPins(10, 4, 22); // pins pour platine uni Pau
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.setTxPower(TXPOWER);

  // read the BME280 sensor
  readBME();
  // read the battery voltage
  batteryvoltage = readVcc()/1000.00;
  // build the APRS load
  packet = buildPacket();
  // send the datas packet
  sendPacket(packet);
  delay (5000);
  // build the APRS status load
  status = buildStatus();
  // send the status packet
  sendPacket(status);
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
  // put in sleep modes
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  LoRa.sleep();
  // make several loops depending of the TXPERIOD
  // we go to sleep for 8s at each loop
  ++counter;
  Serial.println(counter);
  Serial.flush();
  // if number of loops reached
  // we measure, build and transmit
  if (counter == round(TXPERIOD/8)) {
    readBME();
    batteryvoltage = readVcc()/1000.00;
    packet = buildPacket();
    sendPacket(packet);
    counter =0;
    }
 } // end loop
