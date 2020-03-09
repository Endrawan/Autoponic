/***************************************************
  DFRobot Gravity: Analog TDS Sensor/Meter
  <https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244>

 ***************************************************
  This sample code shows how to read the tds value and calibrate it with the standard buffer solution.
  707ppm(1413us/cm)@25^c standard buffer solution is recommended.

  Created 2018-1-3
  By Jason <jason.ling@dfrobot.com@dfrobot.com>

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution.
 ****************************************************/

/***********Notice and Trouble shooting***************
  1. This code is tested on Arduino Uno with Arduino IDE 1.0.5 r2 and 1.8.2.
  2. Calibration CMD:
    enter -> enter the calibration mode
    cal:tds value -> calibrate with the known tds value(25^c). e.g.cal:707
    exit -> save the parameters and exit the calibration mode
****************************************************/

#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "GravityTDS.h"

// Sensor List
#define TdsSensorPin A1
#define waterTemperatureSensor A2
#define waterPump1 2
#define waterPump2 3

GravityTDS gravityTds;
SoftwareSerial s(5, 6);
float temperature = 25, tdsValue = 0, setPoint = 500;

void setup()
{
  Serial.begin(115200);
  s.begin(115200);
  setupTDS();
  pinMode(waterPump1, OUTPUT);
  pinMode(waterPump2, OUTPUT);
}

void loop()
{
  detectTemperatureAndTds();
  checkTdsForWatering();
  transmitDataToNodeMCU();
  Serial.println();
  delay(1000);
}

void setupTDS() {
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
}

void detectTemperatureAndTds() {
  //TODO add detect temperature code
  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  tdsValue = gravityTds.getTdsValue();  // then get the value
  Serial.print("Nilai Tds: ");
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
}

void checkTdsForWatering() {
  if (tdsValue < setPoint) {
    digitalWrite(waterPump1, HIGH);
  } else {
    digitalWrite(waterPump1, LOW);
  }
}

void transmitDataToNodeMCU() {
  String json, transmittedValue;
  DynamicJsonBuffer jb;
  JsonObject& obj = jb.createObject();
  obj["TDS"] = tdsValue;
  obj["TDS_setpoint"] = setPoint;
  obj["temperature"] = temperature;
  obj.printTo(json);
  transmittedValue = "^" + json + "~";
  Serial.println("-----------TRANSMITING data to NodeMCU-----------");
  Serial.println(transmittedValue);
  Serial.println("----------------END OF TRANSMITING---------------");
  s.print(transmittedValue);
  json.remove(0, json.length());
  transmittedValue.remove(0, transmittedValue.length());
}
