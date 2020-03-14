#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include "GravityTDS.h"
//#include <OneWire.h>
//#include <DallasTemperature.h>

// Sensor List
#define TdsSensorPin A1
#define TEMPERATURE_SENSOR 8
#define waterPump1 2
#define waterPump2 3

// Tags
#define TDS_TAG "TDS"
#define SETPOINT_TAG "setpoint"
#define TEMPERATURE_TAG "temperature"

GravityTDS gravityTds;
SoftwareSerial s(5, 6);
//OneWire oneWire(TEMPERATURE_SENSOR);
//DallasTemperature temperatureSensor(&oneWire);

const char startMarker = '|';
const char endMarker = '~';
float temperature = 25, tdsValue = 0, setPoint = 500;
const byte numChars = 950;
char receivedChars[numChars];
bool newData = false;

void setup()
{
  Serial.begin(115200);
  s.begin(115200);
  setupTDS();
  pinMode(waterPump1, OUTPUT);
  pinMode(waterPump2, OUTPUT);
  pinMode(TEMPERATURE_SENSOR, INPUT);
}

void loop()
{
  transmitDataToNodeMCU(tdsValue, setPoint, temperature, startMarker, endMarker);
  checkTdsForWatering(tdsValue);
  receiveDataFromNodeMCU(setPoint, startMarker, endMarker);

  Serial.println(".");
  delay(1000);
}

void setupTDS() {
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
}

//float detectTemperature() {
//  temperatureSensor.requestTemperatures();
//  return temperatureSensor.getTempCByIndex(0);
//}

float detectTds(float temperature) {
  gravityTds.setTemperature(temperature);
  gravityTds.update();
  return gravityTds.getTdsValue();
}

void checkTdsForWatering(float tdsValue) {
  if (tdsValue < setPoint) {
    digitalWrite(waterPump1, LOW);
    digitalWrite(waterPump2, LOW);
  } else {
    digitalWrite(waterPump1, HIGH);
    digitalWrite(waterPump2, HIGH);
  }
}

void transmitDataToNodeMCU(float& oldTds, float& oldSetPoint, float& oldTemperature, char startMarker, char endMarker) {
  float tds = detectTds(oldTemperature); // TODO update when temperature is working
  if (oldTds != tds) {
    String json, transmittedValue;
    DynamicJsonBuffer jb;
    JsonObject& obj = jb.createObject();
    obj[TDS_TAG] = tds;
    obj[SETPOINT_TAG] = setPoint;
    obj[TEMPERATURE_TAG] = temperature;
    obj.printTo(json);
    transmittedValue = startMarker + json + endMarker;
    Serial.println("-----------TRANSMITING data to NodeMCU-----------");
    Serial.println(transmittedValue);
    Serial.println("----------------END OF TRANSMITING---------------");
    s.print(transmittedValue);
    oldTds = tds;
  }
}

void receiveDataFromNodeMCU(float& setPoint, char startMarker, char endMarker) {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char rc;

  while (s.available() > 0 && newData == false) {
    rc = s.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        s.flush();
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  processNewData(setPoint);
}

void processNewData(float& setPoint) {
  if (newData == true) {
    DynamicJsonBuffer jb;
    JsonObject& obj = jb.parseObject(receivedChars);
    if (!obj.success()) {
      Serial.println("Gagal mengubah json");
      Serial.println(receivedChars);
      newData = false;
      return;
    }
    Serial.println("----------------Receiving Object from NodeMCU---------------");
    obj.prettyPrintTo(Serial);
    Serial.println("----------------------End of Receiving----------------------");
    updateSetPoint(obj, setPoint);
    newData = false;
  }
}

void updateSetPoint(JsonObject& obj, float& setPoint) {
  String receivedSetPoint = obj[SETPOINT_TAG];
  Serial.println(receivedSetPoint);
  Serial.println(obj[SETPOINT_TAG] != NULL);
  Serial.println(isValidNumber(receivedSetPoint));
  if(obj[SETPOINT_TAG] != NULL && isValidNumber(receivedSetPoint)) {
    setPoint = obj[SETPOINT_TAG];
  }
  Serial.print("current setpoint: ");
  Serial.print(setPoint);
}

boolean isValidNumber(String str)
{
   boolean isNum=false;
   if(!(str.charAt(0) == '+' || str.charAt(0) == '-' || isDigit(str.charAt(0)))) return false;

   for(byte i=1;i<str.length();i++)
   {
       if(!(isDigit(str.charAt(i)) || str.charAt(i) == '.')) return false;
   }
   return true;
}
