#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include "INA226.h"
#include "ADS1X15.h"

#include "debug.h"
#include "ina219.h"
#include "sensors.h"

const float ina219CalibrationReadingMa = 1000;
const float ina219CalibrationExtMeterReadingMa = 1000;
DFRobot_INA219_IIC ina219_1(&Wire, INA219_I2C_ADDRESS3);  // left motor
DFRobot_INA219_IIC ina219_2(&Wire, INA219_I2C_ADDRESS4);  // right motor

#define INA226_I2C_ADDRESS1 0x40
#define INA226_I2C_ADDRESS2 0x4C
INA226 ina226_1(INA226_I2C_ADDRESS1);
INA226 ina226_2(INA226_I2C_ADDRESS2);

#define BME280_I2C_ADDRESS1 0x76
Adafruit_BME280 bme1; // onboard, I2C address 0x76 (default)

// For ADS1115
// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
ADS1115 ads;

#include "comm_bt.h"
#include "pins.h"

void initSensors() {
  scanI2cDevices();

  Serial.print("INA219_1: ");
  Serial.println(reinitIna219Sensor(&ina219_1));

  Serial.print("INA219_2: ");
  Serial.println(reinitIna219Sensor(&ina219_2));

  Serial.print("BME280 #1:");
  Serial.println(bme1.begin(BME280_I2C_ADDRESS1));

  Serial.print("ADS1115: ");
  Serial.println(ads.begin());
  ads.setGain(1); // max voltage ±4.096V

  Serial.print("INA226_1: ");
  Serial.println(ina226_1.begin());
  ina226_1.setMaxCurrentShunt(20, 0.002);

  Serial.print("INA226_2: ");
  Serial.println(ina226_2.begin());
  ina226_2.setMaxCurrentShunt(20, 0.002);
}

int reinitIna219Sensor(DFRobot_INA219_IIC *ina219) {
  for(int attempt = 0; attempt < 10  && ina219->begin() != true; attempt++);
  
  if (ina219->lastOperateStatus == eIna219_ok) {
    ina219->linearCalibrate(ina219CalibrationReadingMa, ina219CalibrationExtMeterReadingMa);
  }  

  return ina219->lastOperateStatus;
}

int getCurrentFromIna219(Ina219SensorSelector sensorSelector) {
  DFRobot_INA219_IIC* ina219SelectedSensor = sensorSelector == RightMotor ? &ina219_2 : &ina219_1;
  reinitIna219Sensor(ina219SelectedSensor);
  return ina219SelectedSensor->getCurrent_mA();
}

int getVoltageFromIna219(Ina219SensorSelector sensorSelector) {
  DFRobot_INA219_IIC* ina219SelectedSensor = sensorSelector == RightMotor ? &ina219_2 : &ina219_1;
  reinitIna219Sensor(ina219SelectedSensor);
  return ina219SelectedSensor->getBusVoltage_V() * 1000;
}

int getCurrentFromIna226(Ina226SensorSelector sensorSelector) {
  return sensorSelector == BatteryIna226 ? ina226_1.getCurrent_mA() : ina226_2.getCurrent_mA();
}

int getVoltageFromIna226(Ina226SensorSelector sensorSelector) {
  return sensorSelector == BatteryIna226 ? ina226_1.getBusVoltage_mV() : ina226_2.getBusVoltage_mV();
}


int getTemperatureFromBme280(Bme280SensorSelector sensorSelector) {
  return bme1.readTemperature();
}

int getHumidityFromBme280(Bme280SensorSelector sensorSelector) {
  return bme1.readHumidity();
}

int getPressureFromBme280(Bme280SensorSelector sensorSelector) {
  return bme1.readPressure();
}

int getTemperatureFromTmp36(Tmp36SensorSelector sensorSelector) {
  // float volts = multiSampleAnalogRead(PIN_TMP36_MAIN_BAT, 24) / 1023.0; 
  // return (volts - 0.5) * 100;
  return (getVoltageFromAdc(0) - 0.5) * 100;
}

float getVoltageFromAdc(uint8_t channel) {
  int16_t value = ads.readADC(channel);
  return ads.toVoltage(1) * value;
}

int multiSampleAnalogRead(int pin, int sampleCount) {
  long outputSum = 0;
  int count = sampleCount;

  while(count-- > 0) {
    outputSum += analogRead(pin);
  }

  return outputSum / sampleCount;
}

void scanI2cDevices() {
  Serial.println("I2C scan: ");
  for(uint8_t address = 1; address < 127; address++)   {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
 
    if (error == 0) {
      Serial.print("I2C found");
    } else if (error == 4) {
      Serial.print("I2C error");
    } else {
      Serial.print(".");
      continue;
    }

    Serial.print(": 0x");
    if (address < 16) { Serial.print("0"); }
    Serial.println(address, HEX);
  }
}