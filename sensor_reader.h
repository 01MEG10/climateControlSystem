/*
 * Sensor reading class with calibration and filtering
 * Класс чтения датчиков с калибровкой и фильтрацией
 */

#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <Arduino.h>
#include <DHT.h>
#include "config.h"
#include "structures.h"

class SensorReader {
private:  
  SensorCalibration sensor_calibration;
  SensorInfo sensor_info;
  DHT dht;
  
  // Filter buffers / Буферы для фильтрации
  static const int FILTER_SIZE = 5;
  int lightBuffer[FILTER_SIZE] = {0};
  int lightBufferIndex = 0;
  int readErrors;
  
  void resetToInvalid() {
    sensor_info = SensorInfo();
  }
  
  bool checkMeasurementsValid() const {
    return !isnan(sensor_info.temperature) && 
           !isnan(sensor_info.humidity) && 
           sensor_info.light != INVALID_INT;
  }
  
  int applyFilter(int* buffer, int& index, int newValue) {
    buffer[index] = newValue;
    index = (index + 1) % FILTER_SIZE;
    
    long sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      sum += buffer[i];
    }
    return sum / FILTER_SIZE;
  }
  
  int readAndCalibrateLight() {
    int raw = analogRead(PIN_LIGHT_SENSOR);
    
    // Protection against incorrect values / Защита от некорректных значений
    if (raw < 0 || raw > ADC_MAX_VALUE * 1.1) { // Allow 10% overrun / Допускаем 10% превышение
      return INVALID_INT;
    }
    
    int calibrated = constrain(raw + sensor_calibration.light, 0, ADC_MAX_VALUE);
    return applyFilter(lightBuffer, lightBufferIndex, calibrated);
  }
  
  void checkSensorSanity() {
    // Check for clearly unrealistic values / Проверка на явно нереальные значения
    if (!isnan(sensor_info.temperature)) {
      if (sensor_info.temperature < -50 || sensor_info.temperature > 150) {
        sensor_info.temperature = INVALID_FLOAT;
        readErrors++;
      }
    }
    
    if (!isnan(sensor_info.humidity)) {
      if (sensor_info.humidity < 0 || sensor_info.humidity > 100) {
        sensor_info.humidity = INVALID_FLOAT;
        readErrors++;
      }
    }
  }
  
public: 
  SensorReader() : 
    dht(PIN_DHT_SENSOR, DHT_TYPE),
    readErrors(0)
  { 
    dht.begin();
    delay(100); // Give time for sensor initialization / Даем время на инициализацию датчика
    
    // Initialize filter buffers / Инициализация буферов фильтра
    for (int i = 0; i < FILTER_SIZE; i++) {
      lightBuffer[i] = analogRead(PIN_LIGHT_SENSOR);
      delay(10);
    }
  }
  
  void readAll() {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    if (isnan(temperature) || isnan(humidity)) {
      readErrors++;
      resetToInvalid();
      return;
    }
    
    sensor_info.temperature = temperature + sensor_calibration.temperature;
    sensor_info.humidity = humidity + sensor_calibration.humidity;
    sensor_info.light = readAndCalibrateLight();
    
    checkSensorSanity();
    sensor_info.measurement_ok = checkMeasurementsValid();
    
    if (sensor_info.measurement_ok) {
      readErrors = 0;
    }
  }
  
  void setCalibration(const SensorCalibration& new_calib) {
    sensor_calibration = new_calib;
    
    // Limit calibration values / Ограничение калибровочных значений
    sensor_calibration.temperature = constrain(sensor_calibration.temperature, -10.0f, 10.0f);
    sensor_calibration.humidity = constrain(sensor_calibration.humidity, -50.0f, 50.0f);
    sensor_calibration.light = constrain(sensor_calibration.light, -ADC_MAX_VALUE, ADC_MAX_VALUE);
  }
  
  const SensorInfo& getSensorInfo() const {
    return sensor_info;
  }
  
  const SensorCalibration& getCurrentCalibration() const {
    return sensor_calibration;
  }
  
  int getErrorCount() const {
    return readErrors;
  }
  
  static int getADCResolution() {
    return ADC_BITS;
  }
  
  static int getADCMaxValue() {
    return ADC_MAX_VALUE;
  }
};

#endif // SENSOR_READER_H