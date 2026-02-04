/*
 * EEPROM manager implementation
 * Реализация менеджера EEPROM
 */

#include "eeprom_manager.h"

uint8_t calculateChecksum(const uint8_t* data, size_t size) {
  uint8_t sum = 0;
  for (size_t i = 0; i < size; i++) {
    sum ^= data[i]; // Simple XOR for checksum / Простой XOR для контрольной суммы
  }
  return sum;
}

void saveSettingsToEEPROM(ClimateController& controller) {
  StoredSettings settings;
  
  // Get current settings / Получаем текущие настройки
  settings.climate.target_temperature = controller.getTemperatureSetpoint();
  settings.climate.target_humidity = controller.getHumiditySetpoint();
  settings.pidFactors = controller.getTemperaturePID();
  
  // Get light schedule / Получаем расписание освещения
  LightScheduler& scheduler = controller.getLightScheduler();
  uint8_t onHour, onMinute, offHour, offMinute;
  scheduler.getLightOnTime(onHour, onMinute);
  scheduler.getLightOffTime(offHour, offMinute);
  settings.climate.lightOn_hour = onHour;
  settings.climate.lightOn_minute = onMinute;
  settings.climate.lightOff_hour = offHour;
  settings.climate.lightOff_minute = offMinute;
  settings.climate.lightScheduleEnabled = scheduler.isScheduleEnabled();
  
  // Get calibration / Получаем калибровку
  SensorReader& sensorReader = controller.getSensorReader();
  settings.calibration = sensorReader.getCurrentCalibration();
  
  // Calculate checksum / Вычисляем контрольную сумму
  settings.checksum = calculateChecksum(
    (uint8_t*)&settings, 
    sizeof(StoredSettings) - sizeof(uint8_t)
  );
  
  // Save to EEPROM / Сохраняем в EEPROM
  EEPROM.put(0, settings);
  
  #ifdef DEBUG_MODE
    Serial.println(F("Settings saved to EEPROM / Настройки сохранены в EEPROM"));
  #endif
}

bool loadSettingsFromEEPROM(ClimateController& controller) {
  StoredSettings settings;
  
  // Read from EEPROM / Читаем из EEPROM
  EEPROM.get(0, settings);
  
  // Verify checksum / Проверяем контрольную сумму
  uint8_t calculatedChecksum = calculateChecksum(
    (uint8_t*)&settings, 
    sizeof(StoredSettings) - sizeof(uint8_t)
  );
  
  if (settings.checksum != calculatedChecksum) {
    #ifdef DEBUG_MODE
      Serial.println(F("EEPROM checksum error, using defaults / Ошибка контрольной суммы EEPROM, использую значения по умолчанию"));
    #endif
    return false;
  }
  
  // Apply settings / Применяем настройки
  controller.setTemperatureSetpoint(settings.climate.target_temperature);
  controller.setHumiditySetpoint(settings.climate.target_humidity);
  controller.setTemperaturePID(settings.pidFactors);
  
  // Apply light schedule / Применяем расписание освещения
  LightScheduler& scheduler = controller.getLightScheduler();
  scheduler.setLightOnTime(settings.climate.lightOn_hour, settings.climate.lightOn_minute);
  scheduler.setLightOffTime(settings.climate.lightOff_hour, settings.climate.lightOff_minute);
  scheduler.enableSchedule(settings.climate.lightScheduleEnabled);
  
  // Apply calibration / Применяем калибровку
  SensorReader& sensorReader = controller.getSensorReader();
  sensorReader.setCalibration(settings.calibration);
  
  #ifdef DEBUG_MODE
    Serial.println(F("Settings loaded from EEPROM / Настройки загружены из EEPROM"));
  #endif
  return true;
}