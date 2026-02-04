/*
 * EEPROM manager for settings storage
 * Менеджер EEPROM для хранения настроек
 */

#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <EEPROM.h>
#include "structures.h"
#include "climate_controller.h"

// Settings structure for EEPROM / Структура настроек для EEPROM
struct StoredSettings {
  ClimateSetpoint climate;
  PIDFactor pidFactors;
  SensorCalibration calibration;
  uint8_t checksum;
};

// Calculate checksum for data integrity / Вычисление контрольной суммы для целостности данных
uint8_t calculateChecksum(const uint8_t* data, size_t size);

// Save settings to EEPROM / Сохранение настроек в EEPROM
void saveSettingsToEEPROM(ClimateController& controller);

// Load settings from EEPROM / Загрузка настроек из EEPROM
bool loadSettingsFromEEPROM(ClimateController& controller);

#endif // EEPROM_MANAGER_H