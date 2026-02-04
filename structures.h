/*
 * Data structures for Climate Control System
 * Структуры данных системы контроля климата
 */

#ifndef STRUCTURES_H
#define STRUCTURES_H

#include "config.h"
#include <math.h>

// Forward declarations / Предварительные объявления
class ClimateController;
class LCDDisplay;
class RotaryEncoder;

// Menu states / Состояния меню
enum MenuState {
  MAIN_SCREEN,           // Главный экран / Main screen
  MAIN_MENU,             // Главное меню / Main menu
  TEMPERATURE_SETTINGS,  // Настройки температуры / Temperature settings
  HUMIDITY_SETTINGS,     // Настройки влажности / Humidity settings
  LIGHT_SETTINGS,        // Настройки освещения / Light settings
  TIME_SETTINGS,         // Настройки времени / Time settings
  SAVE_CONFIRM,          // Подтверждение сохранения / Save confirmation
  SYSTEM_INFO           // Информация о системе / System info
};

// ==================== DATA STRUCTURES / СТРУКТУРЫ ДАННЫХ ====================

struct PIDFactor {
  double Kp;        // Proportional coefficient / Коэффициент пропорциональности
  double Ki;        // Integral coefficient / Коэффициент интегральности
  double Kd;        // Derivative coefficient / Коэффициент производности
  int outputMin;    // Minimum output value / Минимальное выходное значение
  int outputMax;    // Maximum output value / Максимальное выходное значение

  PIDFactor() : 
    Kp(1.0),  
    Ki(0.5), 
    Kd(0.02),
    outputMin(0),
    outputMax(255)
  { }
};

struct SensorInfo {
  float temperature;      // Temperature in Celsius / Температура в градусах Цельсия
  float humidity;         // Humidity in percentage / Влажность в процентах
  int light;              // Light intensity / Интенсивность освещения
  bool measurement_ok;    // Measurement success flag / Флаг успешности измерений

  SensorInfo() : 
    temperature(INVALID_FLOAT),
    humidity(INVALID_FLOAT),
    light(INVALID_INT),
    measurement_ok(false)
  { }
};

struct SensorCalibration {
  float temperature;      // Temperature calibration offset / Калибровочное смещение температуры
  float humidity;         // Humidity calibration offset / Калибровочное смещение влажности
  int light;              // Light calibration offset / Калибровочное смещение освещенности

  SensorCalibration() : 
    temperature(0),
    humidity(0),
    light(0)
  { }
};

struct ClimateSetpoint {
  float target_temperature;  // Target temperature / Целевая температура
  float target_humidity;     // Target humidity / Целевая влажность
  int lightOn_hour;          // Light ON hour (0-23) / Час включения света (0-23)
  int lightOn_minute;        // Light ON minute (0-59) / Минута включения света (0-59)
  int lightOff_hour;         // Light OFF hour (0-23) / Час выключения света (0-23)
  int lightOff_minute;       // Light OFF minute (0-59) / Минута выключения света (0-59)
  bool lightScheduleEnabled; // Light schedule enabled / Расписание освещения включено
  
  ClimateSetpoint() : 
    target_temperature(25.0f),
    target_humidity(65.0f),
    lightOn_hour(DEFAULT_LIGHT_ON_HOUR),
    lightOn_minute(0),
    lightOff_hour(DEFAULT_LIGHT_OFF_HOUR),
    lightOff_minute(0),
    lightScheduleEnabled(LIGHT_SCHEDULE_ENABLED)
  { }
};

struct DateTime {
  uint8_t year;     // Year (0-99) / Год (0-99)
  uint8_t month;    // Month (1-12) / Месяц (1-12)
  uint8_t day;      // Day (1-31) / День (1-31)
  uint8_t hour;     // Hour (0-23) / Час (0-23)
  uint8_t minute;   // Minute (0-59) / Минута (0-59)
  uint8_t second;   // Second (0-59) / Секунда (0-59)
  uint8_t weekday;  // Weekday (1-7, 1=Sunday) / День недели (1-7, 1=воскресенье)
  
  DateTime() : 
    year(0),
    month(0),
    day(0),
    hour(0),
    minute(0),
    second(0),
    weekday(0)
  { }
};

// System status structure / Структура статуса системы
struct SystemStatus {
  float currentTemperature;
  float targetTemperature;
  float currentHumidity;
  float targetHumidity;
  int pidOutput;
  bool overheatProtection;
  bool humidityWorking;
  int lightLevel;
  int errorCount;
  bool systemEnabled;
  bool lightOn;
  bool scheduleEnabled;
  DateTime currentTime;
  
  SystemStatus() : 
    currentTemperature(NAN),
    targetTemperature(25.0f),
    currentHumidity(NAN),
    targetHumidity(65.0f),
    pidOutput(0),
    overheatProtection(false),
    humidityWorking(false),
    lightLevel(0),
    errorCount(0),
    systemEnabled(true),
    lightOn(false),
    scheduleEnabled(true)
  { }
};
// EEPROM settings structure / Структура настроек EEPROM
struct StoredSettings {
  ClimateSetpoint climate;
  PIDFactor pidFactors;
  SensorCalibration calibration;
  uint8_t checksum;
};

#endif // STRUCTURES_H