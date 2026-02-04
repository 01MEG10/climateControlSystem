/*
 * Main climate control system class
 * Главный класс системы контроля климата
 */

#ifndef CLIMATE_CONTROLLER_H
#define CLIMATE_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "structures.h"
#include "sensor_reader.h"
#include "pid_controller.h"
#include "serial_commander.h"
#include "watchdog_manager.h"
#include "rtc_manager.h"
#include "light_scheduler.h"
#include "lcd_display.h"
#include "rotary_encoder.h"
#include "interrupt_handler.h"
#include "safe_timer.h"

class ClimateController {
private:
  ClimateSetpoint climate_setpoint;
  SensorReader sensor_reader;
  PIDController temperature_pid;
  SerialCommander serial_cmd;
  WatchdogManager watchdog;
  RTCManager rtc;
  LightScheduler light_scheduler;
  LCDDisplay* lcd_display;
  RotaryEncoder* encoder;
  
  bool humidity_work_flag;
  bool overheatProtectionActive;
  bool systemEnabled;
  unsigned long overheatStartTime;
  
  // Safe timers / Безопасные таймеры
  SafeTime sensorReadTimer;
  SafeTime humidifierMinWorkTimer;
  SafeTime humidifierMinPauseTimer;
  SafeTime serialUpdateTimer;
  SafeTime statusDisplayTimer;
  SafeTime rtcUpdateTimer;
  
  // For monitoring / Для мониторинга
  unsigned long humidifierStartTime;
  int errorCount;
  int lastLightPWM;
  
  // System status / Статус системы
  SystemStatus systemStatus;
  
  void initializePins();
  void controlLight();
  void setLightPower(int pwmValue);
  void controlHumidity(float humidityLevel);
  void controlTemperature(float temperatureLevel);
  void setHeaterPower(int pwmValue);
  void monitorSystemHealth();
  void setupSerialCallbacks();
  
  // Static wrapper methods for callbacks / Статические методы-обертки для обратных вызовов
  static void onStatusRequestStatic();
  static void onTimeRequestStatic();
  static void onSetTimeStatic(String timeStr);
  static void onSetDateStatic(String dateStr);
  static void onSetLightOnTimeStatic(String timeStr);
  static void onSetLightOffTimeStatic(String timeStr);
  static void onSetLightScheduleStatic(bool enabled);
  static void onSaveRequestStatic();
  static void onLoadRequestStatic();
  static void onResetRequestStatic();
  static void onEmergencyStopStatic();
  static void onTemperatureSetStatic(float temp);
  static void onHumiditySetStatic(float humid);
  static void onWatchdogTestStatic();
  
  // Command handlers / Обработчики команд
  void setTimeFromString(const String& timeStr);
  void setDateFromString(const String& dateStr);
  void setLightOnTimeFromString(const String& timeStr);
  void setLightOffTimeFromString(const String& timeStr);
  
public:
  ClimateController();
  
  static ClimateController climateController;
  
  static ClimateController& getInstance() {
    return climateController;
  }

  void begin();
  
  void update();
  
  void updateSystemStatus();
  
  // Safe control methods / Безопасные методы управления
  void setTemperaturePID(const PIDFactor& factors);
  void setTemperatureSetpoint(float temperature);
  void setHumiditySetpoint(float humidity);
  void emergencyStop();
  void resetSystem();
  
  void sendStatusToSerial();
  void sendTimeToSerial();
  void saveSettings();
  void loadSettings();
  void testWatchdog();
  
  // Getters for system state / Геттеры для состояния системы
  const SystemStatus& getSystemStatus() const {
    return systemStatus;
  }
  
  // Getters for settings / Геттеры для настроек
  PIDFactor getTemperaturePID() const {
    return temperature_pid.getFactors();
  }
  
  float getTemperatureSetpoint() const {
    return climate_setpoint.target_temperature;
  }
  
  float getHumiditySetpoint() const {
    return climate_setpoint.target_humidity;
  }
  
  bool isSystemEnabled() const {
    return systemEnabled;
  }
  
  void enableSystem() {
    systemEnabled = true;
  }
  
  void disableSystem() {
    systemEnabled = false;
    emergencyStop();
  }
  
  RTCManager& getRTC() {
    return rtc;
  }
  
  LightScheduler& getLightScheduler() {
    return light_scheduler;
  }
  
  SensorReader& getSensorReader() {
    return sensor_reader;
  }
  
  // NEW: Get watchdog reset count / НОВОЕ: Получить количество сбросов watchdog
  int getWatchdogResetCount() const {
    // Since we don't have direct access to watchdog from LCD,
    // we'll track it separately in errorCount for display purposes
    // Так как у нас нет прямого доступа к watchdog из LCD,
    // мы будем отслеживать его отдельно в errorCount для отображения
    return errorCount; // Using errorCount as a placeholder / Используем errorCount как заглушку
  }
  
  ~ClimateController() {
    if (lcd_display) delete lcd_display;
    if (encoder) delete encoder;
  }
};

#endif // CLIMATE_CONTROLLER_H