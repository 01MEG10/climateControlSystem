/*
 * Main climate controller implementation
 * Реализация главного контроллера климата
 */

#include "climate_controller.h"
#include "eeprom_manager.h"

// Create global instance / Создаем глобальный экземпляр
ClimateController ClimateController::climateController;

ClimateController::ClimateController() :
  temperature_pid(PIDFactor()),
  humidity_work_flag(false),
  overheatProtectionActive(false),
  systemEnabled(true),
  overheatStartTime(0),
  sensorReadTimer(SENSOR_READ_INTERVAL, true),
  humidifierMinWorkTimer(HUMIDIFIER_MINIMAL_WORK_TIME, true),
  humidifierMinPauseTimer(HUMIDIFIER_MIN_PAUSE_TIME, true),
  serialUpdateTimer(SERIAL_UPDATE_INTERVAL, true),
  statusDisplayTimer(30000UL, true), // Every 30 seconds / Каждые 30 секунд
  rtcUpdateTimer(RTC_UPDATE_INTERVAL, true),
  humidifierStartTime(0),
  errorCount(0),
  lastLightPWM(0),
  light_scheduler(&rtc, &climate_setpoint),
  lcd_display(nullptr),
  encoder(nullptr)
{  
  initializePins();
  setupSerialCallbacks();
}

void ClimateController::initializePins() {
  pinMode(PIN_HEATER_COOLER_POWER, OUTPUT);
  pinMode(PIN_HUMIDIFIER_POWER, OUTPUT);
  pinMode(PIN_LIGHT_POWER, OUTPUT);
  
  digitalWrite(PIN_HEATER_COOLER_POWER, LOW);
  digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
  digitalWrite(PIN_LIGHT_POWER, LOW);
  
  #if defined(ESP32)
    // PWM setup for ESP32 / Настройка ШИМ для ESP32
    ledcSetup(0, 5000, 8);
    ledcSetup(1, 5000, 8);
    ledcAttachPin(PIN_HEATER_COOLER_POWER, 0);
    ledcAttachPin(PIN_LIGHT_POWER, 1);
  #endif
  
  #ifdef DEBUG_MODE
    Serial.println(F("Pins initialized / Пины инициализированы"));
  #endif
}

void ClimateController::begin() {
  serial_cmd.begin(9600);
  InterruptHandler::getInstance().begin();
  watchdog.enable();
  rtc.begin();
  
  // Initialize LCD and encoder / Инициализация LCD и энкодера
  lcd_display = new LCDDisplay();
  encoder = new RotaryEncoder();
  lcd_display->begin();
  encoder->setLCDDisplay(lcd_display);
  
  // Load settings from EEPROM / Загрузка настроек из EEPROM
  loadSettings();
  
  #ifdef DEBUG_MODE
    Serial.println(F("Climate Controller initialized with LCD / Контроллер климата инициализирован с LCD"));
  #endif
}

void ClimateController::update() {
  // Update RTC time / Обновляем время RTC
  rtc.update();
  
  // Update light scheduler / Обновляем планировщик освещения
  light_scheduler.update();
  
  // Read sensors and control by timer / Чтение и управление по таймеру
  if (sensorReadTimer.isTime()) {
    sensor_reader.readAll();
    const SensorInfo& sensor_info = sensor_reader.getSensorInfo();
    
    if (sensor_info.measurement_ok && systemEnabled) {
      controlTemperature(sensor_info.temperature);
      controlHumidity(sensor_info.humidity);
      controlLight();
      errorCount = 0; // Reset error counter on successful measurements / Сброс счетчика ошибок при успешных измерениях
    } else {
      if (systemEnabled) {
        errorCount++;
        if (errorCount > 5) {
          emergencyStop();
        }
      }
    }
  }
  
  // Check serial commands / Проверка команд с Serial
  serial_cmd.checkForCommand();
  
  // Update LCD display / Обновление LCD дисплея
  if (lcd_display) {
    lcd_display->update();
  }
  
  // Update rotary encoder / Обновление энкодера
  if (encoder) {
    encoder->update();
  }
  
  // Periodically send status / Периодическая отправка статуса
  if (serialUpdateTimer.isTime() && systemEnabled) {
    sendStatusToSerial();
  }
  
  // Update system status / Обновление статуса системы
  updateSystemStatus();
  
  // System health monitoring / Мониторинг здоровья системы
  monitorSystemHealth();
}

void ClimateController::controlLight() {
  if (!systemEnabled || InterruptHandler::getInstance().isEmergencyActive()) {
    setLightPower(0);
    lastLightPWM = 0;
    return;
  }
  
  // Get light level from scheduler / Получаем уровень света из планировщика
  int scheduledLightLevel = light_scheduler.getLightLevel();
  
  // Also consider sensor reading for automatic adjustment / Также учитываем показания датчика для автоматической регулировки
  const SensorInfo& sensor = sensor_reader.getSensorInfo();
  int sensorLightLevel = sensor.light;
  
  if (sensorLightLevel != INVALID_INT && sensorLightLevel >= 0) {
    // Blend scheduled and sensor-based light levels / Смешиваем запланированный и основанный на датчике уровень света
    int adcMax = SensorReader::getADCMaxValue();
    int sensorBasedPWM = map(
      constrain(sensorLightLevel, 0, adcMax),
      0, adcMax,
      0, 255
    );
    
    // Use scheduled level as base, adjust with sensor / Используем запланированный уровень как базовый, корректируем датчиком
    int finalLightLevel = scheduledLightLevel;
    if (scheduledLightLevel > 0) {
      // When light should be on, adjust based on ambient light / Когда свет должен быть включен, корректируем на основе окружающего света
      finalLightLevel = constrain(scheduledLightLevel + (255 - sensorBasedPWM) / 4, 0, 255);
    }
    
    // Smooth brightness change / Плавное изменение яркости
    if (abs(finalLightLevel - lastLightPWM) > 10) {
      if (finalLightLevel > lastLightPWM) {
        finalLightLevel = lastLightPWM + 10;
      } else {
        finalLightLevel = lastLightPWM - 10;
      }
    }
    
    setLightPower(finalLightLevel);
    lastLightPWM = finalLightLevel;
  } else {
    // Use only scheduled light level if sensor fails / Используем только запланированный уровень света если датчик не работает
    setLightPower(scheduledLightLevel);
    lastLightPWM = scheduledLightLevel;
  }
}

void ClimateController::setLightPower(int pwmValue) {
  #if defined(ESP32)
    ledcWrite(1, pwmValue);
  #else
    analogWrite(PIN_LIGHT_POWER, pwmValue);
  #endif
}

void ClimateController::controlHumidity(float humidityLevel) {
  if (!systemEnabled || InterruptHandler::getInstance().isEmergencyActive()) {
    digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
    humidity_work_flag = false;
    return;
  }
  
  // Check data correctness / Проверка корректности данных
  if (isnan(humidityLevel) || isnan(climate_setpoint.target_humidity)) {
    digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
    humidity_work_flag = false;
    return;
  }
  
  // Safety check / Проверка безопасности
  if (humidityLevel < MIN_SAFE_HUMIDITY || humidityLevel > MAX_SAFE_HUMIDITY || 
      climate_setpoint.target_humidity < 20.0f || climate_setpoint.target_humidity > 80.0f) {
    digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
    humidity_work_flag = false;
    return;
  }
  
  // Protection against too frequent on/off switching / Защита от слишком частого включения/выключения
  if (humidity_work_flag) {
    // Check minimum working time / Проверяем минимальное время работы
    if (!humidifierMinWorkTimer.isTime()) {
      return;
    }
    
    float deficit = climate_setpoint.target_humidity - humidityLevel;
    
    if (deficit <= 1.0f) { // Hysteresis for turn off / Гистерезис для выключения
      digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
      humidity_work_flag = false;
      humidifierMinPauseTimer.reset();
      humidifierStartTime = 0;
      return;
    }
    
    // Check maximum working time / Проверка максимального времени работы
    if (humidifierStartTime > 0) {
      unsigned long currentTime = millis();
      unsigned long workTime;
      
      if (currentTime >= humidifierStartTime) {
        workTime = currentTime - humidifierStartTime;
      } else {
        workTime = (0xFFFFFFFFUL - humidifierStartTime) + currentTime;
      }
      
      if (workTime > HUMIDIFIER_MAXIMUM_WORK_TIME) {
        digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
        humidity_work_flag = false;
        humidifierMinPauseTimer.reset();
        humidifierStartTime = 0;
      }
    }
  } else {
    // Check minimum pause time / Проверяем минимальное время паузы
    if (!humidifierMinPauseTimer.isTime()) {
      return;
    }
    
    float deficit = climate_setpoint.target_humidity - humidityLevel;
    
    if (deficit > HUMIDIFIER_HYSTERESIS) {
      digitalWrite(PIN_HUMIDIFIER_POWER, HIGH);
      humidity_work_flag = true;
      humidifierMinWorkTimer.reset();
      humidifierStartTime = millis();
      
      #ifdef DEBUG_MODE
        Serial.print(F("Humidifier ON. Deficit: / Увлажнитель ВКЛ. Дефицит: "));
        Serial.print(deficit);
        Serial.print(F(" Target / Цель: "));
        Serial.print(climate_setpoint.target_humidity);
        Serial.print(F(" Current / Текущая: "));
        Serial.println(humidityLevel);
      #endif
    }
  }
}

void ClimateController::controlTemperature(float temperatureLevel) {
  if (!systemEnabled || InterruptHandler::getInstance().isEmergencyActive()) {
    setHeaterPower(0);
    return;
  }
  
  // Safety check / Проверка безопасности
  if (isnan(temperatureLevel) || isnan(climate_setpoint.target_temperature)) {
    setHeaterPower(0);
    return;
  }
  
  // Overheat protection / Защита от перегрева
  if (temperatureLevel > MAX_SAFE_TEMPERATURE) {
    if (!overheatProtectionActive) {
      overheatProtectionActive = true;
      overheatStartTime = millis();
      temperature_pid.reset();
    }
    
    setHeaterPower(0);
    
    if (millis() - overheatStartTime > 60000) {
      overheatProtectionActive = false;
    }
    return;
  } else {
    overheatProtectionActive = false;
  }
  
  // Protection from too low temperature / Защита от слишком низкой температуры
  if (temperatureLevel < MIN_SAFE_TEMPERATURE) {
    setHeaterPower(255);
    temperature_pid.resetIntegral();
    return;
  }
  
  // Normal PID operation / Нормальная работа PID
  double pidOutput = temperature_pid.compute(
    climate_setpoint.target_temperature,
    temperatureLevel
  );
  
  setHeaterPower((int)pidOutput);
}

void ClimateController::setHeaterPower(int pwmValue) {
  #if defined(ESP32)
    ledcWrite(0, pwmValue);
  #else
    analogWrite(PIN_HEATER_COOLER_POWER, pwmValue);
  #endif
}

void ClimateController::monitorSystemHealth() {
  const SensorInfo& sensor = sensor_reader.getSensorInfo();
  
  // Check for safety limits exceed / Проверка выхода за безопасные пределы
  if (sensor.temperature > MAX_SAFE_TEMPERATURE + 5) {
    emergencyStop();
    errorCount++;
    watchdog.incrementResetCount();
  }
  
  // Check for relay sticking / Проверка на залипание реле
  if (digitalRead(PIN_HUMIDIFIER_POWER) == HIGH && humidifierStartTime > 0) {
    unsigned long currentTime = millis();
    unsigned long workTime;
    
    if (currentTime >= humidifierStartTime) {
      workTime = currentTime - humidifierStartTime;
    } else {
      workTime = (0xFFFFFFFFUL - humidifierStartTime) + currentTime;
    }
    
    if (workTime > HUMIDIFIER_MAXIMUM_WORK_TIME * 2) {
      emergencyStop();
      errorCount++;
      watchdog.incrementResetCount();
    }
  }
  
  // Check interrupt emergency / Проверка аварийной ситуации по прерыванию
  if (InterruptHandler::getInstance().checkEmergency()) {
    if (InterruptHandler::getInstance().isEmergencyActive()) {
      emergencyStop();
      serial_cmd.sendMessage(F("EMERGENCY STOP ACTIVATED VIA BUTTON / АВАРИЙНАЯ ОСТАНОВКА ПО КНОПКЕ"));
    } else {
      systemEnabled = true;
      serial_cmd.sendMessage(F("System re-enabled / Система перезапущена"));
    }
  }
  
  // Check for too many errors / Проверка на слишком много ошибок
  if (errorCount > 10) {
    resetSystem();
  }
  
  // Feed watchdog / Кормление watchdog
  watchdog.feed();
}

void ClimateController::setupSerialCallbacks() {
  // Setup callback pointers / Настраиваем указатели на функции обратного вызова
  serial_cmd.onStatusRequest = &ClimateController::onStatusRequestStatic;
  serial_cmd.onTimeRequest = &ClimateController::onTimeRequestStatic;
  serial_cmd.onSetTime = &ClimateController::onSetTimeStatic;
  serial_cmd.onSetDate = &ClimateController::onSetDateStatic;
  serial_cmd.onSetLightOnTime = &ClimateController::onSetLightOnTimeStatic;
  serial_cmd.onSetLightOffTime = &ClimateController::onSetLightOffTimeStatic;
  serial_cmd.onSetLightSchedule = &ClimateController::onSetLightScheduleStatic;
  serial_cmd.onSaveRequest = &ClimateController::onSaveRequestStatic;
  serial_cmd.onLoadRequest = &ClimateController::onLoadRequestStatic;
  serial_cmd.onResetRequest = &ClimateController::onResetRequestStatic;
  serial_cmd.onEmergencyStop = &ClimateController::onEmergencyStopStatic;
  serial_cmd.onTemperatureSet = &ClimateController::onTemperatureSetStatic;
  serial_cmd.onHumiditySet = &ClimateController::onHumiditySetStatic;
  serial_cmd.onWatchdogTest = &ClimateController::onWatchdogTestStatic;
}

// Статические методы-обертки (уже были написаны ранее, но повторю для полноты)

void ClimateController::onStatusRequestStatic() {
  climateController.sendStatusToSerial();
}

void ClimateController::onTimeRequestStatic() {
  climateController.sendTimeToSerial();
}

void ClimateController::onSetTimeStatic(String timeStr) {
  climateController.setTimeFromString(timeStr);
}

void ClimateController::onSetDateStatic(String dateStr) {
  climateController.setDateFromString(dateStr);
}

void ClimateController::onSetLightOnTimeStatic(String timeStr) {
  climateController.setLightOnTimeFromString(timeStr);
}

void ClimateController::onSetLightOffTimeStatic(String timeStr) {
  climateController.setLightOffTimeFromString(timeStr);
}

void ClimateController::onSetLightScheduleStatic(bool enabled) {
  climateController.light_scheduler.enableSchedule(enabled);
  climateController.serial_cmd.sendMessage(enabled ? 
    F("Light schedule enabled / Расписание света включено") : 
    F("Light schedule disabled / Расписание света выключено"));
}

void ClimateController::onSaveRequestStatic() {
  climateController.saveSettings();
}

void ClimateController::onLoadRequestStatic() {
  climateController.loadSettings();
}

void ClimateController::onResetRequestStatic() {
  climateController.resetSystem();
  climateController.serial_cmd.sendMessage(F("System reset complete / Сброс системы завершен"));
}

void ClimateController::onEmergencyStopStatic() {
  climateController.emergencyStop();
  climateController.serial_cmd.sendMessage(F("Emergency stop via serial / Аварийная остановка по Serial"));
}

void ClimateController::onTemperatureSetStatic(float temp) {
  climateController.setTemperatureSetpoint(temp);
  climateController.serial_cmd.sendMessage("Temperature set to / Температура установлена на: " + String(temp) + "C");
}

void ClimateController::onHumiditySetStatic(float humid) {
  climateController.setHumiditySetpoint(humid);
  climateController.serial_cmd.sendMessage("Humidity set to / Влажность установлена на: " + String(humid) + "%");
}

void ClimateController::onWatchdogTestStatic() {
  climateController.testWatchdog();
}

void ClimateController::setTimeFromString(const String& timeStr) {
  // Format: HH:MM:SS / Формат: ЧЧ:ММ:СС
  int hour = timeStr.substring(0, 2).toInt();
  int minute = timeStr.substring(3, 5).toInt();
  int second = timeStr.substring(6, 8).toInt();
  
  if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60 && second >= 0 && second < 60) {
    rtc.setTime(hour, minute, second);
    serial_cmd.sendMessage("Time set to / Время установлено: " + timeStr);
  } else {
    serial_cmd.sendMessage(F("Invalid time format. Use HH:MM:SS / Неверный формат времени. Используйте ЧЧ:ММ:СС"));
  }
}

void ClimateController::setDateFromString(const String& dateStr) {
  // Format: DD/MM/YY / Формат: ДД/ММ/ГГ
  int day = dateStr.substring(0, 2).toInt();
  int month = dateStr.substring(3, 5).toInt();
  int year = dateStr.substring(6, 8).toInt();
  
  if (day >= 1 && day <= 31 && month >= 1 && month <= 12 && year >= 0 && year <= 99) {
    rtc.setDate(year, month, day);
    serial_cmd.sendMessage("Date set to / Дата установлена: " + dateStr);
  } else {
    serial_cmd.sendMessage(F("Invalid date format. Use DD/MM/YY / Неверный формат даты. Используйте ДД/ММ/ГГ"));
  }
}

void ClimateController::setLightOnTimeFromString(const String& timeStr) {
  // Format: HH:MM / Формат: ЧЧ:ММ
  int hour = timeStr.substring(0, 2).toInt();
  int minute = timeStr.substring(3, 5).toInt();
  
  if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60) {
    light_scheduler.setLightOnTime(hour, minute);
    serial_cmd.sendMessage("Light ON time set to / Время включения света установлено: " + timeStr);
  } else {
    serial_cmd.sendMessage(F("Invalid time format. Use HH:MM / Неверный формат времени. Используйте ЧЧ:ММ"));
  }
}

void ClimateController::setLightOffTimeFromString(const String& timeStr) {
  // Format: HH:MM / Формат: ЧЧ:ММ
  int hour = timeStr.substring(0, 2).toInt();
  int minute = timeStr.substring(3, 5).toInt();
  
  if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60) {
    light_scheduler.setLightOffTime(hour, minute);
    serial_cmd.sendMessage("Light OFF time set to / Время выключения света установлено: " + timeStr);
  } else {
    serial_cmd.sendMessage(F("Invalid time format. Use HH:MM / Неверный формат времени. Используйте ЧЧ:ММ"));
  }
}

void ClimateController::updateSystemStatus() {
  const SensorInfo& sensor = sensor_reader.getSensorInfo();
  
  systemStatus.currentTemperature = sensor.temperature;
  systemStatus.targetTemperature = climate_setpoint.target_temperature;
  systemStatus.currentHumidity = sensor.humidity;
  systemStatus.targetHumidity = climate_setpoint.target_humidity;
  systemStatus.pidOutput = temperature_pid.getLastOutput();
  systemStatus.overheatProtection = overheatProtectionActive;
  systemStatus.humidityWorking = humidity_work_flag;
  systemStatus.lightLevel = sensor.light;
  systemStatus.errorCount = errorCount;
  systemStatus.systemEnabled = systemEnabled;
  systemStatus.lightOn = light_scheduler.isLightOn();
  systemStatus.scheduleEnabled = light_scheduler.isScheduleEnabled();
  systemStatus.currentTime = rtc.getCurrentTime();
}

void ClimateController::setTemperaturePID(const PIDFactor& factors) {
  PIDFactor safeFactors = factors;
  safeFactors.Kp = constrain(factors.Kp, 0.0, 100.0);
  safeFactors.Ki = constrain(factors.Ki, 0.0, 10.0);
  safeFactors.Kd = constrain(factors.Kd, 0.0, 5.0);
  safeFactors.outputMin = constrain(factors.outputMin, 0, 255);
  safeFactors.outputMax = constrain(factors.outputMax, 0, 255);
  
  temperature_pid.setFactors(safeFactors);
}

void ClimateController::setTemperatureSetpoint(float temperature) {
  climate_setpoint.target_temperature = constrain(
    temperature, 
    TEMPERATURE_SETPOINT_MIN, 
    TEMPERATURE_SETPOINT_MAX
  );
  temperature_pid.reset();
}

void ClimateController::setHumiditySetpoint(float humidity) {
  climate_setpoint.target_humidity = constrain(
    humidity,
    20.0f,
    80.0f
  );
}

void ClimateController::emergencyStop() {
  setHeaterPower(0);
  setLightPower(0);
  digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
  
  temperature_pid.reset();
  humidity_work_flag = false;
  overheatProtectionActive = false;
  systemEnabled = false;
  
  serial_cmd.sendMessage(F("!!! EMERGENCY STOP ACTIVATED !!! / !!! АВАРИЙНАЯ ОСТАНОВКА АКТИВИРОВАНА !!!"));
}

void ClimateController::resetSystem() {
  emergencyStop();
  
  sensorReadTimer.reset();
  humidifierMinWorkTimer.reset();
  humidifierMinPauseTimer.reset();
  serialUpdateTimer.reset();
  statusDisplayTimer.reset();
  rtcUpdateTimer.reset();
  
  temperature_pid.reset();
  humidity_work_flag = false;
  overheatProtectionActive = false;
  humidifierStartTime = 0;
  errorCount = 0;
  lastLightPWM = 0;
  InterruptHandler::getInstance().resetEmergency();
  
  // Wait and then re-enable / Ждем и затем включаем снова
  delay(2000);
  systemEnabled = true;
  
  serial_cmd.sendMessage(F("System reset and re-enabled / Система сброшена и перезапущена"));
}

void ClimateController::sendStatusToSerial() {
  const SensorInfo& sensor = sensor_reader.getSensorInfo();
  int pidOutput = temperature_pid.getLastOutput();
  uint8_t lightOnHour, lightOnMinute, lightOffHour, lightOffMinute;
  light_scheduler.getLightOnTime(lightOnHour, lightOnMinute);
  light_scheduler.getLightOffTime(lightOffHour, lightOffMinute);
  
  serial_cmd.sendStatus(
    sensor.temperature,
    climate_setpoint.target_temperature,
    sensor.humidity,
    climate_setpoint.target_humidity,
    sensor.light,
    pidOutput > 0,
    humidity_work_flag,
    errorCount,
    light_scheduler.isLightOn(),
    light_scheduler.isScheduleEnabled()
  );
  
  // Display light schedule / Отображаем расписание освещения
  Serial.print(F("Light Schedule / Расписание света: "));
  Serial.print(lightOnHour);
  Serial.print(F(":"));
  Serial.print(lightOnMinute < 10 ? "0" : "");
  Serial.print(lightOnMinute);
  Serial.print(F(" - "));
  Serial.print(lightOffHour);
  Serial.print(F(":"));
  Serial.print(lightOffMinute < 10 ? "0" : "");
  Serial.println(lightOffMinute);
  
  // Display current time / Отображаем текущее время
  Serial.print(F("Current RTC time / Текущее время RTC: "));
  rtc.printCurrentTime();
  Serial.println();
  
  // Additional info for debugging / Дополнительная информация для отладки
  #ifdef DEBUG_MODE
    Serial.print(F("PID Output / Выход PID: "));
    Serial.println(pidOutput);
    Serial.print(F("Watchdog resets / Сбросы Watchdog: "));
    Serial.println(watchdog.getResetCount());
    Serial.print(F("Sensor errors / Ошибки датчиков: "));
    Serial.println(sensor_reader.getErrorCount());
    Serial.print(F("RTC available / RTC доступен: "));
    Serial.println(rtc.isRTCAvailable() ? "YES / ДА" : "NO / НЕТ");
  #endif
}

void ClimateController::sendTimeToSerial() {
  serial_cmd.sendTime(rtc.getCurrentTime());
}

void ClimateController::saveSettings() {
  // Implementation for EEPROM saving / Реализация сохранения в EEPROM
  saveSettingsToEEPROM(*this);
  serial_cmd.sendMessage(F("Settings saved to EEPROM / Настройки сохранены в EEPROM"));
}

void ClimateController::loadSettings() {
  // Implementation for EEPROM loading / Реализация загрузки из EEPROM
  if (loadSettingsFromEEPROM(*this)) {
    serial_cmd.sendMessage(F("Settings loaded from EEPROM / Настройки загружены из EEPROM"));
  } else {
    serial_cmd.sendMessage(F("Using default settings / Использую настройки по умолчанию"));
  }
}

void ClimateController::testWatchdog() {
  serial_cmd.sendMessage(F("Watchdog test - system will reset in 3 seconds... / Тест Watchdog - система перезагрузится через 3 секунды..."));
  delay(3000);
  
  // Disable watchdog and enter infinite loop / Отключаем watchdog и входим в бесконечный цикл
  watchdog.disable();
  while(true) {
    // This should trigger hardware reset / Это должно вызвать аппаратный сброс
  }
}