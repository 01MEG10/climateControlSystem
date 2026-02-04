/*
 * Serial Commander implementation
 * Реализация командного интерфейса Serial
 */

#include "serial_commander.h"
#include "rtc_manager.h"

void SerialCommander::begin(unsigned long baudRate = 9600) {
  Serial.begin(baudRate);
  while (!Serial) {
    ; // Wait for Serial to initialize / Ждем инициализации Serial
  }
  delay(100);
  
  printWelcomeMessage();
}

void SerialCommander::printWelcomeMessage() {
  Serial.println(F("========================================"));
  Serial.println(F("Climate Control System v3.0 with RTC"));
  Serial.println(F("Продвинутая система контроля климата v3.0 с RTC"));
  Serial.println(F("========================================"));
  Serial.println(F("Type 'help' for available commands"));
  Serial.println(F("Введите 'help' для списка команд"));
  Serial.println(F("========================================"));
}

void SerialCommander::printHelp() {
  Serial.println(F("\n=== Available Commands / Доступные команды ==="));
  Serial.println(F("  status      - Show system status / Показать статус системы"));
  Serial.println(F("  time        - Show current time / Показать текущее время"));
  Serial.println(F("  settime HH:MM:SS - Set time / Установить время"));
  Serial.println(F("  setdate DD/MM/YY - Set date / Установить дату"));
  Serial.println(F("  lighton HH:MM   - Set light ON time / Установить время включения света"));
  Serial.println(F("  lightoff HH:MM  - Set light OFF time / Установить время выключения света"));
  Serial.println(F("  lightschedule on/off - Enable/disable light schedule / Вкл/выкл расписание света"));
  Serial.println(F("  temp X      - Set temperature to X°C / Установить температуру X°C"));
  Serial.println(F("  humid X     - Set humidity to X% / Установить влажность X%"));
  Serial.println(F("  save        - Save settings to EEPROM / Сохранить настройки в EEPROM"));
  Serial.println(F("  load        - Load settings from EEPROM / Загрузить настройки из EEPROM"));
  Serial.println(F("  reset       - Reset system / Сбросить систему"));
  Serial.println(F("  stop        - Emergency stop / Аварийная остановка"));
  Serial.println(F("  echo on/off - Enable/disable command echo / Включить/выключить эхо команд"));
  Serial.println(F("  wdtest      - Test watchdog timer / Тест watchdog таймера"));
  Serial.println(F("  help        - This help message / Это сообщение помощи"));
}

bool SerialCommander::checkForCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
        lastCommandTime = millis();
        return true;
      }
    } else if (c == 8 || c == 127) { // Backspace / Удаление
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
        Serial.write(c);
      }
    } else if (c >= 32 && c <= 126) { // Printable characters / Печатаемые символы
      inputBuffer += c;
      if (echoEnabled) {
        Serial.write(c);
      }
    }
  }
  return false;
}

void SerialCommander::processCommand(String& cmd) {
  cmd.trim();
  if (echoEnabled) {
    Serial.println(); // New line after command / Новая строка после команды
  }
  
  if (cmd == "status") {
    onStatusRequest();
  } else if (cmd == "time") {
    onTimeRequest();
  } else if (cmd.startsWith("settime ")) {
    String timeStr = cmd.substring(8);
    onSetTime(timeStr);
  } else if (cmd.startsWith("setdate ")) {
    String dateStr = cmd.substring(8);
    onSetDate(dateStr);
  } else if (cmd.startsWith("lighton ")) {
    String timeStr = cmd.substring(8);
    onSetLightOnTime(timeStr);
  } else if (cmd.startsWith("lightoff ")) {
    String timeStr = cmd.substring(9);
    onSetLightOffTime(timeStr);
  } else if (cmd == "lightschedule on") {
    onSetLightSchedule(true);
  } else if (cmd == "lightschedule off") {
    onSetLightSchedule(false);
  } else if (cmd == "save") {
    onSaveRequest();
  } else if (cmd == "load") {
    onLoadRequest();
  } else if (cmd == "reset") {
    onResetRequest();
  } else if (cmd == "stop") {
    onEmergencyStop();
  } else if (cmd == "help") {
    printHelp();
  } else if (cmd == "echo on") {
    echoEnabled = true;
    Serial.println(F("Echo enabled / Эхо включено"));
  } else if (cmd == "echo off") {
    echoEnabled = false;
    Serial.println(F("Echo disabled / Эхо выключено"));
  } else if (cmd == "wdtest") {
    onWatchdogTest();
  } else if (cmd.startsWith("temp ")) {
    float temp = cmd.substring(5).toFloat();
    onTemperatureSet(temp);
  } else if (cmd.startsWith("humid ")) {
    float humid = cmd.substring(6).toFloat();
    onHumiditySet(humid);
  } else {
    Serial.print(F("Unknown command: / Неизвестная команда: "));
    Serial.println(cmd);
    Serial.println(F("Type 'help' for available commands / Введите 'help' для списка команд"));
  }
}

void SerialCommander::sendStatus(float temp, float targetTemp, float humid, float targetHumid, 
                                int light, bool heaterOn, bool humidifierOn, int errors,
                                bool lightOn, bool scheduleEnabled) {
  Serial.println(F("\n=== System Status / Статус системы ==="));
  Serial.print(F("Temperature / Температура: "));
  Serial.print(temp);
  Serial.print(F("°C / Target / Цель: "));
  Serial.print(targetTemp);
  Serial.println(F("°C"));
  
  Serial.print(F("Humidity / Влажность: "));
  Serial.print(humid);
  Serial.print(F("% / Target / Цель: "));
  Serial.print(targetHumid);
  Serial.print(F("% / Humidifier / Увлажнитель: "));
  Serial.println(humidifierOn ? F("ON / ВКЛ") : F("OFF / ВЫКЛ"));
  
  Serial.print(F("Light Level / Уровень света: "));
  Serial.print(light);
  Serial.print(F(" / Light / Свет: "));
  Serial.println(lightOn ? F("ON / ВКЛ") : F("OFF / ВЫКЛ"));
  
  Serial.print(F("Heater/Cooler / Обогреватель/Охладитель: "));
  Serial.println(heaterOn ? F("ACTIVE / АКТИВЕН") : F("INACTIVE / НЕАКТИВЕН"));
  
  Serial.print(F("Light Schedule / Расписание света: "));
  Serial.println(scheduleEnabled ? F("ENABLED / ВКЛЮЧЕНО") : F("DISABLED / ВЫКЛЮЧЕНО"));
  
  Serial.print(F("System Errors / Ошибки системы: "));
  Serial.println(errors);
  
  Serial.println(F("========================================"));
}

void SerialCommander::sendMessage(const String& msg) {
  Serial.println(msg);
}

void SerialCommander::sendTime(const DateTime& time) {
  Serial.print(F("Current time / Текущее время: "));
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d %02d/%02d/%02d", 
           time.hour, time.minute, time.second, time.day, time.month, time.year + 2000);
  Serial.println(buffer);
}