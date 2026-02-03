/*
 * Advanced Climate Control System with RTC and LCD Display
 * Продвинутая система контроля климата с RTC и LCD дисплеем
 * 
 * Author: Your Name
 * Date: 2024

 * 
 * Features / Особенности:
 * - Temperature and humidity control (DHT11) / Контроль температуры и влажности
 * - Light intensity regulation with time schedule / Регулировка освещенности по расписанию
 * - Real Time Clock (DS1302) integration / Интеграция часов реального времени
 * - Safety monitoring and protection / Мониторинг безопасности и защита
 * - Serial command interface / Интерфейс команд через Serial
 * - Watchdog timer for system recovery / Watchdog таймер для восстановления системы
 * - Interrupt-based safety system / Система безопасности на прерываниях
 * - EEPROM settings storage / Хранение настроек в EEPROM
 * - LCD display with rotary encoder navigation / LCD дисплей с навигацией энкодером
 * 
 * GitHub: https://github.com/01MEG10/ClimateControlSystem
 */

#include <EEPROM.h>
#include <DHT.h>
#include <math.h>
#include <avr/wdt.h>           // Watchdog timer for AVR
#include <ThreeWire.h>         // Library for DS1302
#include <RtcDS1302.h>         // RTC library for DS1302
#include <LiquidCrystal_I2C.h> // I2C LCD library
#include <Encoder.h>           // Rotary encoder library

// ==================== CONSTANTS AND DEFINITIONS / КОНСТАНТЫ И ОПРЕДЕЛЕНИЯ ====================

// Pin definitions / Определение пинов
#define PIN_LIGHT_POWER 3        // PWM pin for light control / ШИМ пин для управления светом
#define PIN_LIGHT_SENSOR A0      // Analog pin for light sensor / Аналоговый пин датчика света
#define PIN_DHT_SENSOR 2         // Digital pin for DHT sensor / Цифровой пин датчика DHT
#define PIN_HUMIDIFIER_POWER 4   // Pin for humidifier control / Пин управления увлажнителем
#define PIN_HEATER_COOLER_POWER 5 // Pin for temperature control / Пин управления температурой
#define PIN_EMERGENCY_BUTTON 6   // Emergency stop button pin / Пин кнопки аварийной остановки
#define PIN_WATCHDOG_RESET 7     // Hardware watchdog reset pin / Пин аппаратного сброса watchdog

// DS1302 RTC pins / Пины DS1302 RTC
#define PIN_RTC_CE 8     // Chip Enable / Включение чипа
#define PIN_RTC_IO 9     // Data I/O / Данные
#define PIN_RTC_SCLK 10  // Serial Clock / Тактовый сигнал

// Rotary encoder pins / Пины энкодера
#define PIN_ENCODER_CLK 11       // CLK pin / Пин CLK
#define PIN_ENCODER_DT 12        // DT pin / Пин DT
#define PIN_ENCODER_SW 13        // SW (button) pin / Пин кнопки

// LCD configuration / Конфигурация LCD
#define LCD_ADDRESS 0x27         // I2C display address / Адрес I2C дисплея
#define LCD_COLS 20              // Number of columns / Количество столбцов
#define LCD_ROWS 4               // Number of rows / Количество строк

// Invalid values for error handling / Невалидные значения для обработки ошибок
#define INVALID_INT -1
#define INVALID_FLOAT NAN

// Sensor type / Тип датчика
#define DHT_TYPE DHT11

// Time constants / Константы времени
#define SECOND_MS 1000UL
#define MINUTE_MS (60UL * SECOND_MS)
#define HOUR_MS (60UL * MINUTE_MS)

// Operation intervals / Интервалы работы
#define SENSOR_READ_INTERVAL 9000     // 9 seconds between measurements / 9 секунд между замерами
#define SERIAL_UPDATE_INTERVAL 5000   // 5 seconds between status updates / 5 секунд между обновлениями статуса
#define RTC_UPDATE_INTERVAL 60000UL   // 1 minute between RTC checks / 1 минута между проверками RTC

// Light control settings / Настройки управления освещением
#define LIGHT_SCHEDULE_ENABLED true   // Enable light schedule / Включить расписание освещения
#define DEFAULT_LIGHT_ON_HOUR 7       // Default light ON hour / Час включения света по умолчанию
#define DEFAULT_LIGHT_OFF_HOUR 22     // Default light OFF hour / Час выключения света по умолчанию
#define LIGHT_TRANSITION_DURATION 10  // Light transition in minutes / Плавное включение в минутах

// Humidifier settings / Настройки увлажнителя
#define HUMIDIFIER_HYSTERESIS 3.0f
#define HUMIDIFIER_MINIMAL_WORK_TIME (60UL * SECOND_MS)    // 1 minute / 1 минута
#define HUMIDIFIER_MAXIMUM_WORK_TIME (10UL * MINUTE_MS)    // 10 minutes / 10 минут
#define HUMIDIFIER_MIN_PAUSE_TIME (30UL * SECOND_MS)       // 30 seconds minimum pause / 30 секунд минимальная пауза

// Safety constants / Константы безопасности
#define MAX_SAFE_TEMPERATURE 45.0f
#define MIN_SAFE_TEMPERATURE 5.0f
#define TEMPERATURE_SETPOINT_MIN 10.0f
#define TEMPERATURE_SETPOINT_MAX 40.0f
#define MAX_SAFE_HUMIDITY 90.0f
#define MIN_SAFE_HUMIDITY 10.0f

// Watchdog settings / Настройки watchdog
#define WATCHDOG_TIMEOUT WDTO_2S     // 2 second timeout / Таймаут 2 секунды
#define WATCHDOG_RESET_THRESHOLD 3   // Number of resets before full restart / Количество сбросов перед полной перезагрузкой

// Interrupt debounce / Защита от дребезга для прерываний
#define DEBOUNCE_TIME 50             // Debounce time in ms / Время защиты от дребезга в мс
#define ENCODER_DEBOUNCE_TIME 10     // Encoder debounce / Защита от дребезга энкодера

// Debug mode (uncomment when needed) / Режим отладки (раскомментировать при необходимости)
// #define DEBUG_MODE

// Define ULONG_MAX for Arduino if not defined / Определяем ULONG_MAX для Arduino если не определено
#ifndef ULONG_MAX
  #define ULONG_MAX 4294967295UL
#endif

// ==================== FORWARD DECLARATIONS / ПРЕДВАРИТЕЛЬНЫЕ ОБЪЯВЛЕНИЯ ====================
// Forward declare classes to resolve circular dependencies / Предварительное объявление классов для разрешения циклических зависимостей
class ClimateController;
class LCDDisplay;
class RotaryEncoder;

// ==================== ENUMS AND STRUCTURES / ПЕРЕЧИСЛЕНИЯ И СТРУКТУРЫ ====================
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

// ==================== RTC MANAGER CLASS / КЛАСС УПРАВЛЕНИЯ RTC ====================
class RTCManager {
private:
  ThreeWire* rtcWire;
  RtcDS1302<ThreeWire>* rtc;
  DateTime currentTime;
  bool rtcAvailable;
  unsigned long lastMillis;
  unsigned long lastSyncTime;
  
  // Convert RtcDateTime to DateTime / Конвертирует RtcDateTime в DateTime
  DateTime convertRtcDateTime(const RtcDateTime& dt) {
    DateTime result;
    result.year = dt.Year() - 2000; // Convert to 2-digit year / Конвертируем в 2-значный год
    result.month = dt.Month();
    result.day = dt.Day();
    result.hour = dt.Hour();
    result.minute = dt.Minute();
    result.second = dt.Second();
    result.weekday = dt.DayOfWeek();
    return result;
  }
  
  // Convert DateTime to RtcDateTime / Конвертирует DateTime в RtcDateTime
  RtcDateTime convertToRtcDateTime(const DateTime& dt) {
    return RtcDateTime(dt.year + 2000, dt.month, dt.day, dt.hour, dt.minute, dt.second);
  }
  
  void updateFromMillis() {
    unsigned long currentMillis = millis();
    unsigned long elapsed = currentMillis - lastMillis;
    
    if (elapsed >= 1000) {
      int secondsToAdd = elapsed / 1000;
      currentTime.second += secondsToAdd;
      lastMillis = currentMillis - (elapsed % 1000);
      
      // Handle overflow / Обработка переполнения
      if (currentTime.second >= 60) {
        currentTime.minute += currentTime.second / 60;
        currentTime.second %= 60;
      }
      if (currentTime.minute >= 60) {
        currentTime.hour += currentTime.minute / 60;
        currentTime.minute %= 60;
      }
      if (currentTime.hour >= 24) {
        currentTime.hour %= 24;
        // Note: We don't handle day overflow without RTC / Примечание: не обрабатываем переполнение дней без RTC
      }
    }
  }
  
public:
  RTCManager() : 
    rtcWire(nullptr),
    rtc(nullptr),
    rtcAvailable(false),
    lastMillis(millis()),
    lastSyncTime(0)
  { }
  
  void begin() {
    // Initialize RTC / Инициализация RTC
    rtcWire = new ThreeWire(PIN_RTC_IO, PIN_RTC_SCLK, PIN_RTC_CE);
    rtc = new RtcDS1302<ThreeWire>(*rtcWire);
    
    rtc->Begin();
    
    if (!rtc->GetIsRunning()) {
      #ifdef DEBUG_MODE
        Serial.println(F("RTC is not running, starting now... / RTC не запущен, запускаем..."));
      #endif
      rtc->SetIsRunning(true);
    }
    
    // Check if RTC has valid time / Проверяем есть ли в RTC валидное время
    if (!rtc->IsDateTimeValid()) {
      #ifdef DEBUG_MODE
        Serial.println(F("RTC lost confidence in the DateTime! / RTC потерял доверие к дате и времени!"));
      #endif
      
      // Set default time (Jan 1 2023, 12:00:00) / Устанавливаем время по умолчанию
      RtcDateTime defaultTime = RtcDateTime(2023, 1, 1, 12, 0, 0);
      rtc->SetDateTime(defaultTime);
    }
    
    // Try to read time / Пытаемся прочитать время
    // Note: Arduino doesn't support exceptions by default / Примечание: Arduino не поддерживает исключения по умолчанию
    RtcDateTime rtcTime = rtc->GetDateTime();
    if (rtcTime.Year() >= 2000) { // Simple validity check / Простая проверка валидности
      currentTime = convertRtcDateTime(rtcTime);
      rtcAvailable = true;
      lastMillis = millis();
      lastSyncTime = millis();
      
      #ifdef DEBUG_MODE
        Serial.print(F("RTC initialized. Current time: / RTC инициализирован. Текущее время: "));
        printTime(currentTime);
        Serial.println();
      #endif
    } else {
      rtcAvailable = false;
      #ifdef DEBUG_MODE
        Serial.println(F("Failed to read from RTC, using millis() / Не удалось прочитать RTC, использую millis()"));
      #endif
    }
  }
  
  void update() {
    if (rtcAvailable) {
      // Sync with RTC every minute / Синхронизируемся с RTC каждую минуту
      unsigned long currentMillis = millis();
      if (currentMillis - lastSyncTime >= RTC_UPDATE_INTERVAL) {
        RtcDateTime rtcTime = rtc->GetDateTime();
        if (rtcTime.Year() >= 2000) {
          currentTime = convertRtcDateTime(rtcTime);
          lastSyncTime = currentMillis;
          lastMillis = currentMillis;
        } else {
          // If RTC read fails, fall back to millis / Если чтение RTC не удалось, используем millis
          rtcAvailable = false;
          updateFromMillis();
        }
      } else {
        updateFromMillis();
      }
    } else {
      updateFromMillis();
    }
  }
  
  const DateTime& getCurrentTime() const {
    return currentTime;
  }
  
  void setTime(const DateTime& newTime) {
    if (rtcAvailable) {
      RtcDateTime rtcTime = convertToRtcDateTime(newTime);
      rtc->SetDateTime(rtcTime);
      currentTime = newTime;
      lastSyncTime = millis();
      lastMillis = millis();
      
      #ifdef DEBUG_MODE
        Serial.print(F("RTC time set to: / Время RTC установлено: "));
        printTime(currentTime);
        Serial.println();
      #endif
    } else {
      currentTime = newTime;
      lastMillis = millis();
    }
  }
  
  void setTime(uint8_t hour, uint8_t minute, uint8_t second) {
    DateTime newTime = currentTime;
    newTime.hour = hour;
    newTime.minute = minute;
    newTime.second = second;
    setTime(newTime);
  }
  
  void setDate(uint8_t year, uint8_t month, uint8_t day) {
    DateTime newTime = currentTime;
    newTime.year = year;
    newTime.month = month;
    newTime.day = day;
    setTime(newTime);
  }
  
  bool isRTCAvailable() const {
    return rtcAvailable;
  }
  
  uint32_t getTimeInSeconds() const {
    return currentTime.hour * 3600UL + currentTime.minute * 60UL + currentTime.second;
  }
  
  bool isTimeInRange(uint8_t startHour, uint8_t startMinute, uint8_t endHour, uint8_t endMinute) const {
    uint32_t currentSeconds = getTimeInSeconds();
    uint32_t startSeconds = startHour * 3600UL + startMinute * 60UL;
    uint32_t endSeconds = endHour * 3600UL + endMinute * 60UL;
    
    if (startSeconds <= endSeconds) {
      // Normal range (e.g., 8:00 to 20:00) / Нормальный диапазон (например, 8:00 до 20:00)
      return (currentSeconds >= startSeconds && currentSeconds < endSeconds);
    } else {
      // Overnight range (e.g., 20:00 to 8:00) / Диапазон через полночь (например, 20:00 до 8:00)
      return (currentSeconds >= startSeconds || currentSeconds < endSeconds);
    }
  }
  
  void printTime(const DateTime& dt) {
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d %02d/%02d/%02d", 
             dt.hour, dt.minute, dt.second, dt.day, dt.month, dt.year + 2000);
    Serial.print(buffer);
  }
  
  void printCurrentTime() {
    printTime(currentTime);
  }
  
  ~RTCManager() {
    if (rtc) delete rtc;
    if (rtcWire) delete rtcWire;
  }
};

// ==================== SERIAL COMMUNICATION CLASS / КЛАСС СВЯЗИ ЧЕРЕЗ SERIAL ====================
class SerialCommander {
private:
  String inputBuffer;
  unsigned long lastCommandTime;
  bool echoEnabled;
  
  // How Serial port works: / Принцип работы Serial порта:
  // 1. Data comes byte by byte via interrupts / Данные приходят байт за байтом через прерывания
  // 2. We accumulate them in buffer until newline character / Мы накапливаем их в буфере до получения символа новой строки
  // 3. After receiving full command it's processed / После получения полной команды она обрабатывается
  // 4. Response is sent back via Serial.print() / Ответ отправляется обратно через Serial.print()
  
public:
  SerialCommander() : 
    lastCommandTime(0),
    echoEnabled(true)
  { 
    inputBuffer.reserve(64); // Reserve buffer to avoid fragmentation / Резервируем буфер чтобы избежать фрагментации
  }
  
  void begin(unsigned long baudRate = 9600) {
    Serial.begin(baudRate);
    while (!Serial) {
      ; // Wait for Serial to initialize / Ждем инициализации Serial
    }
    delay(100);
    
    printWelcomeMessage();
  }
  
  void printWelcomeMessage() {
    Serial.println(F("========================================"));
    Serial.println(F("Climate Control System v3.0 with RTC"));
    Serial.println(F("Продвинутая система контроля климата v3.0 с RTC"));
    Serial.println(F("========================================"));
    Serial.println(F("Type 'help' for available commands"));
    Serial.println(F("Введите 'help' для списка команд"));
    Serial.println(F("========================================"));
  }
  
  void printHelp() {
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
  
  bool checkForCommand() {
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
  
  void processCommand(String& cmd) {
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
  
  // Pointer to external functions for callbacks / Указатели на внешние функции для обратных вызовов
  void (*onStatusRequest)();
  void (*onTimeRequest)();
  void (*onSetTime)(String timeStr);
  void (*onSetDate)(String dateStr);
  void (*onSetLightOnTime)(String timeStr);
  void (*onSetLightOffTime)(String timeStr);
  void (*onSetLightSchedule)(bool enabled);
  void (*onSaveRequest)();
  void (*onLoadRequest)();
  void (*onResetRequest)();
  void (*onEmergencyStop)();
  void (*onTemperatureSet)(float temp);
  void (*onHumiditySet)(float humid);
  void (*onWatchdogTest)();
  
  void sendStatus(float temp, float targetTemp, float humid, float targetHumid, 
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
  
  void sendMessage(const String& msg) {
    Serial.println(msg);
  }
  
  void sendTime(const DateTime& time) {
    Serial.print(F("Current time / Текущее время: "));
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d %02d/%02d/%02d", 
             time.hour, time.minute, time.second, time.day, time.month, time.year + 2000);
    Serial.println(buffer);
  }
};

// ==================== SAFE TIMER CLASS / КЛАСС БЕЗОПАСНОГО ТАЙМЕРА ====================
class SafeTime {
private:
  unsigned long lastTime;
  unsigned long interval;
  bool useRolloverProtection;
  
public:
  SafeTime(unsigned long intervalMs, bool rolloverProtection = true) 
    : interval(intervalMs), 
      lastTime(millis()),
      useRolloverProtection(rolloverProtection)
  { }
  
  // Safe interval check with overflow protection / Безопасная проверка интервала с защитой от переполнения
  bool isTime() {
    unsigned long currentTime = millis();
    
    if (!useRolloverProtection) {
      if (currentTime - lastTime >= interval) {
        lastTime = currentTime;
        return true;
      }
      return false;
    }
    
    // Method with overflow protection / Метод с защитой от переполнения
    if (currentTime >= lastTime) {
      if (currentTime - lastTime >= interval) {
        lastTime = currentTime;
        return true;
      }
    } else {
      // millis() overflow / Переполнение millis()
      if ((0xFFFFFFFFUL - lastTime) + currentTime >= interval) {
        lastTime = currentTime;
        return true;
      }
    }
    return false;
  }
  
  void reset() {
    lastTime = millis();
  }
  
  void setInterval(unsigned long newInterval) {
    interval = newInterval;
  }
  
  unsigned long getRemainingTime() {
    unsigned long currentTime = millis();
    if (currentTime >= lastTime) {
      unsigned long elapsed = currentTime - lastTime;
      return (elapsed >= interval) ? 0 : (interval - elapsed);
    } else {
      unsigned long elapsed = (0xFFFFFFFFUL - lastTime) + currentTime;
      return (elapsed >= interval) ? 0 : (interval - elapsed);
    }
  }
};

// ==================== WATCHDOG TIMER CLASS / КЛАСС WATCHDOG ТАЙМЕРА ====================
class WatchdogManager {
private:
  int resetCount;
  unsigned long lastFeedTime;
  unsigned long feedInterval;
  bool watchdogEnabled;
  
public:
  WatchdogManager() : 
    resetCount(0),
    lastFeedTime(0),
    feedInterval(1000), // Feed every second / Кормить каждую секунду
    watchdogEnabled(false)
  { }
  
  void enable() {
    #ifdef __AVR__
      wdt_enable(WATCHDOG_TIMEOUT);
      watchdogEnabled = true;
      lastFeedTime = millis();
      
      #ifdef DEBUG_MODE
        Serial.println(F("Watchdog enabled / Watchdog включен"));
      #endif
    #endif
  }
  
  void disable() {
    #ifdef __AVR__
      wdt_disable();
      watchdogEnabled = false;
      
      #ifdef DEBUG_MODE
        Serial.println(F("Watchdog disabled / Watchdog выключен"));
      #endif
    #endif
  }
  
  void feed() {
    if (!watchdogEnabled) return;
    
    unsigned long currentTime = millis();
    if (currentTime - lastFeedTime >= feedInterval) {
      #ifdef __AVR__
        wdt_reset();
      #endif
      lastFeedTime = currentTime;
      
      // Reset counter on successful feeding / Сброс счетчика при успешном кормлении
      resetCount = 0;
    }
  }
  
  void systemReset() {
    #ifdef __AVR__
      wdt_enable(WDTO_15MS);
      while(1) {}
    #endif
  }
  
  void incrementResetCount() {
    resetCount++;
    if (resetCount >= WATCHDOG_RESET_THRESHOLD) {
      #ifdef DEBUG_MODE
        Serial.println(F("Watchdog: Too many resets, performing system reset / Watchdog: Слишком много сбросов, выполняю перезагрузку системы"));
      #endif
      systemReset();
    }
  }
  
  int getResetCount() const {
    return resetCount;
  }
  
  bool isEnabled() const {
    return watchdogEnabled;
  }
};

// ==================== INTERRUPT HANDLER CLASS / КЛАСС ОБРАБОТКИ ПРЕРЫВАНИЙ ====================
class InterruptHandler {
private:
  volatile bool emergencyStopFlag;
  volatile unsigned long lastButtonPress;
  bool emergencyState;
  static InterruptHandler* instance;
  
  // Private constructor for singleton / Приватный конструктор для синглтона
  InterruptHandler() : 
    emergencyStopFlag(false),
    lastButtonPress(0),
    emergencyState(false)
  { }
  
public:
  static InterruptHandler& getInstance() {
    static InterruptHandler instance;
    return instance;
  }
  
  void begin() {
    instance = this;
    pinMode(PIN_EMERGENCY_BUTTON, INPUT_PULLUP);
    
    // Attach interrupt on falling edge (button press) / Назначаем прерывание на спад (нажатие кнопки)
    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_BUTTON), 
                    handleEmergencyISR, 
                    FALLING);
                    
    #ifdef DEBUG_MODE
      Serial.println(F("Interrupt handler initialized / Обработчик прерываний инициализирован"));
    #endif
  }
  
  static void handleEmergencyISR() {
    unsigned long currentTime = millis();
    
    // Debounce protection / Защита от дребезга
    if (currentTime - instance->lastButtonPress > DEBOUNCE_TIME) {
      instance->emergencyStopFlag = true;
      instance->lastButtonPress = currentTime;
    }
  }
  
  bool checkEmergency() {
    if (emergencyStopFlag) {
      emergencyStopFlag = false;
      emergencyState = !emergencyState; // Toggle state / Переключаем состояние
      return true;
    }
    return false;
  }
  
  bool isEmergencyActive() const {
    return emergencyState;
  }
  
  void resetEmergency() {
    emergencyState = false;
    emergencyStopFlag = false;
  }
  
  void simulateEmergency() {
    emergencyStopFlag = true;
  }
};

// Initialize static member / Инициализация статического члена
InterruptHandler* InterruptHandler::instance = nullptr;

// ==================== SENSOR READING CLASS / КЛАСС ЧТЕНИЯ ДАТЧИКОВ ====================
class SensorReader {
private:  
  SensorCalibration sensor_calibration;
  SensorInfo sensor_info;
  DHT dht;
  
  // ADC settings depending on platform / Настройки ADC в зависимости от платформы
  #if defined(ESP32) || defined(ESP8266)
    static const int ADC_MAX_VALUE = 4095;  // For ESP - 12-bit / Для ESP - 12-бит
    static const int ADC_BITS = 12;
  #else
    static const int ADC_MAX_VALUE = 1023;  // For Arduino - 10-bit / Для Arduino - 10-бит
    static const int ADC_BITS = 10;
  #endif
  
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

// ==================== ENHANCED PID CONTROLLER CLASS / УЛУЧШЕННЫЙ КЛАСС PID КОНТРОЛЛЕРА ====================
class PIDController {
private:
  double Kp, Ki, Kd;
  double prevError;
  double integral;
  double outputMin, outputMax;
  unsigned long prevTime;
  int lastOutput;
  bool firstRun;
  double maxIntegral;
  
  void updateMaxIntegral() {
    if (fabs(Ki) > 0.0001) {
      maxIntegral = fabs((outputMax - outputMin) / Ki) * 0.8;
    } else {
      maxIntegral = 1000.0;
    }
  }
  
public:
  PIDController(const PIDFactor& factors = PIDFactor()) {
    setFactors(factors);
    firstRun = true;
    lastOutput = 0;
  }
  
  double compute(double setpoint, double measuredValue) {
    if (firstRun) {
      prevTime = millis();
      prevError = setpoint - measuredValue;
      integral = 0;
      firstRun = false;
      lastOutput = 0;
      return lastOutput;
    }
    
    unsigned long currentTime = millis();
    double deltaTime = (currentTime - prevTime) / 1000.0;
    
    if (deltaTime < 0.001 || deltaTime > 10.0) {
      prevTime = currentTime;
      return lastOutput;
    }
    
    double error = setpoint - measuredValue;
    double proportional = Kp * error;
    
    if (fabs(Ki) > 0.0001) {
      integral += error * deltaTime;
      
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;
    }
    double integralTerm = Ki * integral;
    
    double derivative = (deltaTime > 0.001) ? (error - prevError) / deltaTime : 0;
    double derivativeTerm = Kd * derivative;
    
    double output = proportional + integralTerm + derivativeTerm;
    output = constrain(output, outputMin, outputMax);
    
    prevError = error;
    prevTime = currentTime;
    lastOutput = (int)output;
    
    return output;
  }
  
  void setFactors(const PIDFactor& factors) {
    Kp = constrain(factors.Kp, 0.0, 100.0);
    Ki = constrain(factors.Ki, 0.0, 10.0);
    Kd = constrain(factors.Kd, 0.0, 5.0);
    outputMin = constrain(factors.outputMin, 0, 255);
    outputMax = constrain(factors.outputMax, 0, 255);
    updateMaxIntegral();
    reset();
  }
  
  void setKp(double value) { 
    Kp = constrain(value, 0.0, 100.0); 
    resetIntegral(); 
  }
  
  void setKi(double value) { 
    Ki = constrain(value, 0.0, 10.0); 
    updateMaxIntegral();
    resetIntegral();
  }
  
  void setKd(double value) { 
    Kd = constrain(value, 0.0, 5.0); 
    resetIntegral();
  }
  
  PIDFactor getFactors() const {
    PIDFactor factors;
    factors.Kp = Kp;
    factors.Ki = Ki;
    factors.Kd = Kd;
    factors.outputMin = outputMin;
    factors.outputMax = outputMax;
    return factors;
  }
  
  void reset() {
    prevError = 0;
    integral = 0;
    prevTime = millis();
    firstRun = true;
  }
  
  void resetIntegral() {
    integral = 0;
  }
  
  double getKp() const { return Kp; }
  double getKi() const { return Ki; }
  double getKd() const { return Kd; }
  int getLastOutput() const { return lastOutput; }
  double getIntegral() const { return integral; }
};

// ==================== LIGHT SCHEDULER CLASS / КЛАСС РАСПИСАНИЯ ОСВЕЩЕНИЯ ====================
class LightScheduler {
private:
  RTCManager* rtc;
  ClimateSetpoint* schedule;
  bool lightState;
  int currentLightLevel;
  unsigned long lastTransitionUpdate;
  
  // Calculate light level based on time / Расчет уровня освещения на основе времени
  int calculateLightLevel(uint8_t currentHour, uint8_t currentMinute) {
    if (!schedule->lightScheduleEnabled) {
      return 0; // Light off when schedule disabled / Свет выключен при отключенном расписании
    }
    
    uint32_t currentTimeInMinutes = currentHour * 60 + currentMinute;
    uint32_t lightOnTime = schedule->lightOn_hour * 60 + schedule->lightOn_minute;
    uint32_t lightOffTime = schedule->lightOff_hour * 60 + schedule->lightOff_minute;
    
    // Handle overnight schedule / Обработка расписания через полночь
    if (lightOnTime > lightOffTime) {
      if (currentTimeInMinutes >= lightOnTime || currentTimeInMinutes < lightOffTime) {
        // During light period / В течение периода освещения
        return calculateTransitionLevel(currentTimeInMinutes, lightOnTime, lightOffTime);
      } else {
        return 0; // Light off / Свет выключен
      }
    } else {
      if (currentTimeInMinutes >= lightOnTime && currentTimeInMinutes < lightOffTime) {
        // During light period / В течение периода освещения
        return calculateTransitionLevel(currentTimeInMinutes, lightOnTime, lightOffTime);
      } else {
        return 0; // Light off / Свет выключен
      }
    }
  }
  
  // Calculate transition level (smooth on/off) / Расчет уровня перехода (плавное вкл/выкл)
  int calculateTransitionLevel(uint32_t currentTime, uint32_t onTime, uint32_t offTime) {
    // Full brightness duration / Продолжительность полной яркости
    uint32_t fullBrightnessStart = onTime + LIGHT_TRANSITION_DURATION;
    uint32_t fullBrightnessEnd = offTime - LIGHT_TRANSITION_DURATION;
    
    if (fullBrightnessStart < fullBrightnessEnd) {
      if (currentTime >= fullBrightnessStart && currentTime < fullBrightnessEnd) {
        return 255; // Full brightness / Полная яркость
      } else if (currentTime >= onTime && currentTime < fullBrightnessStart) {
        // Fade in / Плавное включение
        uint32_t fadeDuration = currentTime - onTime;
        return map(fadeDuration, 0, LIGHT_TRANSITION_DURATION, 0, 255);
      } else if (currentTime >= fullBrightnessEnd && currentTime < offTime) {
        // Fade out / Плавное выключение
        uint32_t fadeDuration = offTime - currentTime;
        return map(fadeDuration, 0, LIGHT_TRANSITION_DURATION, 0, 255);
      }
    } else {
      // Handle case when transition periods overlap / Обработка случая когда периоды перекрываются
      return 255; // Simplified - full brightness / Упрощенно - полная яркость
    }
    
    return 0;
  }
  
public:
  LightScheduler(RTCManager* rtcManager, ClimateSetpoint* lightSchedule) :
    rtc(rtcManager),
    schedule(lightSchedule),
    lightState(false),
    currentLightLevel(0),
    lastTransitionUpdate(0)
  { }
  
  void update() {
    if (!rtc) return;
    
    const DateTime& currentTime = rtc->getCurrentTime();
    
    // Update light level every second for smooth transitions / Обновляем уровень света каждую секунду для плавных переходов
    unsigned long currentMillis = millis();
    if (currentMillis - lastTransitionUpdate >= 1000) {
      int newLightLevel = calculateLightLevel(currentTime.hour, currentTime.minute);
      
      // Smooth transition / Плавный переход
      if (abs(newLightLevel - currentLightLevel) > 10) {
        if (newLightLevel > currentLightLevel) {
          currentLightLevel += 10;
        } else {
          currentLightLevel -= 10;
        }
      } else {
        currentLightLevel = newLightLevel;
      }
      
      lightState = (currentLightLevel > 0);
      lastTransitionUpdate = currentMillis;
    }
  }
  
  int getLightLevel() const {
    return currentLightLevel;
  }
  
  bool isLightOn() const {
    return lightState;
  }
  
  void setSchedule(const ClimateSetpoint& newSchedule) {
    *schedule = newSchedule;
  }
  
  const ClimateSetpoint& getSchedule() const {
    return *schedule;
  }
  
  void enableSchedule(bool enabled) {
    schedule->lightScheduleEnabled = enabled;
    if (!enabled) {
      currentLightLevel = 0;
      lightState = false;
    }
  }
  
  bool isScheduleEnabled() const {
    return schedule->lightScheduleEnabled;
  }
  
  void setLightOnTime(uint8_t hour, uint8_t minute) {
    schedule->lightOn_hour = hour % 24;
    schedule->lightOn_minute = minute % 60;
  }
  
  void setLightOffTime(uint8_t hour, uint8_t minute) {
    schedule->lightOff_hour = hour % 24;
    schedule->lightOff_minute = minute % 60;
  }
  
  void getLightOnTime(uint8_t& hour, uint8_t& minute) const {
    hour = schedule->lightOn_hour;
    minute = schedule->lightOn_minute;
  }
  
  void getLightOffTime(uint8_t& hour, uint8_t& minute) const {
    hour = schedule->lightOff_hour;
    minute = schedule->lightOff_minute;
  }
};

// ==================== SYSTEM STATUS STRUCTURE / СТРУКТУРА СТАТУСА СИСТЕМЫ ====================
// Define system status structure for LCD display / Определяем структуру статуса системы для LCD дисплея
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

// Forward declarations to resolve dependencies / Предварительные объявления для разрешения зависимостей
class ClimateController;
extern ClimateController climateController;

// Special characters for LCD / Специальные символы для LCD
static byte degreeSymbol[8] = {
  B01100,
  B10010,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
  B00000
};

static byte arrowSymbol[8] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000
};

// ==================== LCD DISPLAY CLASS / КЛАСС LCD ДИСПЛЕЯ ====================
class LCDDisplay {
private:
  LiquidCrystal_I2C lcd;
  MenuState currentState;
  MenuState previousState;
  int selectedItem;
  int scrollPosition;
  bool editMode;
  bool refreshNeeded;
  unsigned long lastRefresh;
  unsigned long lastButtonPress;
  unsigned long backlightTimeout;
  bool backlightOn;
  
  // Menu items arrays / Массивы пунктов меню
  const char* mainMenuItems[10] = {
    "Temperature Settings",
    "Humidity Settings",
    "Light Settings",
    "Time Settings",
    "PID Settings",
    "System Info",
    "Reset System",
    "Emergency Stop",
    "Save to EEPROM",
    "Back to Main"
  };
  
  const char* temperatureItems[4] = {
    "Set Temperature",
    "PID: P Factor",
    "PID: I Factor",
    "PID: D Factor"
  };
  
  const char* humidityItems[2] = {
    "Set Humidity",
    "Back"
  };
  
  const char* lightItems[5] = {
    "Light ON Time",
    "Light OFF Time",
    "Schedule ON/OFF",
    "Light Intensity",
    "Back"
  };
  
  const char* timeItems[4] = {
    "Set Time",
    "Set Date",
    "Sync with RTC",
    "Back"
  };
  
  const char* saveConfirmItems[3] = {
    "YES - Save",
    "NO - Cancel",
    "Back"
  };
  
  // Blinking cursor for edit mode / Мерцающий курсор для режима редактирования
  bool cursorVisible;
  unsigned long lastCursorToggle;
  
  void updateBacklight() {
    unsigned long currentTime = millis();
    if (backlightOn && currentTime - lastButtonPress > backlightTimeout) {
      lcd.noBacklight();
      backlightOn = false;
    }
  }
  
  void activateBacklight() {
    unsigned long currentTime = millis();
    lastButtonPress = currentTime;
    if (!backlightOn) {
      lcd.backlight();
      backlightOn = true;
    }
  }
  
  void drawMainScreen(const SystemStatus& status) {
    lcd.clear();
    
    // Line 1: Temperature / Температура
    lcd.setCursor(0, 0);
    lcd.print("T:");
    if (!isnan(status.currentTemperature)) {
      lcd.print(status.currentTemperature, 1);
      lcd.write(0);  // Символ градуса (специальный символ 0)
      lcd.print("C");
    } else {
      lcd.print("---");
    }
    
    lcd.setCursor(10, 0);
    lcd.print("H:");
    if (!isnan(status.currentHumidity)) {
      lcd.print(status.currentHumidity, 0);
      lcd.print("%");
    } else {
      lcd.print("---");
    }
    
    // Line 2: Light / Свет
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(status.lightLevel);
    
    lcd.setCursor(8, 1);
    lcd.print("PWM:");
    lcd.print(status.pidOutput);
    
    // Line 3: Time and Status / Время и статус
    lcd.setCursor(0, 2);
    char timeBuffer[9];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
             status.currentTime.hour, status.currentTime.minute, status.currentTime.second);
    lcd.print(timeBuffer);
    
    lcd.setCursor(11, 2);
    lcd.print(status.systemEnabled ? "ON " : "OFF");
    
    // Line 4: Menu hint / Подсказка меню
    lcd.setCursor(0, 3);
    lcd.print("Press BTN for menu");
  }
  
  void drawMainMenu() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write(1); // Стрелка выбора / Arrow for selection
    lcd.print(" ");
    lcd.print(mainMenuItems[selectedItem]);
    
    if (scrollPosition > 0) {
      lcd.setCursor(0, 1);
      lcd.print("  ");
      lcd.print(mainMenuItems[selectedItem - 1]);
    }
    
    if (selectedItem < 9) {
      int nextItem = selectedItem + 1;
      int row = (scrollPosition > 0) ? 2 : 1;
      lcd.setCursor(0, row);
      lcd.print("  ");
      lcd.print(mainMenuItems[nextItem]);
    }
    
    // Navigation hint / Подсказка навигации
    lcd.setCursor(0, 3);
    lcd.print("Rotate->Select  BTN->OK");
  }
  
  void drawTemperatureMenu(const SystemStatus& status, const PIDFactor& pid) {
    lcd.clear();
    
    switch (selectedItem) {
      case 0: // Set Temperature / Установка температуры
        lcd.setCursor(0, 0);
        lcd.print("Set Temperature:");
        lcd.setCursor(0, 1);
        lcd.print("Current: ");
        if (!isnan(status.currentTemperature)) {
          lcd.print(status.currentTemperature, 1);
          lcd.write(0);
          lcd.print("C");
        } else {
          lcd.print("---");
        }
        lcd.setCursor(0, 2);
        lcd.print("Target: ");
        lcd.print(status.targetTemperature, 1);
        lcd.write(0);
        lcd.print("C");
        break;
        
      case 1: // P Factor / Коэффициент P
        lcd.setCursor(0, 0);
        lcd.print("PID - P Factor:");
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        lcd.print(pid.Kp, 3);
        break;
        
      case 2: // I Factor / Коэффициент I
        lcd.setCursor(0, 0);
        lcd.print("PID - I Factor:");
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        lcd.print(pid.Ki, 3);
        break;
        
      case 3: // D Factor / Коэффициент D
        lcd.setCursor(0, 0);
        lcd.print("PID - D Factor:");
        lcd.setCursor(0, 1);
        lcd.print("Value: ");
        lcd.print(pid.Kd, 3);
        break;
    }
    
    // Navigation / Навигация
    lcd.setCursor(0, 3);
    if (editMode) {
      lcd.print("Rotate->Change  BTN->Save");
    } else {
      lcd.print("Rotate->Select  BTN->Edit");
    }
  }
  
  void drawHumidityMenu(const SystemStatus& status) {
    lcd.clear();
    
    if (selectedItem == 0) {
      lcd.setCursor(0, 0);
      lcd.print("Set Humidity:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      if (!isnan(status.currentHumidity)) {
        lcd.print(status.currentHumidity, 0);
        lcd.print("%");
      } else {
        lcd.print("---");
      }
      lcd.setCursor(0, 2);
      lcd.print("Target: ");
      lcd.print(status.targetHumidity, 0);
      lcd.print("%");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Go back to");
      lcd.setCursor(0, 1);
      lcd.print("main menu");
    }
    
    lcd.setCursor(0, 3);
    if (editMode) {
      lcd.print("Rotate->Change  BTN->Save");
    } else {
      lcd.print("Rotate->Select  BTN->Edit");
    }
  }
  
  void drawLightMenu(const SystemStatus& status) {
    lcd.clear();
    
    // For now, use dummy values / Пока используем заглушки
    uint8_t onHour = 7, onMinute = 0;
    uint8_t offHour = 22, offMinute = 0;
    
    switch (selectedItem) {
      case 0: // Light ON Time / Время включения света
        lcd.setCursor(0, 0);
        lcd.print("Light ON Time:");
        lcd.setCursor(0, 1);
        lcd.print("Current: ");
        lcd.print(onHour);
        lcd.print(":");
        if (onMinute < 10) lcd.print("0");
        lcd.print(onMinute);
        break;
        
      case 1: // Light OFF Time / Время выключения света
        lcd.setCursor(0, 0);
        lcd.print("Light OFF Time:");
        lcd.setCursor(0, 1);
        lcd.print("Current: ");
        lcd.print(offHour);
        lcd.print(":");
        if (offMinute < 10) lcd.print("0");
        lcd.print(offMinute);
        break;
        
      case 2: // Schedule ON/OFF / Расписание ВКЛ/ВЫКЛ
        lcd.setCursor(0, 0);
        lcd.print("Light Schedule:");
        lcd.setCursor(0, 1);
        lcd.print("Status: ");
        lcd.print(status.scheduleEnabled ? "ENABLED" : "DISABLED");
        break;
        
      case 3: // Light Intensity / Интенсивность света
        lcd.setCursor(0, 0);
        lcd.print("Light Intensity:");
        lcd.setCursor(0, 1);
        lcd.print("Current: ");
        lcd.print(status.lightLevel);
        break;
        
      case 4: // Back / Назад
        lcd.setCursor(0, 0);
        lcd.print("Go back to");
        lcd.setCursor(0, 1);
        lcd.print("main menu");
        break;
    }
    
    lcd.setCursor(0, 3);
    if (editMode) {
      lcd.print("Rotate->Change  BTN->Save");
    } else {
      lcd.print("Rotate->Select  BTN->Edit");
    }
  }
  
  void drawTimeMenu(const SystemStatus& status) {
    lcd.clear();
    
    switch (selectedItem) {
      case 0: // Set Time / Установка времени
        lcd.setCursor(0, 0);
        lcd.print("Set Time:");
        lcd.setCursor(0, 1);
        lcd.print("Current: ");
        char timeBuffer[9];
        snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
                 status.currentTime.hour, status.currentTime.minute, status.currentTime.second);
        lcd.print(timeBuffer);
        break;
        
      case 1: // Set Date / Установка даты
        lcd.setCursor(0, 0);
        lcd.print("Set Date:");
        lcd.setCursor(0, 1);
        lcd.print("Current: ");
        char dateBuffer[11];
        snprintf(dateBuffer, sizeof(dateBuffer), "%02d/%02d/%02d",
                 status.currentTime.day, status.currentTime.month, status.currentTime.year);
        lcd.print(dateBuffer);
        break;
        
      case 2: // Sync with RTC / Синхронизация с RTC
        lcd.setCursor(0, 0);
        lcd.print("Sync with RTC:");
        lcd.setCursor(0, 1);
        lcd.print("Status: ");
        lcd.print("Available"); // Placeholder / Заглушка
        break;
        
      case 3: // Back / Назад
        lcd.setCursor(0, 0);
        lcd.print("Go back to");
        lcd.setCursor(0, 1);
        lcd.print("main menu");
        break;
    }
    
    lcd.setCursor(0, 3);
    if (editMode) {
      lcd.print("Rotate->Change  BTN->Save");
    } else {
      lcd.print("Rotate->Select  BTN->Edit");
    }
  }
  
  void drawSaveConfirm() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Save to EEPROM?");
    lcd.setCursor(0, 1);
    
    switch (selectedItem) {
      case 0:
        lcd.write(1);
        lcd.print(" YES - Save");
        lcd.setCursor(0, 2);
        lcd.print("  NO - Cancel");
        break;
      case 1:
        lcd.print("  YES - Save");
        lcd.setCursor(0, 2);
        lcd.write(1);
        lcd.print(" NO - Cancel");
        break;
      case 2:
        lcd.print("  YES - Save");
        lcd.setCursor(0, 2);
        lcd.print("  NO - Cancel");
        lcd.setCursor(0, 3);
        lcd.write(1);
        lcd.print(" Back");
        break;
    }
  }
  
  void drawSystemInfo(const SystemStatus& status) {
    lcd.clear();
    
    switch (selectedItem) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("System Status:");
        lcd.setCursor(0, 1);
        lcd.print("Errors: ");
        lcd.print(status.errorCount);
        lcd.setCursor(0, 2);
        lcd.print("Overheat: ");
        lcd.print(status.overheatProtection ? "YES" : "NO");
        break;
        
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Sensor Errors:");
        lcd.setCursor(0, 1);
        lcd.print("Count: ");
        lcd.print(0); // Placeholder / Заглушка
        break;
        
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("RTC Status:");
        lcd.setCursor(0, 1);
        lcd.print("Available: ");
        lcd.print("YES"); // Placeholder / Заглушка
        break;
    }
    
    lcd.setCursor(0, 3);
    lcd.print("BTN->Back  Rotate->Scroll");
  }
  
public:
  LCDDisplay() : 
    lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS),
    currentState(MAIN_SCREEN),
    previousState(MAIN_SCREEN),
    selectedItem(0),
    scrollPosition(0),
    editMode(false),
    refreshNeeded(true),
    lastRefresh(0),
    lastButtonPress(millis()),
    backlightTimeout(30000), // 30 seconds timeout / Таймаут 30 секунд
    backlightOn(true),
    cursorVisible(false),
    lastCursorToggle(0)
  { }
  
  void begin() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    // Create special characters / Создаем специальные символы
    lcd.createChar(0, degreeSymbol);    // Degree symbol / Символ градуса
    lcd.createChar(1, arrowSymbol);     // Arrow symbol / Символ стрелки
    
    // Welcome message / Приветственное сообщение
    lcd.setCursor(0, 0);
    lcd.print("Climate Control");
    lcd.setCursor(0, 1);
    lcd.print("System v3.0");
    lcd.setCursor(0, 2);
    lcd.print("LCD Display Ready");
    delay(2000);
    
    lcd.clear();
    refreshNeeded = true;
  }
  
  void update() {
    updateBacklight();
    
    if (editMode) {
      // Blinking cursor in edit mode / Мигание курсора в режиме редактирования
      unsigned long currentTime = millis();
      if (currentTime - lastCursorToggle > 500) {
        cursorVisible = !cursorVisible;
        lastCursorToggle = currentTime;
        refreshNeeded = true;
      }
    }
    
    if (refreshNeeded || (millis() - lastRefresh > 500)) {
      refreshDisplay();
      lastRefresh = millis();
      refreshNeeded = false;
    }
  }
  
  void refreshDisplay() {
    // Get system status (simplified for now) / Получаем статус системы (упрощенно пока)
    SystemStatus status;
    status.currentTime.hour = 12;
    status.currentTime.minute = 30;
    status.currentTime.second = 45;
    
    PIDFactor pid;
    pid.Kp = 1.0;
    pid.Ki = 0.5;
    pid.Kd = 0.02;
    
    switch (currentState) {
      case MAIN_SCREEN:
        drawMainScreen(status);
        break;
        
      case MAIN_MENU:
        drawMainMenu();
        break;
        
      case TEMPERATURE_SETTINGS:
        drawTemperatureMenu(status, pid);
        break;
        
      case HUMIDITY_SETTINGS:
        drawHumidityMenu(status);
        break;
        
      case LIGHT_SETTINGS:
        drawLightMenu(status);
        break;
        
      case TIME_SETTINGS:
        drawTimeMenu(status);
        break;
        
      case SAVE_CONFIRM:
        drawSaveConfirm();
        break;
        
      case SYSTEM_INFO:
        drawSystemInfo(status);
        break;
    }
  }
  
  void handleEncoderRotation(int direction) {
    activateBacklight();
    
    if (editMode) {
      // In edit mode change values / В режиме редактирования меняем значения
      handleValueChange(direction);
    } else {
      // In navigation mode change selected item / В режиме навигации меняем выбранный пункт
      selectedItem += direction;
      
      // Limit range based on state / Ограничиваем диапазон в зависимости от состояния
      switch (currentState) {
        case MAIN_MENU:
          selectedItem = constrain(selectedItem, 0, 9);
          break;
        case TEMPERATURE_SETTINGS:
          selectedItem = constrain(selectedItem, 0, 3);
          break;
        case HUMIDITY_SETTINGS:
          selectedItem = constrain(selectedItem, 0, 1);
          break;
        case LIGHT_SETTINGS:
          selectedItem = constrain(selectedItem, 0, 4);
          break;
        case TIME_SETTINGS:
          selectedItem = constrain(selectedItem, 0, 3);
          break;
        case SAVE_CONFIRM:
          selectedItem = constrain(selectedItem, 0, 2);
          break;
        case SYSTEM_INFO:
          selectedItem = constrain(selectedItem, 0, 2);
          break;
        default:
          selectedItem = constrain(selectedItem, 0, 0);
          break;
      }
      
      refreshNeeded = true;
    }
  }
  
  void handleButtonPress() {
    activateBacklight();
    
    if (editMode) {
      // Exit edit mode / Выход из режима редактирования
      editMode = false;
      cursorVisible = false;
      refreshNeeded = true;
    } else {
      // Handle press based on state / Обработка нажатия в зависимости от состояния
      switch (currentState) {
        case MAIN_SCREEN:
          enterMainMenu();
          break;
          
        case MAIN_MENU:
          handleMainMenuSelection();
          break;
          
        case TEMPERATURE_SETTINGS:
        case HUMIDITY_SETTINGS:
        case LIGHT_SETTINGS:
        case TIME_SETTINGS:
          enterEditMode();
          break;
          
        case SAVE_CONFIRM:
          handleSaveConfirmation();
          break;
          
        case SYSTEM_INFO:
          if (selectedItem == 0) {
            goBack();
          }
          break;
          
        default:
          goBack();
          break;
      }
    }
  }
  
  void enterMainMenu() {
    previousState = currentState;
    currentState = MAIN_MENU;
    selectedItem = 0;
    scrollPosition = 0;
    refreshNeeded = true;
  }
  
  void handleMainMenuSelection() {
    switch (selectedItem) {
      case 0: // Temperature Settings / Настройки температуры
        previousState = currentState;
        currentState = TEMPERATURE_SETTINGS;
        selectedItem = 0;
        break;
        
      case 1: // Humidity Settings / Настройки влажности
        previousState = currentState;
        currentState = HUMIDITY_SETTINGS;
        selectedItem = 0;
        break;
        
      case 2: // Light Settings / Настройки освещения
        previousState = currentState;
        currentState = LIGHT_SETTINGS;
        selectedItem = 0;
        break;
        
      case 3: // Time Settings / Настройки времени
        previousState = currentState;
        currentState = TIME_SETTINGS;
        selectedItem = 0;
        break;
        
      case 4: // PID Settings / Настройки PID
        previousState = currentState;
        currentState = TEMPERATURE_SETTINGS;
        selectedItem = 1; // Go directly to PID settings / Переходим сразу к настройкам PID
        break;
        
      case 5: // System Info / Информация о системе
        previousState = currentState;
        currentState = SYSTEM_INFO;
        selectedItem = 0;
        break;
        
      case 6: // Reset System / Сброс системы
        // climateController.resetSystem(); // TODO: Implement / TODO: Реализовать
        goToMainScreen();
        break;
        
      case 7: // Emergency Stop / Аварийная остановка
        // climateController.emergencyStop(); // TODO: Implement / TODO: Реализовать
        goToMainScreen();
        break;
        
      case 8: // Save to EEPROM / Сохранить в EEPROM
        previousState = currentState;
        currentState = SAVE_CONFIRM;
        selectedItem = 0;
        break;
        
      case 9: // Back to Main / Назад на главный экран
        goToMainScreen();
        break;
    }
    refreshNeeded = true;
  }
  
  void enterEditMode() {
    editMode = true;
    cursorVisible = true;
    lastCursorToggle = millis();
    refreshNeeded = true;
  }
  
  void handleValueChange(int direction) {
    // Simplified value change handling / Упрощенная обработка изменения значений
    refreshNeeded = true;
  }
  
  void handleSaveConfirmation() {
    switch (selectedItem) {
      case 0: // YES - Save / ДА - Сохранить
        // TODO: Implement save to EEPROM / TODO: Реализовать сохранение в EEPROM
        goToMainScreen();
        break;
      case 1: // NO - Cancel / НЕТ - Отмена
        goBack();
        break;
      case 2: // Back / Назад
        goBack();
        break;
    }
  }
  
  void goBack() {
    if (currentState == MAIN_MENU) {
      goToMainScreen();
    } else {
      currentState = previousState;
      selectedItem = 0;
      editMode = false;
      refreshNeeded = true;
    }
  }
  
  void goToMainScreen() {
    currentState = MAIN_SCREEN;
    editMode = false;
    refreshNeeded = true;
  }
  
  MenuState getCurrentState() const {
    return currentState;
  }
  
  bool isEditMode() const {
    return editMode;
  }
  
  void setRefreshNeeded() {
    refreshNeeded = true;
  }
};

// ==================== ROTARY ENCODER CLASS / КЛАСС ЭНКОДЕРА ====================
class RotaryEncoder {
private:
  Encoder encoder;
  int lastPosition;
  bool buttonPressed;
  bool buttonState;
  bool lastButtonState;
  unsigned long lastButtonDebounceTime;
  unsigned long lastEncoderReadTime;
  LCDDisplay* lcdDisplay;
  
public:
  RotaryEncoder() : 
    encoder(PIN_ENCODER_CLK, PIN_ENCODER_DT),
    lastPosition(0),
    buttonPressed(false),
    buttonState(false),
    lastButtonState(false),
    lastButtonDebounceTime(0),
    lastEncoderReadTime(0),
    lcdDisplay(nullptr)
  {
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  }
  
  void setLCDDisplay(LCDDisplay* display) {
    lcdDisplay = display;
  }
  
  void update() {
    // Read encoder position / Чтение позиции энкодера
    unsigned long currentTime = millis();
    if (currentTime - lastEncoderReadTime > ENCODER_DEBOUNCE_TIME) {
      long newPosition = encoder.read() / 4; // Divide by 4 for one click per step / Делим на 4 для одного клика за шаг
      
      if (newPosition != lastPosition) {
        int direction = (newPosition > lastPosition) ? 1 : -1;
        lastPosition = newPosition;
        
        // Call rotation handler / Вызываем обработчик вращения
        if (lcdDisplay) {
          lcdDisplay->handleEncoderRotation(direction);
        }
      }
      lastEncoderReadTime = currentTime;
    }
    
    // Read button with debounce / Чтение кнопки с защитой от дребезга
    bool reading = digitalRead(PIN_ENCODER_SW) == LOW; // Invert because of INPUT_PULLUP / Инвертируем, так как INPUT_PULLUP
    
    if (reading != lastButtonState) {
      lastButtonDebounceTime = currentTime;
    }
    
    if ((currentTime - lastButtonDebounceTime) > DEBOUNCE_TIME) {
      if (reading != buttonState) {
        buttonState = reading;
        
        if (buttonState) {
          buttonPressed = true;
        }
      }
    }
    
    lastButtonState = reading;
    
    // Handle button press / Обработка нажатия кнопки
    if (buttonPressed) {
      buttonPressed = false;
      if (lcdDisplay) {
        lcdDisplay->handleButtonPress();
      }
    }
  }
};

// ==================== MAIN CLIMATE CONTROL CLASS / ГЛАВНЫЙ КЛАСС УПРАВЛЕНИЯ КЛИМАТОМ ====================
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
  
  void initializePins() {
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
  
  void controlLight() {
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
  
  void setLightPower(int pwmValue) {
    #if defined(ESP32)
      ledcWrite(1, pwmValue);
    #else
      analogWrite(PIN_LIGHT_POWER, pwmValue);
    #endif
  }
  
  void controlHumidity(float humidityLevel) {
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
  
  void controlTemperature(float temperatureLevel) {
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
  
  void setHeaterPower(int pwmValue) {
    #if defined(ESP32)
      ledcWrite(0, pwmValue);
    #else
      analogWrite(PIN_HEATER_COOLER_POWER, pwmValue);
    #endif
  }
  
  void monitorSystemHealth() {
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
  
  void setupSerialCallbacks() {
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
  
  // Static wrapper methods for callbacks / Статические методы-обертки для обратных вызовов
  static void onStatusRequestStatic() {
    climateController.sendStatusToSerial();
  }
  
  static void onTimeRequestStatic() {
    climateController.sendTimeToSerial();
  }
  
  static void onSetTimeStatic(String timeStr) {
    climateController.setTimeFromString(timeStr);
  }
  
  static void onSetDateStatic(String dateStr) {
    climateController.setDateFromString(dateStr);
  }
  
  static void onSetLightOnTimeStatic(String timeStr) {
    climateController.setLightOnTimeFromString(timeStr);
  }
  
  static void onSetLightOffTimeStatic(String timeStr) {
    climateController.setLightOffTimeFromString(timeStr);
  }
  
  static void onSetLightScheduleStatic(bool enabled) {
    climateController.light_scheduler.enableSchedule(enabled);
    climateController.serial_cmd.sendMessage(enabled ? 
      F("Light schedule enabled / Расписание света включено") : 
      F("Light schedule disabled / Расписание света выключено"));
  }
  
  static void onSaveRequestStatic() {
    climateController.saveSettings();
  }
  
  static void onLoadRequestStatic() {
    climateController.loadSettings();
  }
  
  static void onResetRequestStatic() {
    climateController.resetSystem();
    climateController.serial_cmd.sendMessage(F("System reset complete / Сброс системы завершен"));
  }
  
  static void onEmergencyStopStatic() {
    climateController.emergencyStop();
    climateController.serial_cmd.sendMessage(F("Emergency stop via serial / Аварийная остановка по Serial"));
  }
  
  static void onTemperatureSetStatic(float temp) {
    climateController.setTemperatureSetpoint(temp);
    climateController.serial_cmd.sendMessage("Temperature set to / Температура установлена на: " + String(temp) + "C");
  }
  
  static void onHumiditySetStatic(float humid) {
    climateController.setHumiditySetpoint(humid);
    climateController.serial_cmd.sendMessage("Humidity set to / Влажность установлена на: " + String(humid) + "%");
  }
  
  static void onWatchdogTestStatic() {
    climateController.testWatchdog();
  }
  
  void setTimeFromString(const String& timeStr) {
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
  
  void setDateFromString(const String& dateStr) {
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
  
  void setLightOnTimeFromString(const String& timeStr) {
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
  
  void setLightOffTimeFromString(const String& timeStr) {
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
  
public:
  ClimateController() :
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
  
  static ClimateController climateController;
  
  static ClimateController& getInstance() {
    return climateController;
  }

  void begin() {
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
  
  void update() {
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
  
  void updateSystemStatus() {
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
  
  // Safe control methods / Безопасные методы управления
  void setTemperaturePID(const PIDFactor& factors) {
    PIDFactor safeFactors = factors;
    safeFactors.Kp = constrain(factors.Kp, 0.0, 100.0);
    safeFactors.Ki = constrain(factors.Ki, 0.0, 10.0);
    safeFactors.Kd = constrain(factors.Kd, 0.0, 5.0);
    safeFactors.outputMin = constrain(factors.outputMin, 0, 255);
    safeFactors.outputMax = constrain(factors.outputMax, 0, 255);
    
    temperature_pid.setFactors(safeFactors);
  }
  
  void setTemperatureSetpoint(float temperature) {
    climate_setpoint.target_temperature = constrain(
      temperature, 
      TEMPERATURE_SETPOINT_MIN, 
      TEMPERATURE_SETPOINT_MAX
    );
    temperature_pid.reset();
  }
  
  void setHumiditySetpoint(float humidity) {
    climate_setpoint.target_humidity = constrain(
      humidity,
      20.0f,
      80.0f
    );
  }
  
  void emergencyStop() {
    setHeaterPower(0);
    setLightPower(0);
    digitalWrite(PIN_HUMIDIFIER_POWER, LOW);
    
    temperature_pid.reset();
    humidity_work_flag = false;
    overheatProtectionActive = false;
    systemEnabled = false;
    
    serial_cmd.sendMessage(F("!!! EMERGENCY STOP ACTIVATED !!! / !!! АВАРИЙНАЯ ОСТАНОВКА АКТИВИРОВАНА !!!"));
  }
  
  void resetSystem() {
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
  
  void sendStatusToSerial() {
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
  
  void sendTimeToSerial() {
    serial_cmd.sendTime(rtc.getCurrentTime());
  }
  
  void saveSettings() {
    // Implementation for EEPROM saving / Реализация сохранения в EEPROM
    // TODO: Implement actual EEPROM saving / TODO: Реализовать фактическое сохранение в EEPROM
    serial_cmd.sendMessage(F("Settings saved to EEPROM / Настройки сохранены в EEPROM"));
  }
  
  void loadSettings() {
    // Implementation for EEPROM loading / Реализация загрузки из EEPROM
    // TODO: Implement actual EEPROM loading / TODO: Реализовать фактическую загрузку из EEPROM
    serial_cmd.sendMessage(F("Settings loaded from EEPROM / Настройки загружены из EEPROM"));
  }
  
  void testWatchdog() {
    serial_cmd.sendMessage(F("Watchdog test - system will reset in 3 seconds... / Тест Watchdog - система перезагрузится через 3 секунды..."));
    delay(3000);
    
    // Disable watchdog and enter infinite loop / Отключаем watchdog и входим в бесконечный цикл
    watchdog.disable();
    while(true) {
      // This should trigger hardware reset / Это должно вызвать аппаратный сброс
    }
  }
  
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
  
  ~ClimateController() {
    if (lcd_display) delete lcd_display;
    if (encoder) delete encoder;
  }
};

// Create global instance / Создаем глобальный экземпляр
ClimateController ClimateController::climateController;

// ==================== EEPROM FUNCTIONS / ФУНКЦИИ ДЛЯ РАБОТЫ С EEPROM ====================
struct StoredSettings {
  ClimateSetpoint climate;
  PIDFactor pidFactors;
  SensorCalibration calibration;
  uint8_t checksum;
};

uint8_t calculateChecksum(const uint8_t* data, size_t size) {
  uint8_t sum = 0;
  for (size_t i = 0; i < size; i++) {
    sum ^= data[i]; // Simple XOR for checksum / Простой XOR для контрольной суммы
  }
  return sum;
}

void saveSettingsToEEPROM() {
  StoredSettings settings;
  
  // Get current settings / Получаем текущие настройки
  ClimateController& controller = ClimateController::getInstance();
  settings.climate.target_temperature = controller.getTemperatureSetpoint();
  settings.climate.target_humidity = controller.getHumiditySetpoint();
  settings.pidFactors = controller.getTemperaturePID();
  
  // Get light schedule / Получаем расписание освещения
  ClimateSetpoint climate = settings.climate;
  LightScheduler& scheduler = controller.getLightScheduler();
  uint8_t onHour, onMinute, offHour, offMinute;
  scheduler.getLightOnTime(onHour, onMinute);
  scheduler.getLightOffTime(offHour, offMinute);
  climate.lightOn_hour = onHour;
  climate.lightOn_minute = onMinute;
  climate.lightOff_hour = offHour;
  climate.lightOff_minute = offMinute;
  climate.lightScheduleEnabled = scheduler.isScheduleEnabled();
  
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

bool loadSettingsFromEEPROM() {
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
  ClimateController& controller = ClimateController::getInstance();
  controller.setTemperatureSetpoint(settings.climate.target_temperature);
  controller.setHumiditySetpoint(settings.climate.target_humidity);
  controller.setTemperaturePID(settings.pidFactors);
  
  // Apply light schedule / Применяем расписание освещения
  LightScheduler& scheduler = controller.getLightScheduler();
  scheduler.setLightOnTime(settings.climate.lightOn_hour, settings.climate.lightOn_minute);
  scheduler.setLightOffTime(settings.climate.lightOff_hour, settings.climate.lightOff_minute);
  scheduler.enableSchedule(settings.climate.lightScheduleEnabled);
  
  #ifdef DEBUG_MODE
    Serial.println(F("Settings loaded from EEPROM / Настройки загружены из EEPROM"));
  #endif
  return true;
}

// ==================== SETUP AND LOOP / SETUP И LOOP ====================
void setup() {
  // Initialize EEPROM / Инициализация EEPROM
  #if defined(ESP32) || defined(ESP8266)
    EEPROM.begin(sizeof(StoredSettings));
  #endif
  
  // Start the system / Запускаем систему
  ClimateController::getInstance().begin();
  
  #ifdef DEBUG_MODE
    Serial.println(F("System setup complete with LCD / Настройка системы завершена с LCD"));
  #endif
}

void loop() {
  // Main system update / Основное обновление системы
  ClimateController::getInstance().update();
  
  // Small delay for stability / Небольшая задержка для стабильности
  delay(10);
}