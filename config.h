/*
 * Configuration file for Climate Control System
 * Файл конфигурации системы контроля климата
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==================== PIN DEFINITIONS / ОПРЕДЕЛЕНИЯ ПИНОВ ====================

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

// ==================== SENSOR CONFIG / КОНФИГУРАЦИЯ ДАТЧИКОВ ====================

// Sensor type / Тип датчика
#define DHT_TYPE DHT11

// Invalid values for error handling / Невалидные значения для обработки ошибок
#define INVALID_INT -1
#define INVALID_FLOAT NAN

// ==================== TIME CONSTANTS / КОНСТАНТЫ ВРЕМЕНИ ====================

#define SECOND_MS 1000UL
#define MINUTE_MS (60UL * SECOND_MS)
#define HOUR_MS (60UL * MINUTE_MS)

// Operation intervals / Интервалы работы
#define SENSOR_READ_INTERVAL 9000     // 9 seconds between measurements / 9 секунд между замерами
#define SERIAL_UPDATE_INTERVAL 5000   // 5 seconds between status updates / 5 секунд между обновлениями статуса
#define RTC_UPDATE_INTERVAL 60000UL   // 1 minute between RTC checks / 1 минута между проверками RTC

// ==================== LIGHT CONTROL SETTINGS / НАСТРОЙКИ УПРАВЛЕНИЯ ОСВЕЩЕНИЕМ ====================

#define LIGHT_SCHEDULE_ENABLED true   // Enable light schedule / Включить расписание освещения
#define DEFAULT_LIGHT_ON_HOUR 7       // Default light ON hour / Час включения света по умолчанию
#define DEFAULT_LIGHT_OFF_HOUR 22     // Default light OFF hour / Час выключения света по умолчанию
#define LIGHT_TRANSITION_DURATION 10  // Light transition in minutes / Плавное включение в минутах

// ==================== HUMIDIFIER SETTINGS / НАСТРОЙКИ УВЛАЖНИТЕЛЯ ====================

#define HUMIDIFIER_HYSTERESIS 3.0f
#define HUMIDIFIER_MINIMAL_WORK_TIME (60UL * SECOND_MS)    // 1 minute / 1 минута
#define HUMIDIFIER_MAXIMUM_WORK_TIME (10UL * MINUTE_MS)    // 10 minutes / 10 минут
#define HUMIDIFIER_MIN_PAUSE_TIME (30UL * SECOND_MS)       // 30 seconds minimum pause / 30 секунд минимальная пауза

// ==================== SAFETY CONSTANTS / КОНСТАНТЫ БЕЗОПАСНОСТИ ====================

#define MAX_SAFE_TEMPERATURE 45.0f
#define MIN_SAFE_TEMPERATURE 5.0f
#define TEMPERATURE_SETPOINT_MIN 10.0f
#define TEMPERATURE_SETPOINT_MAX 40.0f
#define MAX_SAFE_HUMIDITY 90.0f
#define MIN_SAFE_HUMIDITY 10.0f

// ==================== WATCHDOG SETTINGS / НАСТРОЙКИ WATCHDOG ====================

#define WATCHDOG_TIMEOUT WDTO_2S     // 2 second timeout / Таймаут 2 секунды
#define WATCHDOG_RESET_THRESHOLD 3   // Number of resets before full restart / Количество сбросов перед полной перезагрузкой

// ==================== DEBOUNCE SETTINGS / НАСТРОЙКИ ЗАЩИТЫ ОТ ДРЕБЕЗГА ====================

#define DEBOUNCE_TIME 50             // Debounce time in ms / Время защиты от дребезга в мс
#define ENCODER_DEBOUNCE_TIME 10     // Encoder debounce / Защита от дребезга энкодера

// ==================== DEBUG SETTINGS / НАСТРОЙКИ ОТЛАДКИ ====================

// Uncomment for debug mode / Раскомментировать для режима отладки
// #define DEBUG_MODE

// ==================== PLATFORM SPECIFIC / СПЕЦИФИЧНЫЕ ДЛЯ ПЛАТФОРМЫ ====================

// Define ULONG_MAX for Arduino if not defined / Определяем ULONG_MAX для Arduino если не определено
#ifndef ULONG_MAX
  #define ULONG_MAX 4294967295UL
#endif

// ADC settings depending on platform / Настройки ADC в зависимости от платформы
#if defined(ESP32) || defined(ESP8266)
  #define ADC_MAX_VALUE 4095          // For ESP - 12-bit / Для ESP - 12-бит
  #define ADC_BITS 12
#else
  #define ADC_MAX_VALUE 1023          // For Arduino - 10-bit / Для Arduino - 10-бит
  #define ADC_BITS 10
#endif

#endif // CONFIG_H