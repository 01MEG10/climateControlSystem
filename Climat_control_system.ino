/*
 * Climate Control System with RTC and LCD Display
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

#include "config.h"
#include "structures.h"
#include "climate_controller.h"

// ==================== SETUP FUNCTION / ФУНКЦИЯ SETUP ====================
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

// ==================== LOOP FUNCTION / ФУНКЦИЯ LOOP ====================
void loop() {
  // Main system update / Основное обновление системы
  ClimateController::getInstance().update();
  
  // Small delay for stability / Небольшая задержка для стабильности
  delay(10);
}