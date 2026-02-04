/*
 * RTC Manager implementation
 * Реализация менеджера RTC
 */

#include "rtc_manager.h"

void RTCManager::begin() {
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

void RTCManager::update() {
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

void RTCManager::setTime(const DateTime& newTime) {
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

void RTCManager::setTime(uint8_t hour, uint8_t minute, uint8_t second) {
  DateTime newTime = currentTime;
  newTime.hour = hour;
  newTime.minute = minute;
  newTime.second = second;
  setTime(newTime);
}

void RTCManager::setDate(uint8_t year, uint8_t month, uint8_t day) {
  DateTime newTime = currentTime;
  newTime.year = year;
  newTime.month = month;
  newTime.day = day;
  setTime(newTime);
}

bool RTCManager::isTimeInRange(uint8_t startHour, uint8_t startMinute, 
                               uint8_t endHour, uint8_t endMinute) const {
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

void RTCManager::printTime(const DateTime& dt) {
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d %02d/%02d/%02d", 
           dt.hour, dt.minute, dt.second, dt.day, dt.month, dt.year + 2000);
  Serial.print(buffer);
}

void RTCManager::printCurrentTime() {
  printTime(currentTime);
}