/*
 * RTC Manager class for DS1302
 * Класс управления RTC для DS1302
 */

#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <Arduino.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include "config.h"
#include "structures.h"

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
  
  void begin();
  
  void update();
  
  const DateTime& getCurrentTime() const {
    return currentTime;
  }
  
  void setTime(const DateTime& newTime);
  
  void setTime(uint8_t hour, uint8_t minute, uint8_t second);
  
  void setDate(uint8_t year, uint8_t month, uint8_t day);
  
  bool isRTCAvailable() const {
    return rtcAvailable;
  }
  
  uint32_t getTimeInSeconds() const {
    return currentTime.hour * 3600UL + currentTime.minute * 60UL + currentTime.second;
  }
  
  bool isTimeInRange(uint8_t startHour, uint8_t startMinute, uint8_t endHour, uint8_t endMinute) const;
  
  void printTime(const DateTime& dt);
  
  void printCurrentTime();
  
  ~RTCManager() {
    if (rtc) delete rtc;
    if (rtcWire) delete rtcWire;
  }
};

#endif // RTC_MANAGER_H