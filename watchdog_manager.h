/*
 * Watchdog timer manager class
 * Класс управления watchdog таймером
 */

#ifndef WATCHDOG_MANAGER_H
#define WATCHDOG_MANAGER_H

#include <Arduino.h>
#include "config.h"

#ifdef __AVR__
  #include <avr/wdt.h>  // AVR Watchdog timer library
#endif

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

#endif // WATCHDOG_MANAGER_H