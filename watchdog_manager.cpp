/*
 * Watchdog manager implementation
 * Реализация менеджера watchdog
 */

#include "watchdog_manager.h"

void WatchdogManager::enable() {
  #ifdef __AVR__
    wdt_enable(WATCHDOG_TIMEOUT);
    watchdogEnabled = true;
    lastFeedTime = millis();
    
    #ifdef DEBUG_MODE
      Serial.println(F("Watchdog enabled / Watchdog включен"));
    #endif
  #endif
}

void WatchdogManager::disable() {
  #ifdef __AVR__
    wdt_disable();
    watchdogEnabled = false;
    
    #ifdef DEBUG_MODE
      Serial.println(F("Watchdog disabled / Watchdog выключен"));
    #endif
  #endif
}

void WatchdogManager::feed() {
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

void WatchdogManager::systemReset() {
  #ifdef __AVR__
    wdt_enable(WDTO_15MS);
    while(1) {}
  #endif
}

void WatchdogManager::incrementResetCount() {
  resetCount++;
  if (resetCount >= WATCHDOG_RESET_THRESHOLD) {
    #ifdef DEBUG_MODE
      Serial.println(F("Watchdog: Too many resets, performing system reset / Watchdog: Слишком много сбросов, выполняю перезагрузку системы"));
    #endif
    systemReset();
  }
}

int WatchdogManager::getResetCount() const {
  return resetCount;
}

bool WatchdogManager::isEnabled() const {
  return watchdogEnabled;
}