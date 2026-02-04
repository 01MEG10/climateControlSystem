/*
 * Safe timer class with overflow protection
 * Класс безопасного таймера с защитой от переполнения
 */

#ifndef SAFE_TIMER_H
#define SAFE_TIMER_H

#include <Arduino.h>

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

#endif // SAFE_TIMER_H