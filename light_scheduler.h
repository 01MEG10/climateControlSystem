/*
 * Light scheduler class for time-based light control
 * Класс планировщика освещения для временного управления светом
 */

#ifndef LIGHT_SCHEDULER_H
#define LIGHT_SCHEDULER_H

#include <Arduino.h>
#include "config.h"
#include "structures.h"

// Forward declaration / Предварительное объявление
class RTCManager;

class LightScheduler {
private:
  RTCManager* rtc;
  ClimateSetpoint* schedule;
  bool lightState;
  int currentLightLevel;
  unsigned long lastTransitionUpdate;
  
  // Calculate light level based on time / Расчет уровня освещения на основе времени
  int calculateLightLevel(uint8_t currentHour, uint8_t currentMinute);
  
  // Calculate transition level (smooth on/off) / Расчет уровня перехода (плавное вкл/выкл)
  int calculateTransitionLevel(uint32_t currentTime, uint32_t onTime, uint32_t offTime);
  
public:
  LightScheduler(RTCManager* rtcManager, ClimateSetpoint* lightSchedule);
  
  void update();
  
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
  
  void enableSchedule(bool enabled);
  
  bool isScheduleEnabled() const {
    return schedule->lightScheduleEnabled;
  }
  
  void setLightOnTime(uint8_t hour, uint8_t minute);
  
  void setLightOffTime(uint8_t hour, uint8_t minute);
  
  void getLightOnTime(uint8_t& hour, uint8_t& minute) const;
  
  void getLightOffTime(uint8_t& hour, uint8_t& minute) const;
};

#endif // LIGHT_SCHEDULER_H