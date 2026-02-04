/*
 * Light scheduler implementation
 * Реализация планировщика освещения
 */

#include "light_scheduler.h"
#include "rtc_manager.h"

LightScheduler::LightScheduler(RTCManager* rtcManager, ClimateSetpoint* lightSchedule) :
  rtc(rtcManager),
  schedule(lightSchedule),
  lightState(false),
  currentLightLevel(0),
  lastTransitionUpdate(0)
{ }

int LightScheduler::calculateLightLevel(uint8_t currentHour, uint8_t currentMinute) {
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

int LightScheduler::calculateTransitionLevel(uint32_t currentTime, uint32_t onTime, uint32_t offTime) {
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

void LightScheduler::update() {
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

void LightScheduler::enableSchedule(bool enabled) {
  schedule->lightScheduleEnabled = enabled;
  if (!enabled) {
    currentLightLevel = 0;
    lightState = false;
  }
}

void LightScheduler::setLightOnTime(uint8_t hour, uint8_t minute) {
  schedule->lightOn_hour = hour % 24;
  schedule->lightOn_minute = minute % 60;
}

void LightScheduler::setLightOffTime(uint8_t hour, uint8_t minute) {
  schedule->lightOff_hour = hour % 24;
  schedule->lightOff_minute = minute % 60;
}

void LightScheduler::getLightOnTime(uint8_t& hour, uint8_t& minute) const {
  hour = schedule->lightOn_hour;
  minute = schedule->lightOn_minute;
}

void LightScheduler::getLightOffTime(uint8_t& hour, uint8_t& minute) const {
  hour = schedule->lightOff_hour;
  minute = schedule->lightOff_minute;
}