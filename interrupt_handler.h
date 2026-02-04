/*
 * Interrupt handler class for emergency button
 * Класс обработки прерываний для аварийной кнопки
 */

#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#include <Arduino.h>
#include "config.h"

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

#endif // INTERRUPT_HANDLER_H