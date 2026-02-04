/*
 * Rotary encoder class for menu navigation
 * Класс энкодера для навигации по меню
 */

#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <Arduino.h>
#include <Encoder.h>
#include "config.h"

// Forward declaration / Предварительное объявление
class LCDDisplay;

class RotaryEncoder {
private:
  Encoder encoder;
  int lastPosition;
  bool buttonPressed;
  bool buttonState;
  bool lastButtonState;
  unsigned long lastButtonDebounceTime;
  unsigned long lastEncoderReadTime;
  LCDDisplay* lcdDisplay;
  
public:
  RotaryEncoder();
  
  void setLCDDisplay(LCDDisplay* display) {
    lcdDisplay = display;
  }
  
  void update();
};

#endif // ROTARY_ENCODER_H