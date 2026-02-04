/*
 * Rotary encoder implementation
 * Реализация энкодера
 */

#include "rotary_encoder.h"
#include "lcd_display.h"

RotaryEncoder::RotaryEncoder() : 
  encoder(PIN_ENCODER_CLK, PIN_ENCODER_DT),
  lastPosition(0),
  buttonPressed(false),
  buttonState(false),
  lastButtonState(false),
  lastButtonDebounceTime(0),
  lastEncoderReadTime(0),
  lcdDisplay(nullptr)
{
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
}

void RotaryEncoder::update() {
  // Read encoder position / Чтение позиции энкодера
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderReadTime > ENCODER_DEBOUNCE_TIME) {
    long newPosition = encoder.read() / 4; // Divide by 4 for one click per step / Делим на 4 для одного клика за шаг
    
    if (newPosition != lastPosition) {
      int direction = (newPosition > lastPosition) ? 1 : -1;
      lastPosition = newPosition;
      
      // Call rotation handler / Вызываем обработчик вращения
      if (lcdDisplay) {
        lcdDisplay->handleEncoderRotation(direction);
      }
    }
    lastEncoderReadTime = currentTime;
  }
  
  // Read button with debounce / Чтение кнопки с защитой от дребезга
  bool reading = digitalRead(PIN_ENCODER_SW) == LOW; // Invert because of INPUT_PULLUP / Инвертируем, так как INPUT_PULLUP
  
  if (reading != lastButtonState) {
    lastButtonDebounceTime = currentTime;
  }
  
  if ((currentTime - lastButtonDebounceTime) > DEBOUNCE_TIME) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState) {
        buttonPressed = true;
      }
    }
  }
  
  lastButtonState = reading;
  
  // Handle button press / Обработка нажатия кнопки
  if (buttonPressed) {
    buttonPressed = false;
    if (lcdDisplay) {
      lcdDisplay->handleButtonPress();
    }
  }
}