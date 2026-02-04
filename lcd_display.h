/*
 * LCD display class with menu system
 * Класс LCD дисплея с системой меню
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "config.h"
#include "structures.h"

// Forward declaration / Предварительное объявление
class ClimateController;

class LCDDisplay {
private:
  LiquidCrystal_I2C lcd;
  MenuState currentState;
  MenuState previousState;
  int selectedItem;
  int scrollPosition;
  bool editMode;
  bool refreshNeeded;
  unsigned long lastRefresh;
  unsigned long lastButtonPress;
  unsigned long backlightTimeout;
  bool backlightOn;
  
  // Menu items arrays / Массивы пунктов меню
  static const int MAIN_MENU_ITEMS = 10;
  const char* mainMenuItems[MAIN_MENU_ITEMS];
  
  const char* temperatureItems[4];
  const char* humidityItems[2];
  const char* lightItems[5];
  const char* timeItems[4];
  const char* saveConfirmItems[3];
  
  // Blinking cursor for edit mode / Мерцающий курсор для режима редактирования
  bool cursorVisible;
  unsigned long lastCursorToggle;
  
  void updateBacklight();
  void activateBacklight();
  
  // Drawing methods / Методы отрисовки
  void drawMainScreen(const SystemStatus& status);
  void drawMainMenu();
  void drawTemperatureMenu(const SystemStatus& status, const PIDFactor& pid);
  void drawHumidityMenu(const SystemStatus& status);
  void drawLightMenu(const SystemStatus& status);
  void drawTimeMenu(const SystemStatus& status);
  void drawSaveConfirm();
  void drawSystemInfo(const SystemStatus& status);
  
public:
  LCDDisplay();
  
  void begin();
  
  void update();
  
  void refreshDisplay();
  
  void handleEncoderRotation(int direction);
  
  void handleButtonPress();
  
  void enterMainMenu();
  
  void handleMainMenuSelection();
  
  void enterEditMode();
  
  void handleValueChange(int direction);
  
  void handleSaveConfirmation();
  
  void goBack();
  
  void goToMainScreen();
  
  MenuState getCurrentState() const {
    return currentState;
  }
  
  bool isEditMode() const {
    return editMode;
  }
  
  void setRefreshNeeded() {
    refreshNeeded = true;
  }
};

#endif // LCD_DISPLAY_H