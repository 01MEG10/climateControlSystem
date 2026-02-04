/*
 * LCD display implementation
 * Реализация LCD дисплея
 */

#include "lcd_display.h"
#include "climate_controller.h"

// Special characters for LCD / Специальные символы для LCD
static byte degreeSymbol[8] = {
  B01100,
  B10010,
  B10010,
  B01100,
  B00000,
  B00000,
  B00000,
  B00000
};

static byte arrowSymbol[8] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000
};

LCDDisplay::LCDDisplay() : 
  lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS),
  currentState(MAIN_SCREEN),
  previousState(MAIN_SCREEN),
  selectedItem(0),
  scrollPosition(0),
  editMode(false),
  refreshNeeded(true),
  lastRefresh(0),
  lastButtonPress(millis()),
  backlightTimeout(30000), // 30 seconds timeout / Таймаут 30 секунд
  backlightOn(true),
  cursorVisible(false),
  lastCursorToggle(0)
{
  // Initialize menu items / Инициализация пунктов меню
  mainMenuItems[0] = "Temperature Settings";
  mainMenuItems[1] = "Humidity Settings";
  mainMenuItems[2] = "Light Settings";
  mainMenuItems[3] = "Time Settings";
  mainMenuItems[4] = "PID Settings";
  mainMenuItems[5] = "System Info";
  mainMenuItems[6] = "Reset System";
  mainMenuItems[7] = "Emergency Stop";
  mainMenuItems[8] = "Save to EEPROM";
  mainMenuItems[9] = "Back to Main";
  
  temperatureItems[0] = "Set Temperature";
  temperatureItems[1] = "PID: P Factor";
  temperatureItems[2] = "PID: I Factor";
  temperatureItems[3] = "PID: D Factor";
  
  humidityItems[0] = "Set Humidity";
  humidityItems[1] = "Back";
  
  lightItems[0] = "Light ON Time";
  lightItems[1] = "Light OFF Time";
  lightItems[2] = "Schedule ON/OFF";
  lightItems[3] = "Light Intensity";
  lightItems[4] = "Back";
  
  timeItems[0] = "Set Time";
  timeItems[1] = "Set Date";
  timeItems[2] = "Sync with RTC";
  timeItems[3] = "Back";
  
  saveConfirmItems[0] = "YES - Save";
  saveConfirmItems[1] = "NO - Cancel";
  saveConfirmItems[2] = "Back";
}

void LCDDisplay::begin() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Create special characters / Создаем специальные символы
  lcd.createChar(0, degreeSymbol);    // Degree symbol / Символ градуса
  lcd.createChar(1, arrowSymbol);     // Arrow symbol / Символ стрелки
  
  // Welcome message / Приветственное сообщение
  lcd.setCursor(0, 0);
  lcd.print("Climate Control");
  lcd.setCursor(0, 1);
  lcd.print("System v3.0");
  lcd.setCursor(0, 2);
  lcd.print("LCD Display Ready");
  delay(2000);
  
  lcd.clear();
  refreshNeeded = true;
}

void LCDDisplay::update() {
  updateBacklight();
  
  if (editMode) {
    // Blinking cursor in edit mode / Мигание курсора в режиме редактирования
    unsigned long currentTime = millis();
    if (currentTime - lastCursorToggle > 500) {
      cursorVisible = !cursorVisible;
      lastCursorToggle = currentTime;
      refreshNeeded = true;
    }
  }
  
  if (refreshNeeded || (millis() - lastRefresh > 500)) {
    refreshDisplay();
    lastRefresh = millis();
    refreshNeeded = false;
  }
}

void LCDDisplay::updateBacklight() {
  unsigned long currentTime = millis();
  if (backlightOn && currentTime - lastButtonPress > backlightTimeout) {
    lcd.noBacklight();
    backlightOn = false;
  }
}

void LCDDisplay::activateBacklight() {
  unsigned long currentTime = millis();
  lastButtonPress = currentTime;
  if (!backlightOn) {
    lcd.backlight();
    backlightOn = true;
  }
}

void LCDDisplay::refreshDisplay() {
  // Get system status from controller / Получаем статус системы от контроллера
  ClimateController& controller = ClimateController::getInstance();
  SystemStatus status = controller.getSystemStatus();
  
  PIDFactor pid = controller.getTemperaturePID();
  
  switch (currentState) {
    case MAIN_SCREEN:
      drawMainScreen(status);
      break;
      
    case MAIN_MENU:
      drawMainMenu();
      break;
      
    case TEMPERATURE_SETTINGS:
      drawTemperatureMenu(status, pid);
      break;
      
    case HUMIDITY_SETTINGS:
      drawHumidityMenu(status);
      break;
      
    case LIGHT_SETTINGS:
      drawLightMenu(status);
      break;
      
    case TIME_SETTINGS:
      drawTimeMenu(status);
      break;
      
    case SAVE_CONFIRM:
      drawSaveConfirm();
      break;
      
    case SYSTEM_INFO:
      drawSystemInfo(status);
      break;
  }
}

void LCDDisplay::drawMainScreen(const SystemStatus& status) {
  lcd.clear();
  
  // Line 1: Temperature / Температура
  lcd.setCursor(0, 0);
  lcd.print("T:");
  if (!isnan(status.currentTemperature)) {
    lcd.print(status.currentTemperature, 1);
    lcd.write(0);  // Символ градуса (специальный символ 0)
    lcd.print("C");
  } else {
    lcd.print("---");
  }
  
  lcd.setCursor(10, 0);
  lcd.print("H:");
  if (!isnan(status.currentHumidity)) {
    lcd.print(status.currentHumidity, 0);
    lcd.print("%");
  } else {
    lcd.print("---");
  }
  
  // Line 2: Light / Свет
  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(status.lightLevel);
  
  lcd.setCursor(8, 1);
  lcd.print("PWM:");
  lcd.print(status.pidOutput);
  
  // Line 3: Time and Status / Время и статус
  lcd.setCursor(0, 2);
  char timeBuffer[9];
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
           status.currentTime.hour, status.currentTime.minute, status.currentTime.second);
  lcd.print(timeBuffer);
  
  lcd.setCursor(11, 2);
  lcd.print(status.systemEnabled ? "ON " : "OFF");
  
  // Line 4: Menu hint / Подсказка меню
  lcd.setCursor(0, 3);
  lcd.print("Press BTN for menu");
}

void LCDDisplay::drawMainMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(1); // Стрелка выбора / Arrow for selection
  lcd.print(" ");
  lcd.print(mainMenuItems[selectedItem]);
  
  if (scrollPosition > 0) {
    lcd.setCursor(0, 1);
    lcd.print("  ");
    lcd.print(mainMenuItems[selectedItem - 1]);
  }
  
  if (selectedItem < 9) {
    int nextItem = selectedItem + 1;
    int row = (scrollPosition > 0) ? 2 : 1;
    lcd.setCursor(0, row);
    lcd.print("  ");
    lcd.print(mainMenuItems[nextItem]);
  }
  
  // Navigation hint / Подсказка навигации
  lcd.setCursor(0, 3);
  lcd.print("Rotate->Select  BTN->OK");
}

void LCDDisplay::drawTemperatureMenu(const SystemStatus& status, const PIDFactor& pid) {
  lcd.clear();
  
  switch (selectedItem) {
    case 0: // Set Temperature / Установка температуры
      lcd.setCursor(0, 0);
      lcd.print("Set Temperature:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      if (!isnan(status.currentTemperature)) {
        lcd.print(status.currentTemperature, 1);
        lcd.write(0);
        lcd.print("C");
      } else {
        lcd.print("---");
      }
      lcd.setCursor(0, 2);
      lcd.print("Target: ");
      lcd.print(status.targetTemperature, 1);
      lcd.write(0);
      lcd.print("C");
      break;
      
    case 1: // P Factor / Коэффициент P
      lcd.setCursor(0, 0);
      lcd.print("PID - P Factor:");
      lcd.setCursor(0, 1);
      lcd.print("Value: ");
      lcd.print(pid.Kp, 3);
      break;
      
    case 2: // I Factor / Коэффициент I
      lcd.setCursor(0, 0);
      lcd.print("PID - I Factor:");
      lcd.setCursor(0, 1);
      lcd.print("Value: ");
      lcd.print(pid.Ki, 3);
      break;
      
    case 3: // D Factor / Коэффициент D
      lcd.setCursor(0, 0);
      lcd.print("PID - D Factor:");
      lcd.setCursor(0, 1);
      lcd.print("Value: ");
      lcd.print(pid.Kd, 3);
      break;
  }
  
  // Navigation / Навигация
  lcd.setCursor(0, 3);
  if (editMode) {
    lcd.print("Rotate->Change  BTN->Save");
  } else {
    lcd.print("Rotate->Select  BTN->Edit");
  }
}

void LCDDisplay::drawHumidityMenu(const SystemStatus& status) {
  lcd.clear();
  
  if (selectedItem == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Set Humidity:");
    lcd.setCursor(0, 1);
    lcd.print("Current: ");
    if (!isnan(status.currentHumidity)) {
      lcd.print(status.currentHumidity, 0);
      lcd.print("%");
    } else {
      lcd.print("---");
    }
    lcd.setCursor(0, 2);
    lcd.print("Target: ");
    lcd.print(status.targetHumidity, 0);
    lcd.print("%");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Go back to");
    lcd.setCursor(0, 1);
    lcd.print("main menu");
  }
  
  lcd.setCursor(0, 3);
  if (editMode) {
    lcd.print("Rotate->Change  BTN->Save");
  } else {
    lcd.print("Rotate->Select  BTN->Edit");
  }
}

void LCDDisplay::drawLightMenu(const SystemStatus& status) {
  lcd.clear();
  
  ClimateController& controller = ClimateController::getInstance();
  LightScheduler& scheduler = controller.getLightScheduler();
  uint8_t onHour, onMinute, offHour, offMinute;
  scheduler.getLightOnTime(onHour, onMinute);
  scheduler.getLightOffTime(offHour, offMinute);
  
  switch (selectedItem) {
    case 0: // Light ON Time / Время включения света
      lcd.setCursor(0, 0);
      lcd.print("Light ON Time:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      lcd.print(onHour);
      lcd.print(":");
      if (onMinute < 10) lcd.print("0");
      lcd.print(onMinute);
      break;
      
    case 1: // Light OFF Time / Время выключения света
      lcd.setCursor(0, 0);
      lcd.print("Light OFF Time:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      lcd.print(offHour);
      lcd.print(":");
      if (offMinute < 10) lcd.print("0");
      lcd.print(offMinute);
      break;
      
    case 2: // Schedule ON/OFF / Расписание ВКЛ/ВЫКЛ
      lcd.setCursor(0, 0);
      lcd.print("Light Schedule:");
      lcd.setCursor(0, 1);
      lcd.print("Status: ");
      lcd.print(status.scheduleEnabled ? "ENABLED" : "DISABLED");
      break;
      
    case 3: // Light Intensity / Интенсивность света
      lcd.setCursor(0, 0);
      lcd.print("Light Intensity:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      lcd.print(status.lightLevel);
      break;
      
    case 4: // Back / Назад
      lcd.setCursor(0, 0);
      lcd.print("Go back to");
      lcd.setCursor(0, 1);
      lcd.print("main menu");
      break;
  }
  
  lcd.setCursor(0, 3);
  if (editMode) {
    lcd.print("Rotate->Change  BTN->Save");
  } else {
    lcd.print("Rotate->Select  BTN->Edit");
  }
}

void LCDDisplay::drawTimeMenu(const SystemStatus& status) {
  lcd.clear();
  
  switch (selectedItem) {
    case 0: // Set Time / Установка времени
      lcd.setCursor(0, 0);
      lcd.print("Set Time:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      char timeBuffer[9];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
               status.currentTime.hour, status.currentTime.minute, status.currentTime.second);
      lcd.print(timeBuffer);
      break;
      
    case 1: // Set Date / Установка даты
      lcd.setCursor(0, 0);
      lcd.print("Set Date:");
      lcd.setCursor(0, 1);
      lcd.print("Current: ");
      char dateBuffer[11];
      snprintf(dateBuffer, sizeof(dateBuffer), "%02d/%02d/%02d",
               status.currentTime.day, status.currentTime.month, status.currentTime.year);
      lcd.print(dateBuffer);
      break;
      
    case 2: // Sync with RTC / Синхронизация с RTC
      lcd.setCursor(0, 0);
      lcd.print("Sync with RTC:");
      lcd.setCursor(0, 1);
      lcd.print("Status: ");
      ClimateController& controller = ClimateController::getInstance();
      RTCManager& rtc = controller.getRTC();
      lcd.print(rtc.isRTCAvailable() ? "Available" : "Not Available");
      break;
      
    case 3: // Back / Назад
      lcd.setCursor(0, 0);
      lcd.print("Go back to");
      lcd.setCursor(0, 1);
      lcd.print("main menu");
      break;
  }
  
  lcd.setCursor(0, 3);
  if (editMode) {
    lcd.print("Rotate->Change  BTN->Save");
  } else {
    lcd.print("Rotate->Select  BTN->Edit");
  }
}

void LCDDisplay::drawSaveConfirm() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Save to EEPROM?");
  lcd.setCursor(0, 1);
  
  switch (selectedItem) {
    case 0:
      lcd.write(1);
      lcd.print(" YES - Save");
      lcd.setCursor(0, 2);
      lcd.print("  NO - Cancel");
      break;
    case 1:
      lcd.print("  YES - Save");
      lcd.setCursor(0, 2);
      lcd.write(1);
      lcd.print(" NO - Cancel");
      break;
    case 2:
      lcd.print("  YES - Save");
      lcd.setCursor(0, 2);
      lcd.print("  NO - Cancel");
      lcd.setCursor(0, 3);
      lcd.write(1);
      lcd.print(" Back");
      break;
  }
}

void LCDDisplay::drawSystemInfo(const SystemStatus& status) {
  lcd.clear();
  
  ClimateController& controller = ClimateController::getInstance();
  SensorReader& sensorReader = controller.getSensorReader();
  
  switch (selectedItem) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("System Status:");
      lcd.setCursor(0, 1);
      lcd.print("Errors: ");
      lcd.print(status.errorCount);
      lcd.setCursor(0, 2);
      lcd.print("Overheat: ");
      lcd.print(status.overheatProtection ? "YES" : "NO");
      break;
      
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Sensor Errors:");
      lcd.setCursor(0, 1);
      lcd.print("Count: ");
      lcd.print(sensorReader.getErrorCount());
      break;
      
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("RTC Status:");
      lcd.setCursor(0, 1);
      lcd.print("Available: ");
      RTCManager& rtc = controller.getRTC();
      lcd.print(rtc.isRTCAvailable() ? "YES" : "NO");
      break;
      
    case 3: // NEW: Add watchdog status / НОВОЕ: Добавить статус watchdog
      lcd.setCursor(0, 0);
      lcd.print("Watchdog Status:");
      lcd.setCursor(0, 1);
      lcd.print("Resets: ");
      // Use the new method / Используем новый метод
      lcd.print(controller.getWatchdogResetCount());
      break;
  }
  
  lcd.setCursor(0, 3);
  lcd.print("BTN->Back  Rotate->Scroll");
}

void LCDDisplay::handleEncoderRotation(int direction) {
  activateBacklight();
  
  if (editMode) {
    // In edit mode change values / В режиме редактирования меняем значения
    handleValueChange(direction);
  } else {
    // In navigation mode change selected item / В режиме навигации меняем выбранный пункт
    selectedItem += direction;
    
    // Limit range based on state / Ограничиваем диапазон в зависимости от состояния
    switch (currentState) {
      case MAIN_MENU:
        selectedItem = constrain(selectedItem, 0, 9);
        break;
      case TEMPERATURE_SETTINGS:
        selectedItem = constrain(selectedItem, 0, 3);
        break;
      case HUMIDITY_SETTINGS:
        selectedItem = constrain(selectedItem, 0, 1);
        break;
      case LIGHT_SETTINGS:
        selectedItem = constrain(selectedItem, 0, 4);
        break;
      case TIME_SETTINGS:
        selectedItem = constrain(selectedItem, 0, 3);
        break;
      case SAVE_CONFIRM:
        selectedItem = constrain(selectedItem, 0, 2);
        break;
      case SYSTEM_INFO:
        selectedItem = constrain(selectedItem, 0, 3); // Changed from 2 to 3 / Изменено с 2 на 3
        break;
      default:
        selectedItem = constrain(selectedItem, 0, 0);
        break;
    }
    
    refreshNeeded = true;
  }
}

void LCDDisplay::handleButtonPress() {
  activateBacklight();
  
  if (editMode) {
    // Exit edit mode / Выход из режима редактирования
    editMode = false;
    cursorVisible = false;
    refreshNeeded = true;
  } else {
    // Handle press based on state / Обработка нажатия в зависимости от состояния
    switch (currentState) {
      case MAIN_SCREEN:
        enterMainMenu();
        break;
        
      case MAIN_MENU:
        handleMainMenuSelection();
        break;
        
      case TEMPERATURE_SETTINGS:
      case HUMIDITY_SETTINGS:
      case LIGHT_SETTINGS:
      case TIME_SETTINGS:
        enterEditMode();
        break;
        
      case SAVE_CONFIRM:
        handleSaveConfirmation();
        break;
        
      case SYSTEM_INFO:
        // Allow going back from any item in SYSTEM_INFO
        // Разрешить возврат из любого пункта в SYSTEM_INFO
        goBack();
        break;
        
      default:
        goBack();
        break;
    }
  }
}

void LCDDisplay::enterMainMenu() {
  previousState = currentState;
  currentState = MAIN_MENU;
  selectedItem = 0;
  scrollPosition = 0;
  refreshNeeded = true;
}

void LCDDisplay::handleMainMenuSelection() {
  switch (selectedItem) {
    case 0: // Temperature Settings / Настройки температуры
      previousState = currentState;
      currentState = TEMPERATURE_SETTINGS;
      selectedItem = 0;
      break;
      
    case 1: // Humidity Settings / Настройки влажности
      previousState = currentState;
      currentState = HUMIDITY_SETTINGS;
      selectedItem = 0;
      break;
      
    case 2: // Light Settings / Настройки освещения
      previousState = currentState;
      currentState = LIGHT_SETTINGS;
      selectedItem = 0;
      break;
      
    case 3: // Time Settings / Настройки времени
      previousState = currentState;
      currentState = TIME_SETTINGS;
      selectedItem = 0;
      break;
      
    case 4: // PID Settings / Настройки PID
      previousState = currentState;
      currentState = TEMPERATURE_SETTINGS;
      selectedItem = 1; // Go directly to PID settings / Переходим сразу к настройкам PID
      break;
      
    case 5: // System Info / Информация о системе
      previousState = currentState;
      currentState = SYSTEM_INFO;
      selectedItem = 0;
      break;
      
    case 6: // Reset System / Сброс системы
      ClimateController::getInstance().resetSystem();
      goToMainScreen();
      break;
      
    case 7: // Emergency Stop / Аварийная остановка
      ClimateController::getInstance().emergencyStop();
      goToMainScreen();
      break;
      
    case 8: // Save to EEPROM / Сохранить в EEPROM
      previousState = currentState;
      currentState = SAVE_CONFIRM;
      selectedItem = 0;
      break;
      
    case 9: // Back to Main / Назад на главный экран
      goToMainScreen();
      break;
  }
  refreshNeeded = true;
}

void LCDDisplay::enterEditMode() {
  editMode = true;
  cursorVisible = true;
  lastCursorToggle = millis();
  refreshNeeded = true;
}

void LCDDisplay::handleValueChange(int direction) {
  ClimateController& controller = ClimateController::getInstance();
  
  switch (currentState) {
    case TEMPERATURE_SETTINGS:
      switch (selectedItem) {
        case 0: // Temperature
          controller.setTemperatureSetpoint(
            controller.getTemperatureSetpoint() + (direction * 0.5f)
          );
          break;
        case 1: // P Factor
          PIDFactor pid = controller.getTemperaturePID();
          pid.Kp += direction * 0.1;
          controller.setTemperaturePID(pid);
          break;
        case 2: // I Factor
          pid = controller.getTemperaturePID();
          pid.Ki += direction * 0.05;
          controller.setTemperaturePID(pid);
          break;
        case 3: // D Factor
          pid = controller.getTemperaturePID();
          pid.Kd += direction * 0.01;
          controller.setTemperaturePID(pid);
          break;
      }
      break;
      
    case HUMIDITY_SETTINGS:
      if (selectedItem == 0) {
        controller.setHumiditySetpoint(
          controller.getHumiditySetpoint() + direction
        );
      }
      break;
      
    case LIGHT_SETTINGS:
      LightScheduler& scheduler = controller.getLightScheduler();
      uint8_t hour, minute;
      
      switch (selectedItem) {
        case 0: // Light ON Time
          scheduler.getLightOnTime(hour, minute);
          hour = (hour + direction + 24) % 24;
          scheduler.setLightOnTime(hour, minute);
          break;
        case 1: // Light OFF Time
          scheduler.getLightOffTime(hour, minute);
          hour = (hour + direction + 24) % 24;
          scheduler.setLightOffTime(hour, minute);
          break;
        case 2: // Schedule ON/OFF
          scheduler.enableSchedule(!scheduler.isScheduleEnabled());
          break;
      }
      break;
      
    case TIME_SETTINGS:
      // Time setting would require more complex implementation
      // but for now we just update the display
      break;
  }
  
  refreshNeeded = true;
}

void LCDDisplay::handleSaveConfirmation() {
  switch (selectedItem) {
    case 0: // YES - Save / ДА - Сохранить
      ClimateController::getInstance().saveSettings();
      goToMainScreen();
      break;
    case 1: // NO - Cancel / НЕТ - Отмена
      goBack();
      break;
    case 2: // Back / Назад
      goBack();
      break;
  }
}

void LCDDisplay::goBack() {
  if (currentState == MAIN_MENU) {
    goToMainScreen();
  } else {
    currentState = previousState;
    selectedItem = 0;
    editMode = false;
    refreshNeeded = true;
  }
}

void LCDDisplay::goToMainScreen() {
  currentState = MAIN_SCREEN;
  editMode = false;
  refreshNeeded = true;
}