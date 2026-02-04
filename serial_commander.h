/*
 * Serial communication and command interface class
 * Класс интерфейса последовательной связи и команд
 */

#ifndef SERIAL_COMMANDER_H
#define SERIAL_COMMANDER_H

#include <Arduino.h>
#include "structures.h"

class RTCManager;  // Forward declaration

class SerialCommander {
private:
  String inputBuffer;
  unsigned long lastCommandTime;
  bool echoEnabled;
  
public:
  SerialCommander() : 
    lastCommandTime(0),
    echoEnabled(true)
  { 
    inputBuffer.reserve(64); // Reserve buffer to avoid fragmentation / Резервируем буфер чтобы избежать фрагментации
  }
  
  void begin(unsigned long baudRate = 9600);
  
  void printWelcomeMessage();
  
  void printHelp();
  
  bool checkForCommand();
  
  void processCommand(String& cmd);
  
  // Pointer to external functions for callbacks / Указатели на внешние функции для обратных вызовов
  void (*onStatusRequest)();
  void (*onTimeRequest)();
  void (*onSetTime)(String timeStr);
  void (*onSetDate)(String dateStr);
  void (*onSetLightOnTime)(String timeStr);
  void (*onSetLightOffTime)(String timeStr);
  void (*onSetLightSchedule)(bool enabled);
  void (*onSaveRequest)();
  void (*onLoadRequest)();
  void (*onResetRequest)();
  void (*onEmergencyStop)();
  void (*onTemperatureSet)(float temp);
  void (*onHumiditySet)(float humid);
  void (*onWatchdogTest)();
  
  void sendStatus(float temp, float targetTemp, float humid, float targetHumid, 
                  int light, bool heaterOn, bool humidifierOn, int errors,
                  bool lightOn, bool scheduleEnabled);
  
  void sendMessage(const String& msg);
  
  void sendTime(const DateTime& time);
};

#endif // SERIAL_COMMANDER_H