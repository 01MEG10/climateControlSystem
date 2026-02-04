/*
 * Enhanced PID controller class
 * Улучшенный класс PID контроллера
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "structures.h"

class PIDController {
private:
  double Kp, Ki, Kd;
  double prevError;
  double integral;
  double outputMin, outputMax;
  unsigned long prevTime;
  int lastOutput;
  bool firstRun;
  double maxIntegral;
  
  void updateMaxIntegral() {
    if (fabs(Ki) > 0.0001) {
      maxIntegral = fabs((outputMax - outputMin) / Ki) * 0.8;
    } else {
      maxIntegral = 1000.0;
    }
  }
  
public:
  PIDController(const PIDFactor& factors = PIDFactor()) {
    setFactors(factors);
    firstRun = true;
    lastOutput = 0;
  }
  
  double compute(double setpoint, double measuredValue) {
    if (firstRun) {
      prevTime = millis();
      prevError = setpoint - measuredValue;
      integral = 0;
      firstRun = false;
      lastOutput = 0;
      return lastOutput;
    }
    
    unsigned long currentTime = millis();
    double deltaTime = (currentTime - prevTime) / 1000.0;
    
    if (deltaTime < 0.001 || deltaTime > 10.0) {
      prevTime = currentTime;
      return lastOutput;
    }
    
    double error = setpoint - measuredValue;
    double proportional = Kp * error;
    
    if (fabs(Ki) > 0.0001) {
      integral += error * deltaTime;
      
      if (integral > maxIntegral) integral = maxIntegral;
      if (integral < -maxIntegral) integral = -maxIntegral;
    }
    double integralTerm = Ki * integral;
    
    double derivative = (deltaTime > 0.001) ? (error - prevError) / deltaTime : 0;
    double derivativeTerm = Kd * derivative;
    
    double output = proportional + integralTerm + derivativeTerm;
    output = constrain(output, outputMin, outputMax);
    
    prevError = error;
    prevTime = currentTime;
    lastOutput = (int)output;
    
    return output;
  }
  
  void setFactors(const PIDFactor& factors) {
    Kp = constrain(factors.Kp, 0.0, 100.0);
    Ki = constrain(factors.Ki, 0.0, 10.0);
    Kd = constrain(factors.Kd, 0.0, 5.0);
    outputMin = constrain(factors.outputMin, 0, 255);
    outputMax = constrain(factors.outputMax, 0, 255);
    updateMaxIntegral();
    reset();
  }
  
  void setKp(double value) { 
    Kp = constrain(value, 0.0, 100.0); 
    resetIntegral(); 
  }
  
  void setKi(double value) { 
    Ki = constrain(value, 0.0, 10.0); 
    updateMaxIntegral();
    resetIntegral();
  }
  
  void setKd(double value) { 
    Kd = constrain(value, 0.0, 5.0); 
    resetIntegral();
  }
  
  PIDFactor getFactors() const {
    PIDFactor factors;
    factors.Kp = Kp;
    factors.Ki = Ki;
    factors.Kd = Kd;
    factors.outputMin = outputMin;
    factors.outputMax = outputMax;
    return factors;
  }
  
  void reset() {
    prevError = 0;
    integral = 0;
    prevTime = millis();
    firstRun = true;
  }
  
  void resetIntegral() {
    integral = 0;
  }
  
  double getKp() const { return Kp; }
  double getKi() const { return Ki; }
  double getKd() const { return Kd; }
  int getLastOutput() const { return lastOutput; }
  double getIntegral() const { return integral; }
};

#endif // PID_CONTROLLER_H