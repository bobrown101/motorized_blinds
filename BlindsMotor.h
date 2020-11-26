#ifndef BLINDS_MOTOR_H
#define BLINDS_MOTOR_H
#include "Arduino.h"
#include "PIDController.h"


class BlindsMotor {
  public:
    BlindsMotor();
    void setupMotor(int PIN_ENCODER_POS, int PIN_ENCODER_NEG, int PIN_MOTOR_POS, int PIN_MOTOR_NEG);
    void goToPosition(int DESIRED_POSITION);
    int getCurrentPosition();
  private:
    int pinEncoderPos;
    int pinEncoderNeg;
    int pinMotorPos;
    int pinMotorNeg;
    PIDController pidController;
    volatile long currentEncoderPosition;
    int minDriveValue;
    long pidResult;
    long desiredPosition;

    void turnMotor(int power, int posPin, int negPin);
    void encoderInterrupt(void);
    
    
};

#endif
