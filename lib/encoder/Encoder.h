#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

class Encoder
{
public:
  
  struct EncoderReading
  {
    long left;
    long right;
  };

  struct RPM
  {
    // Meters per second
    double left;
    double right;
    double testLeft;
    double testRight;
  };

  struct EncoderData
  {
    EncoderReading reading;
    RPM rpm;
  };

  Encoder(double wheelRadius, long ticksPerRevolution);

  void readEncoders(Encoder::EncoderData &rpm);

private:
  double wheelRadius;
  long ticksPerRevolution;
  
};

#endif
