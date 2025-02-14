#include "Encoder.h"

#include <Zumo32U4Encoders.h>

Zumo32U4Encoders zumoEncoders;
long encoderLeftLast = 0;
long encoderRightLast = 0;
long lastReadTimeLeft = 0;
long lastReadTimeRight = 0;
double radius = 0;
long tpr = 0;

Encoder::Encoder(double wheelRadius, long ticksPerRevolution)
{
  radius = wheelRadius;
  tpr = ticksPerRevolution;
}

void Encoder::readEncoders(Encoder::EncoderData &encoderData) {
    long readTime = millis();
    
    // Get encoder counts from Zumo
    long encoderLeftReading = zumoEncoders.getCountsLeft();
    long encoderRightReading = zumoEncoders.getCountsRight();
    
    // Store raw readings
    encoderData.reading.left = encoderLeftReading;
    encoderData.reading.right = encoderRightReading;
    
    // Calculate RPM for left encoder
    if (encoderLeftReading != 0) {
        long deltaTime = readTime - lastReadTimeLeft;
        if (deltaTime > 0) {
            long deltaLeft = encoderLeftReading - encoderLeftLast;
            lastReadTimeLeft = readTime;
            encoderLeftLast = encoderLeftReading;
            encoderData.rpm.left = (static_cast<double>(deltaLeft) * 60000.0) / (tpr * static_cast<double>(deltaTime));
        }
    }
    
    // Calculate RPM for right encoder
    if (encoderRightReading != 0) {
        long deltaTime = readTime - lastReadTimeRight;
        if (deltaTime > 0) {
            long deltaRight = encoderRightReading - encoderRightLast;
            lastReadTimeRight = readTime;
            encoderRightLast = encoderRightReading;
            encoderData.rpm.right = (static_cast<double>(deltaRight) * 60000.0) / (tpr * static_cast<double>(deltaTime));
        }
    }
}