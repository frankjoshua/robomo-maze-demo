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
    // Serial.print("Left: ");
    // Serial.print(encoderLeftReading);
    // Serial.print(" Right: ");
    // Serial.println(encoderRightReading);
    
    // Store raw readings
    encoderData.reading.left = encoderLeftReading;
    encoderData.reading.right = encoderRightReading;
    
    // Calculate RPM for left encoder
    long deltaTime = readTime - lastReadTimeLeft;     // Always compute deltaTime
    long deltaLeft = encoderLeftReading - encoderLeftLast;  // Calculate change in counts

    // Handle overflow for left encoder
    if (deltaLeft < -32768) {
        deltaLeft += 65536;
    } else if (deltaLeft > 32767) {
        deltaLeft -= 65536;
    }

    // Update timing and reading variables every cycle
    lastReadTimeLeft = readTime;
    encoderLeftLast = encoderLeftReading;

    if (deltaTime > 0) {
        encoderData.rpm.left = (static_cast<double>(deltaLeft) * 60000.0) / (tpr * static_cast<double>(deltaTime));
    }
    
    // Calculate RPM for right encoder
    long deltaTimeRight = readTime - lastReadTimeRight;     
    long deltaRight = encoderRightReading - encoderRightLast;  

    // Handle overflow for right encoder
    if (deltaRight < -32768) {
        deltaRight += 65536;
    } else if (deltaRight > 32767) {
        deltaRight -= 65536;
    }

    // Update timing and reading variables every cycle
    lastReadTimeRight = readTime;
    encoderRightLast = encoderRightReading;

    if (deltaTimeRight > 0) {
        encoderData.rpm.right = (static_cast<double>(deltaRight) * 60000.0) / (tpr * static_cast<double>(deltaTimeRight));
    }
}
