#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float wheel_diameter,
                       float wheels_x_distance, float wheels_y_distance) : base_platform(robot_base),
                                                                           max_rpm_(motor_max_rpm),
                                                                           wheels_x_distance_(base_platform == DIFFERENTIAL_DRIVE ? 0 : wheels_x_distance),
                                                                           wheels_y_distance_(wheels_y_distance),
                                                                           wheel_circumference_(PI * wheel_diameter),
                                                                           total_wheels_(getTotalWheels(robot_base))
{
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{
  float linear_vel_x_mins;
  float linear_vel_y_mins;
  float angular_vel_z_mins;
  float tangential_vel;
  float x_rpm;
  float y_rpm;
  float tan_rpm;

  //convert m/s to m/min
  linear_vel_x_mins = linear_x * 60;
  linear_vel_y_mins = linear_y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins = angular_z * 60;

  tangential_vel = angular_vel_z_mins * ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2));

  x_rpm = linear_vel_x_mins / wheel_circumference_;
  y_rpm = linear_vel_y_mins / wheel_circumference_;
  tan_rpm = tangential_vel / wheel_circumference_;

  Kinematics::rpm rpm;

  //calculate for the target motor RPM and direction
  //front-left motor
  rpm.motor1 = x_rpm - y_rpm - tan_rpm;
  rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

  //front-right motor
  rpm.motor2 = x_rpm + y_rpm + tan_rpm;
  rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

  //rear-left motor
  rpm.motor3 = x_rpm + y_rpm - tan_rpm;
  rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

  //rear-right motor
  rpm.motor4 = x_rpm - y_rpm + tan_rpm;
  rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

  return rpm;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
  Kinematics::rpm rpm;

  if (base_platform == DIFFERENTIAL_DRIVE || base_platform == SKID_STEER)
  {
    rpm = calculateRPM(linear_x, 0.0, angular_z);
  }
  else if (base_platform == ACKERMANN || base_platform == ACKERMANN1)
  {
    rpm = calculateRPM(linear_x, 0.0, 0.0);
  }
  else if (base_platform == MECANUM)
  {
    rpm = calculateRPM(linear_x, linear_y, angular_z);
  }

  return rpm;
}

Kinematics::velocities Kinematics::getVelocities(float steering_angle, int rpm1, int rpm2)
{
  Kinematics::velocities vel;
  float average_rps_x;

  //convert average revolutions per minute to revolutions per second
  average_rps_x = ((float)(rpm1 + rpm2) / total_wheels_) / 60; // RPM
  vel.linear_x = average_rps_x * wheel_circumference_;         // m/s

  vel.linear_y = 0.0;

  //http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
  vel.angular_z = (vel.linear_x * tan(steering_angle)) / wheels_x_distance_;

  return vel;
}

Kinematics::velocities Kinematics::getVelocities(velocities &vel, float leftRpm, float rightRpm, float rpm3, float rpm4)
{
  float average_rps_x;
  float average_rps_y;
  float average_rps_a;

  //convert average revolutions per minute to revolutions per second
  average_rps_x = ((float)(leftRpm + rightRpm + rpm3 + rpm4) / total_wheels_) / 60; // RPM
  vel.linear_x = average_rps_x * wheel_circumference_;                       // m/s

  //convert average revolutions per minute in y axis to revolutions per second
  average_rps_y = ((float)(-leftRpm + rightRpm + rpm3 - rpm4) / total_wheels_) / 60; // RPM
  if (base_platform == MECANUM)
    vel.linear_y = average_rps_y * wheel_circumference_; // m/s
  else
    vel.linear_y = 0;

  //convert average revolutions per minute to revolutions per second
  average_rps_a = -(leftRpm - rightRpm + rpm3 - rpm4) / total_wheels_ / 60;
  vel.angular_z = (average_rps_a * wheel_circumference_) / ((wheels_x_distance_ / 2) + (wheels_y_distance_ / 2)); //  rad/s

  return vel;
}

// Convention:
// - Left motor is motor1
// - Right motor is motor2
// For differential drive, positive RPM:
// - Left wheel forward = positive
// - Right wheel forward = positive

// Removed duplicate definition of getVelocities

int Kinematics::getTotalWheels(base robot_base)
{
  switch (robot_base)
  {
  case DIFFERENTIAL_DRIVE:
    return 2;
  case ACKERMANN:
    return 2;
  case ACKERMANN1:
    return 1;
  case SKID_STEER:
    return 4;
  case MECANUM:
    return 4;
  default:
    return 2;
  }
}

Kinematics::velocities Kinematics::getVelocitiesFromIMU(float acceleration_x, float acceleration_y, float gyro_z)
{
    Kinematics::velocities vel;

    // Convert accelerometer data to linear velocities (m/s)
    // Accelerometer data is in g's, multiply by 9.81 to get m/s^2
    vel.linear_x = acceleration_x * 9.81;
    vel.linear_y = acceleration_y * 9.81;

    // Gyroscope data is already in rad/s from IMU class
    vel.angular_z = gyro_z;

    return vel;
}

void Kinematics::fuseVelocities(float alpha, velocities& vel1, velocities& vel2, velocities& fused) 
{
    // Constrain alpha between 0 and 1
    float filtered_alpha = constrain(alpha, 0.0f, 1.0f);
    
    // Complementary filter:
    // vel1 (typically encoder-based) weighted by (1-alpha)
    // vel2 (typically IMU-based) weighted by alpha
    fused.linear_x = (1.0f - filtered_alpha) * vel1.linear_x + filtered_alpha * vel2.linear_x;
    fused.linear_y = (1.0f - filtered_alpha) * vel1.linear_y + filtered_alpha * vel2.linear_y;
    fused.angular_z = (1.0f - filtered_alpha) * vel1.angular_z + filtered_alpha * vel2.angular_z;
}

void Kinematics::fuseIMU(float acc_x, float acc_y, float acc_z,
                        float gyro_x, float gyro_y, float gyro_z,
                        float mag_x, float mag_y, float mag_z,
                        velocities* vel)
{    
    // Print raw input values
    // Serial.println("Raw IMU Values:");
    // Serial.print("Acc (x,y,z): "); 
    // Serial.print(acc_x); Serial.print(", ");
    // Serial.print(acc_y); Serial.print(", ");
    // Serial.println(acc_z);
    
    // Serial.print("Gyro (x,y,z): ");
    // Serial.print(gyro_x); Serial.print(", ");
    // Serial.print(gyro_y); Serial.print(", ");
    // Serial.println(gyro_z);

    // Convert and apply scaling
    float acc_scale = 0.000061f * 9.81f;
    vel->linear_x = acc_x * acc_scale;
    vel->linear_y = acc_y * acc_scale;

    float gyro_scale = 0.00875f * (PI / 180.0f);
    vel->angular_z = gyro_z * gyro_scale;

    // Print converted values
    // Serial.println("Converted Values:");
    // Serial.print("Linear (x,y): ");
    // Serial.print(vel->linear_x); Serial.print(", ");
    // Serial.println(vel->linear_y);
    // Serial.print("Angular z: ");
    // Serial.println(vel->angular_z);
    // Serial.println("-------------------");
}



