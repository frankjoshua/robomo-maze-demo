#ifndef ODOMETRY_H
#define ODOMETRY_H

class Odometry
{

public:
  struct Position
  {
    double x;
    double y;
    double theta;
  };

  Odometry(/* args */);
  ~Odometry();

  Position calculatePosition(double xLinearVelocity, double zAngularVelocity);
  Position calculatePositionWithYaw(double xLinearVelocity, double yaw);

  void reset();

private:
  Position position;
};

#endif