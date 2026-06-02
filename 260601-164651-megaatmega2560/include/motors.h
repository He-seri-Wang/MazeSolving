#ifndef MOTORS_H
#define MOTORS_H

void setupMotors();
void stopForwardBackMotor();
void stopLeftRightMotor();
void stopAllMotors();

void moveForward();
void moveBackward();
void moveRight();
void moveLeft();
void moveSideByDirection(int direction);

void applyBrake(unsigned long duration);

#endif
