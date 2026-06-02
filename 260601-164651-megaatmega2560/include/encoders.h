#ifndef ENCODERS_H
#define ENCODERS_H

void setupEncoders();
void resetEncoders();
void resetForwardEncoders();
void resetSideEncoders();
float getForwardTravelCm();
float getSideTravelCm();

#endif
