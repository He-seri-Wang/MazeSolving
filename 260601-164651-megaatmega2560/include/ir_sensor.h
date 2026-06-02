#ifndef IR_SENSOR_H
#define IR_SENSOR_H

void setupIrSensor();
bool updateIrSensor();
float getGapDistanceCm();
float getStopDistanceCm();
float getRawVoltage();

#endif
