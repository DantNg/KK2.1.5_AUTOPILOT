#include <stdint.h>
/*
rx channel raw value 200-1800
channel[0] : ROLL
channel[1] : PITCH
channel[2] : THROTTLE
channel[3] : YAW
channel[4] : CH5
channel[5] : CH6
*/
int16_t channel[18] = {0};
// Published values for SG90 servos; adjust if needed
int minUs = 1000;
int maxUs = 2000;
