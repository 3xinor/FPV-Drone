#ifndef __PID_CONTROL__
#define __PID_CONTROL__

#include <stdint.h>
#include <stdbool.h>

/* Struct defines PID parameters used for throttle inputs */
typedef struct {
    uint8_t Kp; // Proportional Gain
    uint8_t Ki; // Proportional Integral
    uint8_t Kd; // Proportional Derivative
    uint8_t cycle_time_seconds; // The PID cycle time
    uint8_t i_limit; // Safety feature. Prevents the motors from infinitely spooling up
    float previous_error;
    float previous_i;
} PIDController_t;

void initPIDController(PIDController_t* PIDController, uint8_t Kp, uint8_t Ki, uint8_t Kd, uint8_t cycle_time_seconds, uint8_t i_limit);

void resetPID(PIDController_t* PIDController);

float calculatePID(PIDController_t* PIDController, float error);


#endif /* __PID_CONTROL__ */
