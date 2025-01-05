
#include "PID_Control.h"

void initPIDController(PIDController_t* PIDController, uint8_t Kp, uint8_t Ki, uint8_t Kd, uint8_t cycle_time_seconds, uint8_t i_limit) {
    // Initialize struct with specified settings
    PIDController->Kp = Kp;
    PIDController->Ki = Ki;
    PIDController->Kd = Kd;
    PIDController->cycle_time_seconds = cycle_time_seconds; // Operation time of system lets set to 250Hz(0.004s)
    PIDController->i_limit = i_limit;

    // state variables
    PIDController->previous_error = 0.0;
    PIDController->previous_i = 0.0;
}
// Performs PID calculation where the actual and setpoint relative postions are provided and the a throttle output is given in response
float calculatePID(PIDController_t* PIDController, float error) {

    // P Term
    float pTerm = error * PIDController->Kp; 
    // I Term
    float iTerm = PIDController->previous_i + (error * PIDController->Ki * PIDController->cycle_time_seconds); 
    if ( iTerm > PIDController->i_limit){iTerm = PIDController->i_limit;}
    if (iTerm < (-1 * PIDController->i_limit)){iTerm = (-1 * PIDController->i_limit);}
    // D Term
    float dTerm = PIDController->Kd * (error - PIDController->previous_error) / PIDController->cycle_time_seconds;

    // set state variables for next iteration
    PIDController->previous_error = error;
    PIDController->previous_i = iTerm;

    return pTerm + iTerm + dTerm;
}

// Resets state variables
void resetPID(PIDController_t* PIDController) {
    PIDController->previous_error = 0.0;
    PIDController->previous_i = 0.0;
}

