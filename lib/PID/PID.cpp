#include <PID.hpp>

float PID::PID::update(float pv, float dt)
{
    float error = setpoint - pv;

    // Convert 0-360 to -180-180 for circular measurements
    if(errorMode == ErrorMode::CIRCULAR)
    {
        if(error >= 180)
        {
            error -= 360;
        }
        else if(error < -180)
        {
            error += 360;
        }
    }

    integrator += pv * dt;
    if(antiwindup != 0)
    {
        if(integrator > antiwindup) integrator = antiwindup;
        if(integrator < -antiwindup) integrator = -antiwindup;
    }

    float derivative = (error - lastError) / dt;
    lastError = error;

    output = (kp * error)
           + (ki * integrator)
           + (kd * derivative);

    return output;
}