#pragma once

#include <Arduino.h>

namespace PID
{
    enum class ErrorMode
    {
        LINEAR,
        CIRCULAR
    };

    class PID
    {
    public:
        float kp;
        float ki;
        float kd;

        float setpoint;
        float antiwindup;

        float output;

        ErrorMode errorMode = ErrorMode::LINEAR;

        PID()                                    : kp(0),    ki(0),  kd(0),  setpoint(0),    antiwindup(0) { }
        PID(float p, float i, float d)           : kp(p),    ki(i),  kd(d),  setpoint(0),    antiwindup(0) { }
        PID(float p, float i, float d, float s)  : kp(p),    ki(i),  kd(d),  setpoint(s),    antiwindup(0) { }
        PID(float p, float i, float d, float s, float a) : kp(p), ki(i), kd(d), setpoint(s), antiwindup(a) { }

        float update(float pv, float dt);
    private:
        float lastError = 0;
        float integrator = 0;
    };
}

