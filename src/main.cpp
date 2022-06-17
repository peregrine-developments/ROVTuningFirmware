#include <Arduino.h>
#include <PID.hpp>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>

#include <Servo.h>

#include <Adafruit_BNO055.h>
#include <MS5837.h>

//#define DEBUG

// Controllers
float yaw = 0;
PID::PID yawController;
bool yawControlEnabled;
int16_t yawOutput;

float pitch = 0;
PID::PID pitchController;
bool pitchControlEnabled;
int16_t pitchOutput;

float roll = 0;
PID::PID rollController;
bool rollControlEnabled;
int16_t rollOutput;

float depth = 0;
PID::PID depthController;
bool depthControlEnabled;
int16_t depthOutput;

// Controller gain object
struct ControllerGains
{
    float p;
    float i;
    float d;
    float antiwindup;
};

// Thruster outputs
uint16_t thrusterFLValue = 1500;
uint16_t thrusterMLValue = 1500;
uint16_t thrusterRLValue = 1500;
uint16_t thrusterFRValue = 1500;
uint16_t thrusterMRValue = 1500;
uint16_t thrusterRRValue = 1500;

// Serial buffer
char serialBuffer[40];
uint8_t serialIndex = 0;

// Timing variables
unsigned long lastTelemetryUpdate = 0;
unsigned int telemetryDelay = 1000 / 10; // 20Hz telemetry   changed

unsigned long lastThrusterUpdate = 0;
unsigned int thrusterDelay = 1000 / 50; // 50Hz PWM

// Sensor variables
Adafruit_BNO055 bno;
MS5837 baro;

// Pins
uint8_t thrusterFLPin = 8;
uint8_t thrusterMLPin = 9;
uint8_t thrusterRLPin = 10;
uint8_t thrusterFRPin = 11;
uint8_t thrusterMRPin = 12;
uint8_t thrusterRRPin = 13;

// Servos
Servo thrusterFL;
Servo thrusterML;
Servo thrusterRL;
Servo thrusterFR;
Servo thrusterMR;
Servo thrusterRR;

// Safety
bool armed = false;

// Function prototypes
void loadGains();
void saveGains();
void processSerialCommand(char* buffer, uint8_t s);

void setup() {
    Serial.begin(57600);
    while(!Serial) delay(50);
    delay(500);

    Serial.println(F("/I/"));

#ifdef DEBUG
    Serial.println(F("Initializing connection"));
#endif

    // Controller initialization
    yawController.errorMode = PID::ErrorMode::CIRCULAR;

    // I2C initialization
    Wire.begin();
    Wire.setClock(400000);

    // Barometer initialization
    while(!baro.init())
    {
#ifdef DEBUG
        Serial.println(F("Barometer init failed!"));
#endif
        delay(2000);
    }
    baro.setModel(MS5837::MS5837_30BA);
    baro.setFluidDensity(997);

    // IMU initialization
    while(!bno.begin())
    {
#ifdef DEBUG
        Serial.println(F("IMU init failed!"));
#endif
        delay(2000);
    }
    bno.setExtCrystalUse(true);

    // PWM initialization
    thrusterFL.attach(thrusterFLPin);
    thrusterML.attach(thrusterMLPin);
    thrusterRL.attach(thrusterRLPin);
    thrusterFR.attach(thrusterFRPin);
    thrusterMR.attach(thrusterMRPin);
    thrusterRR.attach(thrusterRRPin);

#ifdef DEBUG
    Serial.println(F("Initialization complete!"));
#endif

    lastTelemetryUpdate = 
    lastThrusterUpdate = 
    millis();
}

void loop() {
    unsigned long loopTime = millis();

    while(Serial.available() > 0)
    {
        char c = Serial.read(); 

        if(c == '\n')
        {
            serialBuffer[serialIndex++] = '\0';
            processSerialCommand(serialBuffer, serialIndex);
            serialIndex = 0;
        }
        else
        {
            serialBuffer[serialIndex++] = c;
        }
    }

    // Handle controller & output updates
    if(loopTime > lastThrusterUpdate + thrusterDelay)
    {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        baro.read();

        yaw = euler.x();
        pitch = -euler.y();
        roll = euler.z();

        depth = baro.depth();

        // Controllers only need to be updated as fast as
        // the PWM outputs can be written, any faster simply
        // wastes clock cycles
        float dt = thrusterDelay / 1000.f;

        // Reset all thruster values
        thrusterFLValue = 1500;
        thrusterMLValue = 1500;
        thrusterRLValue = 1500;
        thrusterFRValue = 1500;
        thrusterMRValue = 1500;
        thrusterRRValue = 1500;

        // Update controllers & mix outputs
        if(yawControlEnabled)
        {
            yawController.update(yaw, dt);

            thrusterMLValue += (int)yawController.output;
            thrusterMRValue -= (int)yawController.output;
        }
        if(pitchControlEnabled)
        {
            pitchController.update(pitch, dt);

            thrusterFLValue += (int)pitchController.output;
            thrusterFRValue += (int)pitchController.output;
            thrusterRLValue -= (int)pitchController.output;
            thrusterRRValue -= (int)pitchController.output;
        }
        if(rollControlEnabled)
        {
            rollController.update(roll, dt);

            thrusterFLValue += (int)rollController.output;
            thrusterRLValue += (int)rollController.output;
            thrusterFRValue -= (int)rollController.output;
            thrusterRRValue -= (int)rollController.output;
        }
        if(depthControlEnabled)
        {
            depthController.update(depth, dt);

            thrusterFLValue -= (int)depthController.output;
            thrusterRLValue -= (int)depthController.output;
            thrusterFRValue -= (int)depthController.output;
            thrusterRRValue -= (int)depthController.output;
        }

        // Constrain outputs
        thrusterFLValue = constrain(thrusterFLValue, 1100, 1900);
        thrusterMLValue = constrain(thrusterMLValue, 1100, 1900);
        thrusterRLValue = constrain(thrusterRLValue, 1100, 1900);
        thrusterFRValue = constrain(thrusterFRValue, 1100, 1900);
        thrusterMRValue = constrain(thrusterMRValue, 1100, 1900);
        thrusterRRValue = constrain(thrusterRRValue, 1100, 1900);

        // Write to ESCs
        if(armed)
        {
            thrusterFL.writeMicroseconds(thrusterFLValue);
            thrusterML.writeMicroseconds(thrusterMLValue);
            thrusterRL.writeMicroseconds(thrusterRLValue);
            
            thrusterFR.writeMicroseconds(thrusterFRValue);
            thrusterMR.writeMicroseconds(thrusterMRValue);
            thrusterRR.writeMicroseconds(thrusterRRValue);
        }
        else
        {
            thrusterFL.writeMicroseconds(1500);
            thrusterML.writeMicroseconds(1500);
            thrusterRL.writeMicroseconds(1500);
            
            thrusterFR.writeMicroseconds(1500);
            thrusterMR.writeMicroseconds(1500);
            thrusterRR.writeMicroseconds(1500);
        }

        // Setup for next update
        lastThrusterUpdate += thrusterDelay;
    }

    // Handle telemetry sending
    if(loopTime > lastTelemetryUpdate + telemetryDelay)
    {
        Serial.print('/');
        Serial.print('T');
        Serial.print(yaw, 2); // Yaw
        Serial.print(',');
        Serial.print(pitch, 2); // Pitch
        Serial.print(',');
        Serial.print(roll, 2); // Roll
        Serial.print(',');
        Serial.print(depth, 2); // Depth
        Serial.print(',');
        Serial.print(thrusterFLValue); // FL thruster
        Serial.print(',');
        Serial.print(thrusterMLValue); // ML thruster
        Serial.print(',');
        Serial.print(thrusterRLValue); // RL thruster
        Serial.print(',');
        Serial.print(thrusterFRValue); // FR thruster
        Serial.print(',');
        Serial.print(thrusterMRValue); // MR thruster
        Serial.print(',');
        Serial.print(thrusterRRValue); // RR thruster
        Serial.println('/');

        lastTelemetryUpdate += telemetryDelay;
    }
}

void loadGains()
{
    // Load controller gains
    ControllerGains gainsLoader;

    EEPROM.get(4 + (sizeof(gainsLoader) * 0), gainsLoader);

    yawController.kp = gainsLoader.p;
    yawController.ki = gainsLoader.i;
    yawController.kd = gainsLoader.d;
    yawController.antiwindup = gainsLoader.antiwindup;

    EEPROM.get(4 + (sizeof(gainsLoader) * 1), gainsLoader);

    pitchController.kp = gainsLoader.p;
    pitchController.ki = gainsLoader.i;
    pitchController.kd = gainsLoader.d;
    pitchController.antiwindup = gainsLoader.antiwindup;

    EEPROM.get(4 + (sizeof(gainsLoader) * 2), gainsLoader);

    rollController.kp = gainsLoader.p;
    rollController.ki = gainsLoader.i;
    rollController.kd = gainsLoader.d;
    rollController.antiwindup = gainsLoader.antiwindup;

    EEPROM.get(4 + (sizeof(gainsLoader) * 3), gainsLoader);

    depthController.kp = gainsLoader.p;
    depthController.ki = gainsLoader.i;
    depthController.kd = gainsLoader.d;
    depthController.antiwindup = gainsLoader.antiwindup;

    // Enabled controllers
    yawControlEnabled = 
        !(yawController.kp == 0 && yawController.ki == 0 &&
          yawController.kd == 0 && yawController.antiwindup == 0);
    pitchControlEnabled = 
        !(pitchController.kp == 0 && pitchController.ki == 0 &&
          pitchController.kd == 0 && pitchController.antiwindup == 0);
    rollControlEnabled = 
        !(rollController.kp == 0 && rollController.ki == 0 &&
          rollController.kd == 0 && rollController.antiwindup == 0);
    depthControlEnabled = 
        !(depthController.kp == 0 && depthController.ki == 0 &&
          depthController.kd == 0 && depthController.antiwindup == 0);
}

void saveGains()
{
    // Save controller gains
    ControllerGains gainsSaver;

    gainsSaver.p = yawController.kp;
    gainsSaver.i = yawController.ki;
    gainsSaver.d = yawController.kd;
    gainsSaver.antiwindup = yawController.antiwindup;

    EEPROM.put(4 + (sizeof(gainsSaver) * 0), gainsSaver);

    gainsSaver.p = pitchController.kp;
    gainsSaver.i = pitchController.ki;
    gainsSaver.d = pitchController.kd;
    gainsSaver.antiwindup = pitchController.antiwindup;

    EEPROM.put(4 + (sizeof(gainsSaver) * 1), gainsSaver);

    gainsSaver.p = rollController.kp;
    gainsSaver.i = rollController.ki;
    gainsSaver.d = rollController.kd;
    gainsSaver.antiwindup = rollController.antiwindup;

    EEPROM.put(4 + (sizeof(gainsSaver) * 2), gainsSaver);

    gainsSaver.p = depthController.kp;
    gainsSaver.i = depthController.ki;
    gainsSaver.d = depthController.kd;
    gainsSaver.antiwindup = depthController.antiwindup;

    EEPROM.put(4 + (sizeof(gainsSaver) * 3), gainsSaver);
}

/*
* Serial command strings:
*
* H - ping connection
*
* Yp,i,d,a - set yaw controller gains
* Pp,i,d,a - set pitch controller gains
* Rp,i,d,a - set roll controller gains
* Dp,i,d,a - set depth controller gains
* (all P, I, D, antiwindup gains are sent in x100 scale,
* i.e 10000 = 100.00)
*
* Ey,p,r,d - load setpoints (also x100 scale)
* 
* G - report currently loaded gains
*
* L - load gain set from EEPROM
* S - save gain set to EEPROM
*
* A - arm vehicle
* F - safe vehicle
*/
void processSerialCommand(char* buffer, uint8_t s)
{
#ifdef DEBUG
    Serial.println(F("Processing serial command..."));
    Serial.print(F("Buffer: "));
    Serial.println(buffer);
#endif
    char commandType = buffer[0];
    char* subBuf = buffer + 1;
#ifdef DEBUG
    Serial.print(F("Command type is "));
    Serial.println(commandType);
#endif

    switch(commandType)
    {
        case 'H':
        {
#ifdef DEBUG
            Serial.println("Ping");
#endif      
            Serial.println("OK");
            break;
        }
        case 'E':
        {
#ifdef DEBUG
            Serial.println("Command is setpoints");
#endif
            int syRaw;
            int spRaw;
            int srRaw;
            int sdRaw;

            float sy;
            float sp;
            float sr;
            float sd;

            syRaw = atoi(strtok(subBuf, ","));
            spRaw = atoi(strtok(NULL, ","));
            srRaw = atoi(strtok(NULL, ","));
            sdRaw = atoi(strtok(NULL, ","));

            sy = (float)syRaw / 100.f;
            sp = (float)spRaw / 100.f;
            sr = (float)srRaw / 100.f;
            sd = (float)sdRaw / 100.f;

#ifdef DEBUG
            Serial.println(sy);
            Serial.println(sp);
            Serial.println(sr);
            Serial.println(sd);
#endif

            yawController.setpoint = sy;
            pitchController.setpoint = sp;
            rollController.setpoint = sr;
            depthController.setpoint = sd;

            Serial.println("OK");
            break;
        }
        case 'Y':
        case 'P':
        case 'R':
        case 'D':
        {
#ifdef DEBUG
            Serial.println("Command is set gains");
#endif      
            unsigned int pRaw;
            unsigned int iRaw;
            unsigned int dRaw;
            unsigned int aRaw;

            float p;
            float i;
            float d;
            float antiwindup;

            pRaw = atoi(strtok(subBuf, ","));
            iRaw = atoi(strtok(NULL, ","));
            dRaw = atoi(strtok(NULL, ","));
            aRaw = atoi(strtok(NULL, ","));

            p = (float)pRaw / 100.f;
            i = (float)iRaw / 100.f;
            d = (float)dRaw / 100.f;
            antiwindup = (float)aRaw / 100.f;

            bool enabled = true;

            if(pRaw == 0 && iRaw == 0 && dRaw == 0 && aRaw == 0)
            {
                enabled = false;
            }

            switch(buffer[0])
            {
                case 'Y':
                    yawController.kp = p;
                    yawController.ki = i;
                    yawController.kd = d;
                    yawController.antiwindup = antiwindup;

                    yawControlEnabled = enabled;
#ifdef DEBUG
                    Serial.println("Wrote yaw controller");
#endif
                    break;
                case 'P':
                    pitchController.kp = p;
                    pitchController.ki = i;
                    pitchController.kd = d;
                    pitchController.antiwindup = antiwindup;

                    pitchControlEnabled = enabled;
#ifdef DEBUG
                    Serial.println("Wrote pitch controller");
#endif
                    break;
                case 'R':
                    rollController.kp = p;
                    rollController.ki = i;
                    rollController.kd = d;
                    rollController.antiwindup = antiwindup;

                    rollControlEnabled = enabled;
#ifdef DEBUG
                    Serial.println("Wrote roll controller");
#endif
                    break;
                case 'D':
                    depthController.kp = p;
                    depthController.ki = i;
                    depthController.kd = d;
                    depthController.antiwindup = antiwindup;

                    depthControlEnabled = enabled;
#ifdef DEBUG
                    Serial.println("Wrote depth controller");
#endif
                    break;
            }
#ifdef DEBUG
            Serial.println(p);
            Serial.println(i);
            Serial.println(d);
            Serial.println(antiwindup);
#endif
            Serial.println("OK");
            break;
        }
        case 'G':
        {
#ifdef DEBUG
            Serial.println("Command is report gains");
#endif
            Serial.print('/');
            Serial.print('Y');
            if(yawControlEnabled)
            {
                Serial.print((unsigned int)(yawController.kp * 100));
                Serial.print(',');
                Serial.print((unsigned int)(yawController.ki * 100));
                Serial.print(',');
                Serial.print((unsigned int)(yawController.kd * 100));
                Serial.print(',');
                Serial.print((unsigned int)(yawController.antiwindup * 100));
            }
            else
            {
                Serial.print('D');
            }
            Serial.println('/');

            Serial.print('/');
            Serial.print('P');
            if(pitchControlEnabled)
            {
                Serial.print((unsigned int)(pitchController.kp * 100));
                Serial.print(',');
                Serial.print((unsigned int)(pitchController.ki * 100));
                Serial.print(',');
                Serial.print((unsigned int)(pitchController.kd * 100));
                Serial.print(',');
                Serial.print((unsigned int)(pitchController.antiwindup * 100));
            }
            else
            {
                Serial.print('D');
            }
            Serial.println('/');
            
            Serial.print('/');
            Serial.print('R');
            if(rollControlEnabled)
            {
                Serial.print((unsigned int)(rollController.kp * 100));
                Serial.print(',');
                Serial.print((unsigned int)(rollController.ki * 100));
                Serial.print(',');
                Serial.print((unsigned int)(rollController.kd * 100));
                Serial.print(',');
                Serial.print((unsigned int)(rollController.antiwindup * 100));
            }
            else
            {
                Serial.print('D');
            }
            Serial.println('/');

            Serial.print('/');
            Serial.print('D');
            if(depthControlEnabled)
            {
                Serial.print((unsigned int)(depthController.kp * 100));
                Serial.print(',');
                Serial.print((unsigned int)(depthController.ki * 100));
                Serial.print(',');
                Serial.print((unsigned int)(depthController.kd * 100));
                Serial.print(',');
                Serial.print((unsigned int)(depthController.antiwindup * 100));
            }
            else
            {
                Serial.print('D');
            }
            Serial.println('/');
            
            Serial.println("OK");

            break;
        }
        case 'L':
        {
#ifdef DEBUG
            Serial.println("Command is load gains");
#endif
            loadGains();
            Serial.println("OK");
            break;
        }
        case 'S':
        {
#ifdef DEBUG
            Serial.println("Command is save gains");
#endif
            saveGains();
            Serial.println("OK");
            break;
        }
        case 'A':
        {
#ifdef DEBUG
            Serial.println("Command is arm");
#endif
            armed = true;
            Serial.println("/A/\nOK");
            break;
        }
        case 'F':
        {
#ifdef DEBUG
            Serial.println("Command is disarm");
#endif
            armed = false;
            Serial.println("/F/\nOK");
            break;
        }
        default:
        {
#ifdef DEBUG
            Serial.println("Unknown command");
#endif
            break;
        }
    }
}