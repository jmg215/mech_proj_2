//Jon Grote

//definitely need the gamepad module
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_TERMINAL_MODULE
#define INCLUDE_PINMONITOR_MODULE
#define CUSTOM_SETTINGS
#define INCLUDE_LEDCONTROL_MODULE

//must have the dabble library to communicate through the application to the blutooth
#include <Dabble.h>
#include <QTRSensors.h>
#include <stdlib.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

bool gStartMeasuring = false;

//Left wheel direction
const int LDIR = 41;
//right wheel direction
const int RDIR = 42;
//left pwm
const int LPWM = 45;
//right pwm
const int RPWM = 46;
// era pin
const int era = 18;
//ela pin
const int ela = 19;
//the dutycycle for all motor inputs
//want 70
float dutyCycle = 255 * (82.0 / 100.00); // make greater than 1 and watch for weird behavior
//use volatile to indicate to the compiler that the value of these variables may change without its awarness
volatile long eracount = 0;
volatile long elacount = 0;

//here we go
//this is the best for half speed
//double KpR = 0.09, KiR = 0.03, KdR = 1.1;

/*
another core part of the PID control setup, these are the proportional, integral, and derivative constants */
double KpR = 0.09, KiR = 0.03, KdR = 1.1;
//double KpR = 0.009, KiR = 0.040, KdR = 0.0007795;
int lastError = 0;

//int error;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

    attachInterrupt(digitalPinToInterrupt(ela), leftwheel, CHANGE);
    //setup pin 18 to trigger an ISR when a change is detected
    attachInterrupt(digitalPinToInterrupt(era), rightwheel, CHANGE);

    //both the inputs for era and ela are INPUT_PULLUP
    pinMode(era, INPUT_PULLUP);
    pinMode(ela, INPUT_PULLUP);

    //set up pinModes here
    //much of this adapted from ex5.
    pinMode(LDIR, OUTPUT);
    pinMode(RDIR, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);

    //being Serial communication between the mega and computer and the mega and the Blutooth using Dabble library
    Serial.begin(115200);
    Dabble.begin(9600);
} //######### end setup routine ##############

void loop()
{

    Dabble.processInput();

    if (GamePad.isSelectPressed() == true)
    {
        CalibrateSensorArray();
    }
    if (GamePad.isStartPressed() == true)
    {
        gStartMeasuring = true;
    }
    else if (GamePad.isCrossPressed() == true)
    {
        gStartMeasuring = false;
    }
    // else // w/ this line commented out, robot went on its own without dabble.cross()
    // {
    //     gStartMeasuring = false;
    // }
    if (gStartMeasuring == true)
    {
        // read calibrated sensor values and obtain a measure of the line position
        // from 0 to 7000 (for a white line, use readLineWhite() instead)
        uint16_t position = qtr.readLineBlack(sensorValues);

        /*
        this is the core of the PID control funtionality. set point is the position of 3500,
        and the uint16_t "position" is the process variable.
        base speed is 40% of the duty cycle */
        int error = 3500 - position;
        int P = error;
        int I = I + error;
        int D = error - lastError;
        lastError = error;
        int speed = P * KpR + I * KiR + D * KdR;
        int X;
        int Y;

        float Rbase = 255 * .5; //255/2 .4
        float Lbase = 255 *.5;
        float RWspeed = Rbase - speed;
        float LWspeed = Lbase + speed;
        float RWmax = dutyCycle;
        float LWmax = dutyCycle;


        /*
        
        A series of speed evaluation procedures. never allow a speed over the max speed, 
        which is the duty cycle. if a speed is negative, allow for motor reversal. motor direction
        is indicated by variables X and Y, and can be set to 1 (HIGH - backwards)
        and 0 (LOW - forwards) */
        if (RWspeed > RWmax)
        {
            RWspeed = RWmax;
            X = 0;
            Y = 0;
        }
        if (LWspeed > LWmax)
        {
            LWspeed = LWmax;
            X = 0;
            Y = 0;
        }
        if (RWspeed < 0)
        {
            RWspeed = fabs(RWspeed);
            if (RWspeed > RWmax)
            {
                RWspeed = RWmax;
            }
            X = 1;
            Y = 0;
        }
        if (LWspeed < 0)
        {
            LWspeed = fabs(LWspeed);
            if (LWspeed > LWmax)
            {
                LWspeed = LWmax;
            }
            X = 0;
            Y = 1;
        }
        // if (analogRead(A7) > 140 && abs(error) > 1440 ) //66 / 940  also 110/1040
        // {
        //     Mercedes();
        // }
        // else if(analogRead(A0) > 140 && abs(error) > 1440)
        // {
        //     McLaren();
        // }

        /*
        
        this is the code that allows for sharp corner traversal, namely in the hairpins */
        if (abs(error) > 1000)
        {
            if (sensorValues[7] > 800 && sensorValues[6] >= 400 && (sensorValues[3] > 900 || sensorValues[4] > 900) && sensorValues[5] < 500)
            {
                Mercedes();

                //Serial.print("merc");
            }
            else if (sensorValues[0] > 900 && sensorValues[1] >= 400 && (sensorValues[3] > 900 || sensorValues[4] > 900) && sensorValues[2] < 500)
            {
                McLaren();
                //Serial.print("mclaren");
            }
            else if(sensorValues[7] > 950 && sensorValues[6] < 400 && (sensorValues[3] > 900 || sensorValues[4] > 900) && sensorValues[5] < 500){
                Mercedes();
                Serial.print("");
            }
            else if(sensorValues[0] > 950 && sensorValues[1] < 400 && (sensorValues[3] > 900 || sensorValues[4] > 900) && sensorValues[2] < 500){
                McLaren();
                Serial.print("");
            }
            else{
                eracount = 0;
                LewisHamilton(RWspeed, LWspeed, X, Y);
            }

        }
        else
        {
            eracount = 0;
            LewisHamilton(RWspeed, LWspeed, X, Y);
        }
    }
    else
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
    }
}
void CalibrateSensorArray()
{
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
    // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 13 seconds.
    for (uint16_t i = 0; i < 450; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}
/*
this is the main drive function. Lewis Hamilton, while I am not a fan of him as a driver,
he is someone I have a lot of respect for. tied with Michael Schumacher
with 7x F1 world champion, and the winningest driver in F1 history. */
void LewisHamilton(float right, float left, int RD, int LD)
{
    digitalWrite(RDIR, RD);
    digitalWrite(LDIR, LD);
    analogWrite(RPWM, right);
    analogWrite(LPWM, left);
}
void leftwheel()
{
    elacount++;
}
void rightwheel()
{
    eracount++;
}



/*Lewis Hamilton Drives for Mercedes-AMG. what else whould i name this? anyway,
this is the code that dictates sharp left-handed corners. also drives forward
ever so slightly to move the sensor away from the previous line on onto the next. 
i should point out Lewis isn't my favorite driver. not even close. I just respect his talent
and dedication. my favorite driver is....
*/
void Mercedes()
{
    //eracount = 0;

    while (eracount < 300) //should turn this thot 90 degrees CCW
    {
        digitalWrite(RDIR, LOW); //low is forwards
        digitalWrite(LDIR, HIGH);
        analogWrite(RPWM, dutyCycle);
        analogWrite(LPWM, dutyCycle);
    }
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    delay(10);
    digitalWrite(RDIR, LOW);
    digitalWrite(LDIR, LOW);
    analogWrite(RPWM, dutyCycle*.3);
    analogWrite(LPWM, dutyCycle*.3);
    delay(100);
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    //GONOGO = false;
    Serial.println("get in there LEWIS");
}
/*
...Daniel Ricciardo. Dan the man. following in the tradition of the great
Aussie drivers Mark Webber Alan Jones and Sir Jack Brabham, Daniel is a legend. 7x race winner, 
31 podiums, 3x pole sitter, and 2x 3rd place finsihes in the F1 championship, and a smile 
that lights up a room. Also, he drives for McLaren
 */
void McLaren()
{
    //eracount = 0;

    while (eracount < 300) //should turnt this thot 90 degrees CW
    {
        digitalWrite(RDIR, HIGH); //low is forwards
        digitalWrite(LDIR, LOW);
        analogWrite(RPWM, dutyCycle);
        analogWrite(LPWM, dutyCycle);
    }
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    delay(10);
    digitalWrite(RDIR, LOW);
    digitalWrite(LDIR, LOW);
    analogWrite(RPWM, dutyCycle*.3);
    analogWrite(LPWM, dutyCycle*.3);
    delay(100);
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);
    //GONOGO = false;
    Serial.print("DANNY RICC!!!!!!");
}