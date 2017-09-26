#include <PID_v1.h>

#define RADIUS          7                          // this is in cm
#define WARNING_LAMP    13
#define motorPin        5
#define PingPinFront    7
#define EchoPinFront    8

#define PingPinBack     9
#define EchoPinBack     10

double PIDoutputSpeed=0;
//tachometer is connected to pin 2

//global variables
volatile int tachoCount=0;

double measuredSpeed=0,desiredSpeed=10;

double measuredDistanceFront=0,desiredDistanceFront=120;
double measuredDistanceBack=0, desiredDistanceBack=120;

double Kp_distanceBack=2, Ki_distanceBack=5, kd_distanceBack=1;
PID pidForDistanceBack( &measuredDistanceBack, 
                        &PIDoutputSpeed,
                        &desiredDistanceBack, 
                        Kp_distanceBack, 
                        Ki_distanceBack, 
                        kd_distanceBack, 
                        REVERSE);

double Kp_distanceFront=2, Ki_distanceFront=5, Kd_distanceFront=1;
PID pidForDistanceFront(&measuredDistanceFront, 
                        &PIDoutputSpeed,
                        &desiredDistanceFront,
                        Kp_distanceFront, 
                        Ki_distanceFront, 
                        Kd_distanceFront, 
                        DIRECT);

double Kp_speed=2, Ki_speed=5, Kd_speed=1;
PID pidForSpeed        (&measuredSpeed, 
                        &PIDoutputSpeed, 
                        &desiredSpeed,
                        Kp_speed, 
                        Ki_speed, 
                        Kd_speed, 
                        REVERSE);

void setup() 
{
  Serial.begin(9600);
  attachInterrupt(0, tachoISR, FALLING);
  pinMode(PingPinFront, OUTPUT);
  pinMode(EchoPinFront, INPUT);
  pinMode(PingPinBack, OUTPUT);
  pinMode(EchoPinBack, INPUT);
  pinMode(WARNING_LAMP, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pidForSpeed.SetMode(AUTOMATIC);
  pidForDistanceBack.SetMode(AUTOMATIC);
  pidForDistanceFront.SetMode(AUTOMATIC);
  pidForDistanceBack.SetOutputLimits(0, 255);
  pidForDistanceFront.SetOutputLimits(0,255);
  pidForSpeed.SetOutputLimits(0,255);

}

void loop() 
{
  delay(1000);
  detachInterrupt(0);
  
  measuredDistanceFront=calculateDistance(PingPinFront, EchoPinFront);
  measuredDistanceBack=calculateDistance(PingPinBack, EchoPinBack);

  //SPEED IN KMPH=RPM*PERIMETER IN CM*0.01  /   1000
  //                                   |          |
  //                                 cm TO m    m TO Km
  measuredSpeed=tachoCount*(2*PI*RADIUS)*60*0.01/1000;
  
  //this is where all magic happens
  if(measuredDistanceFront<=desiredDistanceFront)
  {
    if(measuredDistanceBack>desiredDistanceBack)
    {
        //reduce the speed
        pidForDistanceBack.Compute();
        Serial.println(PIDoutputSpeed);
        analogWrite(motorPin, PIDoutputSpeed);

    }
    else
    {
        //warning, stop the car and go to infinite loop
        digitalWrite(motorPin, LOW);
        digitalWrite(WARNING_LAMP, HIGH);
        while(1){}
    }
  }

  else
  {
    if(measuredDistanceBack<=desiredDistanceBack)
    {
        //increase the speed
        pidForDistanceFront.Compute();
        Serial.println(PIDoutputSpeed);
        analogWrite(motorPin, PIDoutputSpeed);
    }
    else
    {
        //make speed variation based on tachometer
        pidForSpeed.Compute();
        Serial.println(PIDoutputSpeed);
        analogWrite(motorPin, PIDoutputSpeed);
    }
  }

  attachInterrupt(0, tachoISR, FALLING);
}



void tachoISR()
{
  tachoCount++;
}

unsigned int calculateDistance(int PingPin, int EchoPin)
{
  long duration;
  digitalWrite(PingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(PingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(PingPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  return duration / 29 / 2;
}
