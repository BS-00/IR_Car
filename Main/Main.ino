#include <ECE3.h>
#undef min
#undef max
#include <array>

#include "Main.hpp"
#include "Wheels.hpp"
#include "SensorArray.hpp"


SensorArray sensorArray;
Wheels wheels;

float prevError = 0,
      errorIntegral = 0;
int nDonuts = 0;


void setup() {
  ECE3_Init();
  Serial.begin(115200);
  if (!sensorArray.init() || !wheels.init()) exit(1);

  //Indicate done callibrating
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  //Wait until user presses bump switch to start the loop
  if(!sensorArray.awaitBump(0)) exit(1);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (sensorArray.sensorSum() > DONUT_ERR_THRESH) {
    wheels.donut();
    //Car reached the start
    if (nDonuts == 1) {
      wheels.halt();
      exit(0); 
    }
    nDonuts++;
  }
  
  float error = sensorArray.error();
  float errorDerivative = error - prevError;
  errorIntegral += error;
  int correction = int(Kp*error + Kd*errorDerivative + Ki*errorIntegral);

  int lSpeed = BASE_SPEED,
      rSpeed = BASE_SPEED;
  if (abs(error) > CURVE_ERR_THRESH) {
    //On a curve
    lSpeed -= CURVE_SPEED_DECREASE;
    rSpeed -= CURVE_SPEED_DECREASE;
  } else if (abs(error) < STRAIGHT_ERR_THRESH) {
    //On a straightaway
    lSpeed += STRAIGHT_SPEED_BOOST;
    rSpeed += STRAIGHT_SPEED_BOOST;
  }

  //Neg error => turn right, positive error => turn left
  wheels.setVelocities(lSpeed-correction, rSpeed+correction, 1, 0);

  prevError = error;
}