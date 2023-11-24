#include <ECE3.h>

//CONSTANTS
const int L_NSLP_PIN = 31, 
          L_DIR_PIN = 29,
          L_PWM_PIN = 40;
const int R_NSLP_PIN = 11, 
          R_DIR_PIN = 30,
          R_PWM_PIN = 39;

const int N_SENSORS = 8;
const int SENSOR_WEIGHTS[] = {-15, -14, -12, -8, 8, 12, 14, 15};
//Consider calculating at the start of every run to account for room-specific lighting
const float MIN_SENSOR_VALS[] = {700.6, 644.2, 649.2, 593.2, 607.6, 635.4, 682.4, 696.4}; 
const float NORM_FACTORS[] = {0.55574, 0.53885, 0.54897, 0.52444, 0.52843, 0.53631, 0.55018, 0.55444};

const float Kp = 1,
            Ki = 1,
            Kd = 1;
//VARIABLES
float normSensorVals[N_SENSORS] = {};

float error = 0,
      prevError = 0,
      errorIntegral = 0;
float prevSpeed = 0;

float lastUpdateSecs = 0; //Consider the possibllity memory runs out?

void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
  /*
  * This given function changes the car base speed gradually (in about 300 ms) from
  * initialBaseSpeed to finalBaseSpeed. This non-instantaneous speed change
  * reduces the load on the plastic geartrain, and reduces the failure rate of
  * the motors.
  */
  int numSteps = 5;
  int pwmLeftVal = initialBaseSpd; 
  int pwmRightVal = initialBaseSpd; 
  int deltaLeft = (finalBaseSpd-initialBaseSpd)/numSteps; 
  int deltaRight = (finalBaseSpd-initialBaseSpd)/numSteps; 
  for(int k = 0; k < numSteps; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(L_PWM_PIN, pwmLeftVal);
    analogWrite(R_PWM_PIN, pwmRightVal);
    delay(60);
  }
} 

int updateNormSensorVals(int nSamples, int delayMillis) {
  float avgSensorVals[N_SENSORS] = {};
  //Average sensor values
  for (int i = 0; i < nSamples; i++) {
    uint16_t rawSensorVals[N_SENSORS] = {};
    ECE3_read_IR(rawSensorVals);

    for (int j = 0; j < N_SENSORS; j++) {
      avgSensorVals[j] += (float)rawSensorVals[j];
    }
    delay(delayMillis);
  }
  for (int i = 0; i < N_SENSORS; i++) {
    avgSensorVals[i] /= nSamples;
    //Normalize sensor values
    normSensorVals[i] = max((avgSensorVals[i] - MIN_SENSOR_VALS[i]) * NORM_FACTORS[i], 0);
  }
  return 1;
}

float updateError() {
  int N_SAMPLES = 5,
      DELAY_MILLIS = 20;
  updateNormSensorVals(N_SAMPLES, DELAY_MILLIS);
  error = 0;
  for (int i = 0; i < N_SENSORS; i++) {
    error += normSensorVals[i]*SENSOR_WEIGHTS[i];
  }
}

void setup() {
  ECE3_Init();
  
  pinMode(L_NSLP_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_NSLP_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT); 
  
  resetEncoderCount_left();
  resetEncoderCount_right();

  //Enable motors
  digitalWrite(L_NSLP_PIN, HIGH);
  digitalWrite(R_NSLP_PIN, HIGH);
  
  Serial.begin(9600);
  lastUpdateSecs = millis()/1000;
}

void loop() {
  updateError();
  float dt = (millis()/1000) - lastUpdateSecs;
  float errorDerivative = (error - prevError)/dt;
  errorIntegral += error * dt;
  
  float motorSpeed = Kp * error + Kd * errorDerivative + Ki * errorIntegral;
  lastUpdateSecs = millis()/1000;
  ChangeBaseSpeed(prevSpeed, motorSpeed);

  prevError = error;
  prevSpeed = motorSpeed;
}
