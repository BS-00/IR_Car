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

const float MAX_NORM_VAL = 1000,
            MIN_NORM_VAL = 0;
const float NORM_FACTORS[] = {0.55574, 0.53885, 0.54897, 0.52444, 0.52843, 0.53631, 0.55018, 0.55444};

const int BASE_SPEED = 100,
          MIN_SPEED = 10,
          MAX_SPEED = 255;
#define TUNABLE_SPEED min(BASE_SPEED-MIN_SPEED, MAX_SPEED-BASE_SPEED)

//Completley arbitrary / based on prop constants?
const int MAX_PID_VAL = 7000;
//Proportionality constants (need tweaking)
const float Kp = 1,
            Ki = 1,
            Kd = 1;
//VARIABLES
float error = 0,
      prevError = 0,
      errorIntegral = 0,
      prevCorrection = 0;

unsigned long lastUpdateMillis = 0;


int updateWheelSpeeds(int initRSpeed=-1, int finalRSpeed=-1, 
                      int initLSpeed=-1,  int finalLSpeed=-1, 
                      int nSteps=5, int delayMillis=50) {
  int lPWM = initLSpeed,
      rPWM = initRSpeed;
  int dL = (finalLSpeed - initLSpeed)/nSteps,
      dR = (finalRSpeed - initRSpeed)/nSteps;
  for (int i = 0; i < nSteps; i++) {
    lPWM += dL;
    rPWM += dR;
    if (initLSpeed > 0 && finalLSpeed > 0) analogWrite(L_PWM_PIN, lPWM);
    if (initRSpeed > 0 && finalRSpeed > 0) analogWrite(R_PWM_PIN, rPWM);
    delay(delayMillis);
  }
  return 1;
}

int updateError(int nSamples=5, int delayMillis=20) {
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
  error = 0;
  for (int i = 0; i < N_SENSORS; i++) {
    avgSensorVals[i] /= nSamples;
    //Normalize sensor values between 0 and 1000 (some cuttoff can occur here but hopefully it will not make a large difference)
    float normSensorVal = min(max((avgSensorVals[i] - MIN_SENSOR_VALS[i]) * NORM_FACTORS[i], MIN_NORM_VAL), MAX_NORM_VAL);
    //Calaulate sensor fusion error
    error += normSensorVal * SENSOR_WEIGHTS[i];
  }
  return 1;
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
  lastUpdateMillis = millis();
}

const int LOOP_DELAY_MILLIS = 10;
void loop() {
  //Check for special cases like donuts / end of track

  updateError();
  float dt = millis() - lastUpdateMillis;
  float errorDerivative = (error - prevError)/dt;
  errorIntegral += error * dt;
  
  float pidOut = constrain(Kp*error + Kd*errorDerivative + Ki*errorIntegral, -MAX_PID_VAL, MAX_PID_VAL);
  int correction = map(pidOut, -MAX_PID_VAL, MAX_PID_VAL, -TUNABLE_SPEED, TUNABLE_SPEED);
  lastUpdateMillis = millis();
  updateWheelSpeeds(BASE_SPEED+prevCorrection, BASE_SPEED+correction, 
                    BASE_SPEED-prevCorrection, BASE_SPEED-correction);

  prevError = error;
  prevCorrection = correction;
  delay(LOOP_DELAY_MILLIS);
}
