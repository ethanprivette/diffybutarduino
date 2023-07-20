#include <AlfredoConnect.h>
#include <BluetoothSerial.h>
#include <Alfredo_NoU2.h>
#include <Adafruit_BNO08x.h>

BluetoothSerial bluetooth;

#define BNO08X_RESET -1

//Motor def
NoU_Motor moduleMotorL(1);
NoU_Motor moduleMotorR(2);

//bno08x def
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

long report_interval = 5000;

//yaw-pitch-roll struct
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

//Translation struct
struct Translation2d {
  double y;
  double x;
};

//Module state struct
struct moduleState {
  float speed;
  double angle;
};

//encoder stuff
double motorRotationsPerDegree =  360.0;

//encoder 0
int encoder0Pin1 = 0; //CHANGE
int encoder0Pin2 = 0; //CHANGE

volatile int lastEncoder0Pos = 0;
volatile long encoder0Pos = 0;

//encoder 1
int encoder1Pin1 = 0; //CHANGE
int encoder1Pin2 = 0; //CHANGE

volatile int lastEncoder1Pos = 0;
volatile long encoder1Pos = 0;

//number of modules
int numModules = 3;

//drive speeds
float vxMetersPerSec = 0;
float vyMetersPerSec = 0;
float radiansPerSec = 0;

//drive kinematics
float inverseKinematics[6][3] = { {1, 0, 0.083439}, {0, 1, 0}, {1, 0, -0.0359214166}, {0, 1, 0.0512338574}, {1, 0, -0.0359214166}, {0, 1, -0.0512338574} };
float forwardKinematics[3][6];

//matrix for something
float moduleStatesMatrix[6][3] = {0};

//module states matrix
moduleState m_moduleStates[3] = { moduleState{0, 0}, moduleState{0, 0}, moduleState{0, 0} };

//previous module states matrix
moduleState m_prevModuleStates[3] = { moduleState{0, 0}, moduleState{0, 0}, moduleState{0, 0} };

//psuedo inverse function (thanks chat-gpt)
void calculatePseudoInverse(float A[][3], float Aplus[][6], int rows, int cols) {
  // Temporary variables
  float U[rows][rows], S[rows][cols], V[cols][cols];
  
  // Perform Singular Value Decomposition (SVD)
  // You can implement your own SVD function or use a library
  
  // Compute pseudo-inverse of S
  float Splus[cols][rows];
  
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      if (i == j && S[i][j] != 0.0) {
        Splus[i][j] = 1.0 / S[i][j];
      } else {
        Splus[i][j] = 0.0;
      }
    }
  }
  
  // Compute pseudo-inverse A+
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      Aplus[i][j] = 0.0;
      for (int k = 0; k < rows; k++) {
        Aplus[i][j] += V[i][k] * Splus[k][j] * U[j][k];
      }
    }
  }
}

//mult for moduleStatesMatrix (thanks again chat-gpt)
void mult(float a[][3], float b[][3], float result[][3]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

//gyro stuff
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void setReports() {
  if (!bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, report_interval)) {
    bluetooth.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

//convert raw encoder value into angle(degrees)
double rawEncoderToAngle(long rawValue) {
  return (rawValue);
}

//convert angle(degrees) into encoder count
long angleToRawEncoder(double angle) {
  return angle * 2800;
}

//encoder 0 update
void updateEncoder0(){
  int MSB = digitalRead(encoder0Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder0Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoder0Pos << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder0Pos --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder0Pos ++;

  lastEncoder0Pos = encoded; //store this value for next time
}

//encoder 1 update
void updateEncoder1(){
  int MSB = digitalRead(encoder1Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder1Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoder1Pos << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder1Pos --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder1Pos ++;

  lastEncoder1Pos = encoded; //store this value for next time
}

void setup() {
  //begin bluetooth and Alfredo
  bluetooth.begin("moduletestencoders");
  AlfredoConnect.begin(bluetooth);

  //wait for Alfredo
  while (!(AlfredoConnect.getGamepadCount() >= 1)) {
    delay(100);
    AlfredoConnect.update();
  }

  //see if gyro is found
  if (!bno08x.begin_I2C()) {
    bluetooth.println("Failed to find BNO08x chip. Restart to attempt reconnecting");
  }

  //sets report/report interval
  setReports();

  //encoder 0 setup
  pinMode(encoder0Pin1, INPUT_PULLUP); 
  pinMode(encoder0Pin2, INPUT_PULLUP);

  digitalWrite(encoder0Pin1, HIGH);
  digitalWrite(encoder0Pin2, HIGH);

  attachInterrupt(0, updateEncoder0, CHANGE); 
  attachInterrupt(1, updateEncoder0, CHANGE);

  //encoder 1 setup
  pinMode(encoder1Pin1, INPUT_PULLUP); 
  pinMode(encoder1Pin2, INPUT_PULLUP);

  digitalWrite(encoder1Pin1, HIGH);
  digitalWrite(encoder1Pin2, HIGH);

  attachInterrupt(0, updateEncoder1, CHANGE); 
  attachInterrupt(1, updateEncoder1, CHANGE);

  //calculate drive kinematics on startup
  calculatePseudoInverse(inverseKinematics, forwardKinematics, 6, 3);
}

void loop() {
  //drive stuff
  if (AlfredoConnect.getGamepadCount() >= 1) {
    float xVel = AlfredoConnect.getAxis(0, 1);
    float yVel = AlfredoConnect.getAxis(0, 0);
    float rot = AlfredoConnect.getAxis(0, 2);
    
    drive(xVel, yVel, rot, false);
  }

  //gyro reset print
  if (bno08x.wasReset()) {
    bluetooth.println("sensor was reset");
    setReports();
  }

  //update gyro reading
  if (bno08x.getSensorEvent(&sensorValue)) {
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  }
}

//converts speeds into swerve states
void toSwerveModuleStates(float vxMeterSec, float vyMeterSec, float radPerSec) {
  //set everything to 0 if no input
  if (vxMeterSec == 0.0 && vyMeterSec == 0.0 && radPerSec == 0.0) {
    for(int i = 0; i < numModules; i++) {
      m_moduleStates[i].angle = 0.0;
      m_moduleStates[i].speed = 0.0;
    }
  }

  //new chassis speeds matrix
  float chassisSpeedVector[1][3] = {vxMeterSec, vyMeterSec, radPerSec};

  //mult kinematics and chassis speeds to get states matrix
  mult(inverseKinematics, chassisSpeedVector, moduleStatesMatrix);

  //set module states speed/angle to 0
  for (int i = 0; i < numModules; i++) {
    m_moduleStates[i].speed = 0.0;
    m_moduleStates[i].angle = 0.0;
  }

  //sin/cos vars
  double m_sin = 0;
  double m_cos = 0;

  //set module state array with correct speeds/angles
  for (int i = 0; i < numModules; i++) {
    double x = moduleStatesMatrix[i * 2][0];
    double y = moduleStatesMatrix[i * 2 + 1][0];

    double speed = hypot(x, y);
    if (speed > 1e-6) {
      m_sin = y / speed;
      m_cos = x / speed;
    } else {
      m_sin = 0.0;
      m_cos = 1.0;
    }
    double angle = atan2(m_sin, m_cos);

    m_moduleStates[i] = (moduleState) {speed, angle};
  }
}

//convert joystick input into field oriented speeds
void fromFieldRelSpeed(float vxMetersSec, float vyMetersSec, float radiansSec, float robotAngle) {
  vxMetersPerSec = vxMetersSec * cos(robotAngle) + vyMetersSec * sin(robotAngle);
  vyMetersPerSec = -vxMetersSec * sin(robotAngle) + vyMetersSec * cos(robotAngle);
  radiansPerSec = radiansSec;
}

//make sure speeds are not too high
void desaturateWheelSpeeds(moduleState moduleStates[], double attainableMaxSpeedMetersPerSecond) {
  double realMaxSpeed = moduleStates[0].speed;
  
  for (int i = 1; i < numModules; i++) {
    if (moduleStates[i].speed > realMaxSpeed) {
      realMaxSpeed = moduleStates[i].speed;
    }
  }
  
  if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
    for (int i = 0; i < numModules; i++) {
      moduleStates[i].speed = (moduleStates[i].speed / realMaxSpeed) * attainableMaxSpeedMetersPerSecond;
    }
  }
}

//uses current and new rot to optimize module angle
void addRotation(float currentRotation, float newRotation, int module) {
  float cosCurrent = cos(currentRotation);
  float sinCurrent = sin(currentRotation);
  float cosNew = cos(newRotation);
  float sinNew = sin(newRotation);

  float cosResult = cosNew * cosCurrent - sinNew * sinCurrent;
  float sinResult = sinNew * cosCurrent + cosNew * sinCurrent;

  m_moduleStates[module].angle = atan2(sinResult, cosResult);
}

//optimize modules
void optimize() {
  //module 0
  float delta0 = m_prevModuleStates[0].angle - m_moduleStates[0].angle;
  if (abs(delta0) > 90.0) {
    m_moduleStates[0].speed *= -1;
    addRotation(m_prevModuleStates[0].angle, m_moduleStates[0].angle, 0);
  }

  //module 1
  float delta1 = m_prevModuleStates[1].angle - m_moduleStates[1].angle;
  if (abs(delta1) > 90.0) {
    m_moduleStates[1].speed *= -1;
    addRotation(m_prevModuleStates[1].angle, m_moduleStates[1].angle, 1);
  }

  //module 2
  float delta2 = m_prevModuleStates[2].angle - m_moduleStates[2].angle;
  if (abs(delta2) > 90.0) {
    m_moduleStates[2].speed *= -1;
    addRotation(m_prevModuleStates[2].angle, m_moduleStates[2].angle, 2);
  }
}

//helper for setting module states
// void  toMotorOutput() {
//   int desiredCounts = ();
//   int prevCounts = (prevAngle / 360.0 * ENCODER_RESOLUTION);
// }

//set module states and motor speeds
void setModuleStates(moduleState moduleStates[]) {
  //optimize modules
  optimize();

  //calculate motor output to set module angle
  //TODO: math

  //set module0 drive speeds
  if (moduleStates[0].speed < 0) {
    moduleMotorL.set(moduleStates[0].speed);
    moduleMotorR.set(-moduleStates[0].speed);
  } else {
    moduleMotorL.set(-moduleStates[0].speed);
    moduleMotorR.set(moduleStates[0].speed);
  }
}

//main drive function
void drive(float xVel, float yVel, float rot, bool fieldCentric) {
  if(!fieldCentric) {
    vxMetersPerSec = xVel;
    vyMetersPerSec = yVel;
    radiansPerSec = rot;
  } else {
    fromFieldRelSpeed(xVel, yVel, rot, ypr.yaw);
  }
  for (int i = 0; i < 3; i++) {
    m_prevModuleStates[i] = m_moduleStates[i];
  }
  toSwerveModuleStates(vxMetersPerSec, vyMetersPerSec, radiansPerSec);
  desaturateWheelSpeeds(m_moduleStates, 1);
  setModuleStates(m_moduleStates);
}