#include <Adafruit_MotorShield.h>
#include <AlfredoConnect.h>
#include <BluetoothSerial.h>
#include <Alfredo_NoU2.h>
#include <Adafruit_BNO08x.h>

BluetoothSerial bluetooth;

#define BNO08X_RESET -1

// Create the motor shield object with the default I2C address
// Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
Adafruit_MotorShield rearModules = Adafruit_MotorShield(0x60);

Adafruit_MotorShield frontModuleAndExtra = Adafruit_MotorShield(0x61); //front module motors port 1 and 2

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *rearLeft1 = rearModules.getMotor(1);
Adafruit_DCMotor *rearLeft2 = rearModules.getMotor(2);
Adafruit_DCMotor *rearRight1 = rearModules.getMotor(3);
Adafruit_DCMotor *rearRight2 = rearModules.getMotor(4);
Adafruit_DCMotor *frontLeft = frontModuleAndExtra.getMotor(1);
Adafruit_DCMotor *frontRight = frontModuleAndExtra.getMotor(2);

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

long report_interval = 5000;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct Translation2d {
  double y;
  double x;
};

struct moduleState {
  float speed;
  double angle;
};

int encoderPin1 = 2;
int encoderPin2 = 3;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

double m_sin = 0;
double m_cos = 0;

int numModules = 3;

float vxMetersPerSec = 0;
float vyMetersPerSec = 0;
float radiansPerSec = 0;

float inverseKinematics[6][3] = { {1, 0, 0.083439}, {0, 1, 0}, {1, 0, -0.0359214166}, {0, 1, 0.0512338574}, {1, 0, -0.0359214166}, {0, 1, -0.0512338574} };
float forwardKinematics[3][6];

float moduleStatesMatrix[6][3] = {0};

moduleState m_moduleStates[3] = { moduleState{0, 0}, moduleState{0, 0}, moduleState{0, 0} };
moduleState m_prevModuleStates[3] = { moduleState{0, 0}, moduleState{0, 0}, moduleState{0, 0} };

double prev_CoR = 0.0;

double robotAngle = 0;

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
  if (! bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, report_interval)) {
    bluetooth.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

long convertRawEncoder(long rawValue) {
  return rawValue/2800;
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  lastEncoded = encoded; //store this value for next time

}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  bluetooth.begin("diffytest");
  AlfredoConnect.begin(bluetooth);

  if (!rearModules.begin() && !frontModuleAndExtra.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shields. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  if (!bno08x.begin_I2C()) {
    bluetooth.println("Failed to find BNO08x chip. Restart to attempt reconnecting");
  }

  setReports();

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);

  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  calculatePseudoInverse(inverseKinematics, forwardKinematics, 6, 3);
}

void loop() {
  if (AlfredoConnect.getGamepadCount() >= 1) {
    float xVel = AlfredoConnect.getAxis(0, 1);
    float yVel = AlfredoConnect.getAxis(0, 0);
    float rot = AlfredoConnect.getAxis(0, 2);
    
    drive(xVel, yVel, rot, false);

  }

  if (bno08x.wasReset()) {
    bluetooth.println("sensor was reset");
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  }
}

void toSwerveModuleStates(float vxMeterSec, float vyMeterSec, float radPerSec) {
  if (vxMeterSec == 0.0 && vyMeterSec == 0.0 && radPerSec == 0.0) {
    for(int i = 0; i < numModules; i++) {
      m_moduleStates[i].angle = 0.0;
      m_moduleStates[i].speed = 0.0;
    }
  }

  float chassisSpeedVector[1][3] = {vxMeterSec, vyMeterSec, radPerSec};

  mult(inverseKinematics, chassisSpeedVector, moduleStatesMatrix);

  for (int i = 0; i < numModules; i++) {
    m_moduleStates[i].speed = 0.0;
    m_moduleStates[i].angle = 0.0;
  }

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

void fromFieldRelSpeed(float vxMetersSec, float vyMetersSec, float radiansSec, float robotAngle) {
  vxMetersPerSec = vxMetersSec * cos(robotAngle) + vyMetersSec * sin(robotAngle);
  vyMetersPerSec = -vxMetersSec * sin(robotAngle) + vyMetersSec * cos(robotAngle);
  radiansPerSec = radiansSec;
}

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

void addRotation(float currentRotation, float newRotation, int module) {
  float cosCurrent = cos(currentRotation);
  float sinCurrent = sin(currentRotation);
  float cosNew = cos(newRotation);
  float sinNew = sin(newRotation);

  float cosResult = cosNew * cosCurrent - sinNew * sinCurrent;
  float sinResult = sinNew * cosCurrent + cosNew * sinCurrent;

  m_moduleStates[module].angle = atan2(sinResult, cosResult);
}


void optimize() {
  float delta0 = m_prevModuleStates[0].angle - m_moduleStates[0].angle;
  if (abs(delta0) > 90.0) {
    m_moduleStates[0].speed *= -1;
    addRotation(m_prevModuleStates[0].angle, m_moduleStates[0].angle, 0);
  }

  float delta1 = m_prevModuleStates[1].angle - m_moduleStates[1].angle;
  if (abs(delta1) > 90.0) {
    m_moduleStates[1].speed *= -1;
    addRotation(m_prevModuleStates[1].angle, m_moduleStates[1].angle, 1);
  }

  float delta2 = m_prevModuleStates[2].angle - m_moduleStates[2].angle;
  if (abs(delta2) > 90.0) {
    m_moduleStates[2].speed *= -1;
    addRotation(m_prevModuleStates[2].angle, m_moduleStates[2].angle, 2);
  }
}

float getModuleSpeed(int module) {
  return m_moduleStates[module].speed * 255;
}

void setModuleStates(moduleState moduleStates[]) {
  optimize();

  //math

  if (moduleStates[0].speed < 0) {
    frontLeft.run(FORWARD);
    frontRight.run(BACKWARD);
    frontLeft.setSpeed(getModuleSpeed(0));
    frontRight.setSpeed(getModuleSpeed(0));
  } else {
    frontLeft.run(BACKWARD);
    frontRight.run(FORWARD);
    frontLeft.setSpeed(getModuleSpeed(0));
    frontRight.setSpeed(getModuleSpeed(0));
  }

  if (moduleStates[1].speed < 0) {
    frontLeft.run(FORWARD);
    frontRight.run(BACKWARD);
    frontLeft.setSpeed(getModuleSpeed(1));
    frontRight.setSpeed(getModuleSpeed(1));
  } else {
    frontLeft.run(BACKWARD);
    frontRight.run(FORWARD);
    frontLeft.setSpeed(getModuleSpeed(1));
    frontRight.setSpeed(getModuleSpeed(1));
  }

  if (moduleStates[2].speed < 0) {
    frontLeft.run(FORWARD);
    frontRight.run(BACKWARD);
    frontLeft.setSpeed(getModuleSpeed(2));
    frontRight.setSpeed(getModuleSpeed(2));
  } else {
    frontLeft.run(BACKWARD);
    frontRight.run(FORWARD);
    frontLeft.setSpeed(getModuleSpeed(2));
    frontRight.setSpeed(getModuleSpeed(2));
  }
}

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