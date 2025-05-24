#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver rightServos = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver leftServos = Adafruit_PWMServoDriver(0x60);

#define SERVOMIN 102  // Minimum pulse length count (approx 500µs)
#define SERVOMAX 512  // Maximum pulse length count (approx 2500µs)

// Converts angle (0-180) to PWM pulse count
uint16_t angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

char auth[] = "e7OzjMzulxmwu4DOAZxRhZ9gOdB1ivR2";
char ssid[] = "Alex's Pixel 9";
char pass[] = "nostradamus";

BlynkTimer timer;

int offset[6][3] = {
  { 1, -4, 8 },
  { 13, -2, 0 },
  { 7, -10, -6 },
  { -5, -3, -3 },
  { 5, 5, 1 },
  { -8, -1, -1 }
};

#define DHTPIN 32  //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


void readDht() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);

  Serial.print("Temperature : ");
  Serial.print(t);
  Serial.print("    Humidity : ");
  Serial.println(h);
}

int Mq135SensorValue;
void readMq135() {
  Mq135SensorValue = analogRead(35);
  if (Mq135SensorValue < 800)
    Blynk.virtualWrite(V2, "Calitate aer: Buna");
  else if (Mq135SensorValue < 1200)
    Blynk.virtualWrite(V2, "Calitate aer: Medie");
  else
    Blynk.virtualWrite(V2, "Calitate aer: Rea");
}

int Mq2SensorValue;
const int smokeThreshold = 400;
void readMq2() {
  Mq2SensorValue - analogRead(33);
  if (Mq2SensorValue < smokeThreshold)
    Blynk.virtualWrite(V3, "Fum: Nu");
  else
    Blynk.virtualWrite(V3, "Fum: Da");
}

int Mq5SensorValue;
const int gasThreshold = 400;
void readMq5() {
  Mq5SensorValue - analogRead(34);
  if (Mq2SensorValue < gasThreshold)
    Blynk.virtualWrite(V4, "Gaz: Nu");
  else
    Blynk.virtualWrite(V4, "Gaz: Da");
}

void setup() {

  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  dht.begin();

  rightServos.begin();
  rightServos.setPWMFreq(50);  // Analog servos run at ~50 Hz

  leftServos.begin();
  leftServos.setPWMFreq(50);  // Analog servos run at ~50 Hz

  resetPosition();

  smartDelay(1000);
}

bool forward;
bool backward;
bool left;
bool right;
bool turnLeft;
bool turnRight;
int height = 0;

BLYNK_WRITE(V5) {
  left = param.asInt();
}

BLYNK_WRITE(V6) {
  right = param.asInt();
}

BLYNK_WRITE(V7) {
  forward = param.asInt();
}

BLYNK_WRITE(V8) {
  backward = param.asInt();
}

BLYNK_WRITE(V9) {
  turnLeft = param.asInt();
}

BLYNK_WRITE(V10) {
  turnRight = param.asInt();
}

BLYNK_WRITE(V11) {
  height = param.asInt();
}

void loop() {
  Blynk.run();
  timer.run();
  readDht();
  readMq135();
  readMq2();
  readMq5();
  checkForMovement();
  smartDelay(100);
}

void checkForMovement() {
  if (forward) moveForward();
  if (backward) moveBackward();
  if (left) moveLeft();
  if (right) moveRight();
  if (turnLeft) turnRobotLeft();
  if (turnRight) turnRobotRight();
  Serial.println(height);
  if (!forward && !backward && !left && !right && !turnLeft && !turnRight) resetPosition();
}

void moveForward() {
  Serial.println("Forward");

  bool groupALifted = false;

  while (forward) {

    if (!groupALifted) {
      //Lift Group A
      moveLeg(leftServos, 1, 45, 120, 50);
      smartDelay(50);
      moveLeg(rightServos, 2, 90, 60, 130);
      smartDelay(50);
      moveLeg(leftServos, 3, 135, 120, 50);
      smartDelay(200);
    }

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Move Group A Forward
    moveLeg(leftServos, 1, 25, 120, 50);
    smartDelay(50);
    moveLeg(rightServos, 2, 110, 60, 130);
    smartDelay(50);
    moveLeg(leftServos, 3, 110, 120, 50);
    smartDelay(200);

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Put Down Group A
    moveLeg(leftServos, 1, 25, 90, 85);
    smartDelay(50);
    moveLeg(rightServos, 2, 110, 90, 95);
    smartDelay(50);
    moveLeg(leftServos, 3, 110, 90, 85);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Lift Group B
    moveLeg(rightServos, 1, 135, 60, 130);
    smartDelay(50);
    moveLeg(leftServos, 2, 90, 120, 50);
    smartDelay(50);
    moveLeg(rightServos, 3, 45, 60, 130);
    smartDelay(500);

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Move Group A Back(Moving the robot forward)
    moveLeg(leftServos, 1, 45, 90, 85);
    smartDelay(50);
    moveLeg(rightServos, 2, 90, 90, 95);
    smartDelay(50);
    moveLeg(leftServos, 3, 135, 90, 85);
    smartDelay(500);

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Move Group B Forward
    moveLeg(rightServos, 1, 155, 60, 130);
    smartDelay(50);
    moveLeg(leftServos, 2, 70, 120, 50);
    smartDelay(50);
    moveLeg(rightServos, 3, 70, 60, 130);
    smartDelay(200);

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Put Down Group B
    moveLeg(rightServos, 1, 155, 90, 95);
    smartDelay(50);
    moveLeg(leftServos, 2, 70, 90, 85);
    smartDelay(50);
    moveLeg(rightServos, 3, 70, 90, 95);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Lift Group A
    moveLeg(leftServos, 1, 45, 120, 50);
    smartDelay(50);
    moveLeg(rightServos, 2, 90, 60, 130);
    smartDelay(50);
    moveLeg(leftServos, 3, 135, 120, 50);
    smartDelay(500);
    groupALifted = true;


    //Check If The Button Is Still Pressed
    if (!forward) {
      resetPosition();
      return;
    }

    //Move Group B Back(Moving the robot forward)
    moveLeg(rightServos, 1, 135, 90, 95);
    smartDelay(50);
    moveLeg(leftServos, 2, 90, 90, 85);
    smartDelay(50);
    moveLeg(rightServos, 3, 45, 90, 95);
    smartDelay(200);
  }
}


void moveBackward() {
  Serial.println("Backward");
  while (backward) {


    moveLeg(rightServos, 1, 90, 30, 95);
    smartDelay(100);
    if (!backward) {
      resetPosition();
      return;
    }
    moveLeg(rightServos, 3, 90, 30, 95);
    smartDelay(100);
  }
}

void moveLeft() {
  Serial.println("Left");
}

void moveRight() {
  Serial.println("Right");
}

void turnRobotLeft() {
  Serial.println("TurnLeft");
}

void turnRobotRight() {
  Serial.println("TurnRight");
}

void resetPosition() {
  moveLeg(leftServos, 1, 45, 90, 85);
  moveLeg(rightServos, 2, 90, 90, 95);
  moveLeg(leftServos, 3, 135, 90, 85);

  moveLeg(rightServos, 1, 135, 90, 95);
  moveLeg(leftServos, 2, 90, 90, 85);
  moveLeg(rightServos, 3, 45, 90, 95);
}


void moveLeg(Adafruit_PWMServoDriver& servos, int leg, int coxaDegrees, int femurDegrees, int tibiaDegrees) {
  int coxa = 0 + (4 * (leg - 1));
  int femur = 1 + (4 * (leg - 1));
  int tibia = 2 + (4 * (leg - 1));

  if (&servos == &leftServos) {
    coxaDegrees += offset[leg - 1][0];
    femurDegrees += offset[leg - 1][1];
    tibiaDegrees += offset[leg - 1][2];
  } else {
    coxaDegrees += offset[leg + 2][0];
    femurDegrees += offset[leg + 2][1];
    tibiaDegrees += offset[leg + 2][2];
  }

  int legHeight;
  if (&servos == &leftServos) legHeight = -(height);
  else legHeight = height;

  servos.setPWM(coxa, 0, angleToPulse(coxaDegrees));
  smartDelay(10);
  servos.setPWM(femur, 0, angleToPulse(femurDegrees + legHeight));
  smartDelay(10);
  servos.setPWM(tibia, 0, angleToPulse(tibiaDegrees + legHeight));
  smartDelay(10);
}

void smartDelay(int delayParam) {
  for (int i = 0; i < delayParam / 10; i++) {
    delay(10);
    Blynk.run();
    timer.run();
  }
}