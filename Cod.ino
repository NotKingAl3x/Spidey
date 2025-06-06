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

int currentDegrees[6][3]{
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

#define DHTPIN 32  //Connect Out pin to D2 in NODE MCU
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


void readDht() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  Blynk.virtualWrite(V0, (t - 4));
  Blynk.virtualWrite(V1, h);

  Serial.print("Temperature : ");
  Serial.print(t - 4);
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
const int smokeThreshold = 2500;
void readMq2() {
  Mq2SensorValue = analogRead(33);
  Serial.print("Smoke: ");
  Serial.println(Mq2SensorValue);
  if (Mq2SensorValue < smokeThreshold)
    Blynk.virtualWrite(V3, "Fum: Nu");
  else
    Blynk.virtualWrite(V3, "Fum: Da");
}

int Mq5SensorValue;
const int gasThreshold = 1000;
void readMq5() {
  Mq5SensorValue = analogRead(34);
  Serial.print("Gas: ");
  Serial.println(Mq5SensorValue);
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

  while (forward) {

    //Put Leg L1 In Position
    smoothLifting(leftServos, 1, 35);
    moveLeg(leftServos, 1, -20, 0, 0);
    smartDelay(50);
    smoothLifting(leftServos, 1, -35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Leg R2 In Position
    smoothLifting(rightServos, 2, -35);
    moveLeg(rightServos, 2, 20, 0, 0);
    smartDelay(50);
    smoothLifting(rightServos, 2, 35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Leg L3 In Position
    smoothLifting(leftServos, 3, 35);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(50);
    smoothLifting(leftServos, 3, -35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Lift Group B
    smoothLifting(rightServos, 1, -35);
    smoothLifting(leftServos, 2, 35);
    smoothLifting(rightServos, 3, -35);
    smartDelay(200);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Move Group A Back(Moving the robot forward)
    moveLeg(leftServos, 1, 20, 0, 0);
    moveLeg(rightServos, 2, -20, 0, 0);
    moveLeg(leftServos, 3, 20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Down Group B
    smoothLifting(rightServos, 1, 35);
    smoothLifting(leftServos, 2, -35);
    smoothLifting(rightServos, 3, 35);
    smartDelay(200);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Leg R1 In Position
    smoothLifting(rightServos, 1, -35);
    moveLeg(rightServos, 1, 20, 0, 0);
    smartDelay(50);
    smoothLifting(rightServos, 1, 35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Leg L2 In Position
    smoothLifting(leftServos, 2, 35);
    moveLeg(leftServos, 2, -20, 0, 0);
    smartDelay(50);
    smoothLifting(leftServos, 2, -35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Leg R3 In Position
    smoothLifting(rightServos, 3, -35);
    moveLeg(rightServos, 3, 20, 0, 0);
    smartDelay(50);
    smoothLifting(rightServos, 3, 35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Lift Group A
    smoothLifting(leftServos, 1, 35);
    smoothLifting(rightServos, 2, -35);
    smoothLifting(leftServos, 3, 35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Move Group B Back(Moving the robot forward)
    moveLeg(rightServos, 1, -20, 0, 0);
    moveLeg(leftServos, 2, 20, 0, 0);
    moveLeg(rightServos, 3, -20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }

    //Put Down Group A
    smoothLifting(leftServos, 1, -35);
    smoothLifting(rightServos, 2, 35);
    smoothLifting(leftServos, 3, -35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!forward) {
      preResetPosition();
      return;
    }
  }
}


void moveBackward() {
  Serial.println("Backward");

  while (backward) {

    //Put Leg L1 In Position
    smoothLifting(leftServos, 1, 35);
    moveLeg(leftServos, 1, 20, 0, 0);
    smartDelay(50);
    smoothLifting(leftServos, 1, -35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Leg R2 In Position
    smoothLifting(rightServos, 2, -35);
    moveLeg(rightServos, 2, -20, 0, 0);
    smartDelay(50);
    smoothLifting(rightServos, 2, 35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Leg L3 In Position
    smoothLifting(leftServos, 3, 35);
    moveLeg(leftServos, 3, 20, 0, 0);
    smartDelay(50);
    smoothLifting(leftServos, 3, -35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Lift Group B
    smoothLifting(rightServos, 1, -35);
    smoothLifting(leftServos, 2, 35);
    smoothLifting(rightServos, 3, -35);
    smartDelay(200);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Move Group A forwards(Moving the robot backward)
    moveLeg(leftServos, 1, -20, 0, 0);
    moveLeg(rightServos, 2, 20, 0, 0);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Down Group B
    smoothLifting(rightServos, 1, 35);
    smoothLifting(leftServos, 2, -35);
    smoothLifting(rightServos, 3, 35);
    smartDelay(200);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Leg R1 In Position
    smoothLifting(rightServos, 1, -35);
    moveLeg(rightServos, 1, -20, 0, 0);
    smartDelay(50);
    smoothLifting(rightServos, 1, 35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Leg L2 In Position
    smoothLifting(leftServos, 2, 35);
    moveLeg(leftServos, 2, 20, 0, 0);
    smartDelay(50);
    smoothLifting(leftServos, 2, -35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Leg R3 In Position
    smoothLifting(rightServos, 3, -35);
    moveLeg(rightServos, 3, -20, 0, 0);
    smartDelay(50);
    smoothLifting(rightServos, 3, 35);
    smartDelay(80);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Lift Group A
    smoothLifting(leftServos, 1, 35);
    smoothLifting(rightServos, 2, -35);
    smoothLifting(leftServos, 3, 35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Move Group B Forward(Moving the robot backward)
    moveLeg(rightServos, 1, 20, 0, 0);
    moveLeg(leftServos, 2, -20, 0, 0);
    moveLeg(rightServos, 3, 20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }

    //Put Down Group A
    smoothLifting(leftServos, 1, -35);
    smoothLifting(rightServos, 2, 35);
    smoothLifting(leftServos, 3, -35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!backward) {
      preResetPosition();
      return;
    }
  }
}

void moveLeft() {
  Serial.println("Left");

  //Put Leg R1 In Position
  smoothLifting(rightServos, 1, -35);
  smartDelay(100);
  moveLeg(rightServos, 1, -50, 0, 0);
  smartDelay(100);
  smoothLifting(rightServos, 1, 35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!left) {
    preResetPosition();
    return;
  }

  //Put Leg L1 In Position
  smoothLifting(leftServos, 1, 35);
  smartDelay(100);
  moveLeg(leftServos, 1, 60, 0, 0);
  smartDelay(100);
  smoothLifting(leftServos, 1, -35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!left) {
    preResetPosition();
    return;
  }


  //Put Leg R3 In Position
  smoothLifting(rightServos, 3, -35);
  smartDelay(100);
  moveLeg(rightServos, 3, 30, 0, 0);
  smartDelay(100);
  smoothLifting(rightServos, 3, 35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!left) {
    preResetPosition();
    return;
  }


  //Put Leg L3 In Position
  smoothLifting(leftServos, 3, 35);
  smartDelay(100);
  moveLeg(leftServos, 3, -20, 0, 0);
  smartDelay(100);
  smoothLifting(leftServos, 3, -35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!left) {
    preResetPosition();
    return;
  }

  while (left) {
    //Lift Side Legs
    smoothLifting(leftServos, 2, 35);
    smoothLifting(rightServos, 2, -35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }

    //Move All legs to the Right (Moving the robot Leftwards)
    moveLeg(rightServos, 1, 20, 0, 0);
    moveLeg(leftServos, 1, 20, 0, 0);
    moveLeg(rightServos, 3, -20, 0, 0);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }

    //Put Down Side Legs
    smoothLifting(leftServos, 2, -35);
    smoothLifting(rightServos, 2, 35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }

    //Put Leg R1 Back In Position
    smoothLifting(rightServos, 1, -35);
    smartDelay(100);
    moveLeg(rightServos, 1, -20, 0, 0);
    smartDelay(100);
    smoothLifting(rightServos, 1, 35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }

    //Put Leg L1 Back In Position
    smoothLifting(leftServos, 1, 35);
    smartDelay(100);
    moveLeg(leftServos, 1, -20, 0, 0);
    smartDelay(100);
    smoothLifting(leftServos, 1, -35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }

    //Put Leg R3 Back In Position
    smoothLifting(rightServos, 3, -35);
    smartDelay(100);
    moveLeg(rightServos, 3, 20, 0, 0);
    smartDelay(100);
    smoothLifting(rightServos, 3, 35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }

    //Put Leg L3 In Position
    smoothLifting(leftServos, 3, 35);
    smartDelay(100);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(100);
    smoothLifting(leftServos, 3, -35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!left) {
      preResetPosition();
      return;
    }
  }
}

void moveRight() {
  Serial.println("Right");

  //Put Leg R1 In Position
  smoothLifting(rightServos, 1, -35);
  smartDelay(100);
  moveLeg(rightServos, 1, -50, 0, 0);
  smartDelay(100);
  smoothLifting(rightServos, 1, 35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!right) {
    preResetPosition();
    return;
  }

  //Put Leg L1 In Position
  smoothLifting(leftServos, 1, 35);
  smartDelay(100);
  moveLeg(leftServos, 1, 60, 0, 0);
  smartDelay(100);
  smoothLifting(leftServos, 1, -35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!right) {
    preResetPosition();
    return;
  }


  //Put Leg R3 In Position
  smoothLifting(rightServos, 3, -35);
  smartDelay(100);
  moveLeg(rightServos, 3, 30, 0, 0);
  smartDelay(100);
  smoothLifting(rightServos, 3, 35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!right) {
    preResetPosition();
    return;
  }


  //Put Leg L3 In Position
  smoothLifting(leftServos, 3, 35);
  smartDelay(100);
  moveLeg(leftServos, 3, -20, 0, 0);
  smartDelay(100);
  smoothLifting(leftServos, 3, -35);
  smartDelay(100);

  //Check If The Button Is Still Pressed
  if (!right) {
    preResetPosition();
    return;
  }

  while (right) {
    //Lift Side Legs
    smoothLifting(leftServos, 2, 35);
    smoothLifting(rightServos, 2, -35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }

    //Move All legs to the Left (Moving the robot Rightwards)
    moveLeg(rightServos, 1, -20, 0, 0);
    moveLeg(leftServos, 1, -20, 0, 0);
    moveLeg(rightServos, 3, 20, 0, 0);
    moveLeg(leftServos, 3, 20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }

    //Put Down Side Legs
    smoothLifting(leftServos, 2, -35);
    smoothLifting(rightServos, 2, 35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }

    //Put Leg R1 Back In Position
    smoothLifting(rightServos, 1, -35);
    smartDelay(100);
    moveLeg(rightServos, 1, 20, 0, 0);
    smartDelay(100);
    smoothLifting(rightServos, 1, 35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }

    //Put Leg L1 Back In Position
    smoothLifting(leftServos, 1, 35);
    smartDelay(100);
    moveLeg(leftServos, 1, 20, 0, 0);
    smartDelay(100);
    smoothLifting(leftServos, 1, -35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }

    //Put Leg R3 Back In Position
    smoothLifting(rightServos, 3, -35);
    smartDelay(100);
    moveLeg(rightServos, 3, -20, 0, 0);
    smartDelay(100);
    smoothLifting(rightServos, 3, 35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }

    //Put Leg L3 Back In Position
    smoothLifting(leftServos, 3, 35);
    smartDelay(100);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(100);
    smoothLifting(leftServos, 3, -35);
    smartDelay(100);

    //Check If The Button Is Still Pressed
    if (!right) {
      preResetPosition();
      return;
    }
  }
}

void turnRobotLeft() {
  bool groupALifted = false;
  Serial.println("TurnLeft");

  while (turnLeft) {
    if (!groupALifted) {
      //Lift Group A
      smoothLifting(leftServos, 1, 35);
      smoothLifting(rightServos, 2, -35);
      smoothLifting(leftServos, 3, 35);
      smartDelay(300);

      //Check If The Button Is Still Pressed
      if (!turnLeft) {
        preResetPosition();
        return;
      }
    }

    //Rotate Group A To The Left
    moveLeg(leftServos, 1, 20, 0, 0);
    moveLeg(rightServos, 2, 20, 0, 0);
    moveLeg(leftServos, 3, 20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Put Down Group A
    smoothLifting(leftServos, 1, -35);
    smoothLifting(rightServos, 2, 35);
    smoothLifting(leftServos, 3, -35);
    smartDelay(300);
    groupALifted = false;

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Lift Group B
    smoothLifting(rightServos, 1, -35);
    smoothLifting(leftServos, 2, 35);
    smoothLifting(rightServos, 3, -35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Rotate Back Group A (Rotating the robot to the left)
    moveLeg(leftServos, 1, -20, 0, 0);
    moveLeg(rightServos, 2, -20, 0, 0);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Rotate Group B To The Left
    moveLeg(rightServos, 1, 20, 0, 0);
    moveLeg(leftServos, 2, 20, 0, 0);
    moveLeg(rightServos, 3, 20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Put Down Group B
    smoothLifting(rightServos, 1, 35);
    smoothLifting(leftServos, 2, -35);
    smoothLifting(rightServos, 3, 35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Lift Group A
    smoothLifting(leftServos, 1, 35);
    smoothLifting(rightServos, 2, -35);
    smoothLifting(leftServos, 3, 35);
    groupALifted = true;
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }

    //Rotate Group B Back(Rotating the robot to the left)
    moveLeg(rightServos, 1, -20, 0, 0);
    moveLeg(leftServos, 2, -20, 0, 0);
    moveLeg(rightServos, 3, -20, 0, 0);

    //Check If The Button Is Still Pressed
    if (!turnLeft) {
      preResetPosition();
      return;
    }
  }
}

void turnRobotRight() {
  bool groupALifted = false;
  Serial.println("TurnRight");

  while (turnRight) {
    if (!groupALifted) {
      //Lift Group A
      smoothLifting(leftServos, 1, 35);
      smoothLifting(rightServos, 2, -35);
      smoothLifting(leftServos, 3, 35);
      smartDelay(300);

      //Check If The Button Is Still Pressed
      if (!turnRight) {
        preResetPosition();
        return;
      }
    }

    //Rotate Group A To The Right
    moveLeg(leftServos, 1, -20, 0, 0);
    moveLeg(rightServos, 2, -20, 0, 0);
    moveLeg(leftServos, 3, -20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Put Down Group A
    smoothLifting(leftServos, 1, -35);
    smoothLifting(rightServos, 2, 35);
    smoothLifting(leftServos, 3, -35);
    smartDelay(300);
    groupALifted = false;

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Lift Group B
    smoothLifting(rightServos, 1, -35);
    smoothLifting(leftServos, 2, 35);
    smoothLifting(rightServos, 3, -35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Rotate Back Group A (Rotating the robot to the right)
    moveLeg(leftServos, 1, 20, 0, 0);
    moveLeg(rightServos, 2, 20, 0, 0);
    moveLeg(leftServos, 3, 20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Rotate Group B To The Right
    moveLeg(rightServos, 1, -20, 0, 0);
    moveLeg(leftServos, 2, -20, 0, 0);
    moveLeg(rightServos, 3, -20, 0, 0);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Put Down Group B
    smoothLifting(rightServos, 1, 35);
    smoothLifting(leftServos, 2, -35);
    smoothLifting(rightServos, 3, 35);
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Lift Group A
    smoothLifting(leftServos, 1, 35);
    smoothLifting(rightServos, 2, -35);
    smoothLifting(leftServos, 3, 35);
    groupALifted = true;
    smartDelay(300);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }

    //Rotate Group B Back (Rotating the robot to the Right)
    moveLeg(rightServos, 1, 20, 0, 0);
    moveLeg(leftServos, 2, 20, 0, 0);
    moveLeg(rightServos, 3, 20, 0, 0);

    //Check If The Button Is Still Pressed
    if (!turnRight) {
      preResetPosition();
      return;
    }
  }
}

void preResetPosition() {

  //Put Leg R1 To Home Position
  smoothLifting(rightServos, 1, -35);
  smartDelay(50);
  moveLeg(rightServos, 1, 110-currentDegrees[3][0], 0, 0);
  smartDelay(100);
  moveLegToPosition(rightServos, 1, 110, 90, 95);
  smartDelay(50);

  //Put Leg L1 To Home Position
  smoothLifting(leftServos, 1, 35);
  smartDelay(50);
  moveLeg(leftServos, 1, 70-currentDegrees[0][0], 0, 0);
  smartDelay(100);
  moveLegToPosition(leftServos, 1, 70, 90, 85);
  smartDelay(50);

  //Put Leg R2 To Home Position
  smoothLifting(rightServos, 2, -35);
  smartDelay(50);
  moveLeg(rightServos, 2, 90-currentDegrees[4][0], 0, 0);
  smartDelay(100);
  moveLegToPosition(rightServos, 2, 90, 90, 95);
  smartDelay(50);

  //Put Leg L2 To Home Position
  smoothLifting(leftServos, 2, 35);
  smartDelay(50);
  moveLeg(leftServos, 1, 90-currentDegrees[1][0], 0, 0);
  smartDelay(100);
  moveLegToPosition(leftServos, 2, 90, 90, 85);
  smartDelay(50);

  //Put Leg R3 To Home Position
  smoothLifting(rightServos, 3, -35);
  smartDelay(50);
  moveLeg(rightServos, 1, 70-currentDegrees[5][0], 0, 0);
  smartDelay(100);
  moveLegToPosition(rightServos, 3, 70, 90, 95);
  smartDelay(50);

  //Put Leg L3 To Home Position
  smoothLifting(leftServos, 3, 35);
  smartDelay(50);
  moveLeg(leftServos, 1, 110-currentDegrees[2][0], 0, 0);
  smartDelay(100);
  moveLegToPosition(leftServos, 3, 110, 90, 85);
  smartDelay(50);

  resetPosition();
}

void resetPosition() {
  moveLegToPosition(leftServos, 1, 70, 90, 85);
  moveLegToPosition(rightServos, 2, 90, 90, 95);
  moveLegToPosition(leftServos, 3, 110, 90, 85);

  moveLegToPosition(rightServos, 1, 110, 90, 95);
  moveLegToPosition(leftServos, 2, 90, 90, 85);
  moveLegToPosition(rightServos, 3, 70, 90, 95);
}

void smoothLifting(Adafruit_PWMServoDriver& servos, int leg, int degrees) {
  if (degrees > 0) {
    for (int i = 1; i <= degrees; i++) {
      moveLeg(servos, leg, 0, 1, 1);
    }
  } else {
    for (int i = -1; i >= degrees; i--) {
      moveLeg(servos, leg, 0, -1, -1);
    }
  }
}

void moveLeg(Adafruit_PWMServoDriver& servos, int leg, int coxaDegrees, int femurDegrees, int tibiaDegrees) {
  if (&servos == &leftServos) {
    coxaDegrees += currentDegrees[leg - 1][0];
    femurDegrees += currentDegrees[leg - 1][1];
    tibiaDegrees += currentDegrees[leg - 1][2];
  } else {
    coxaDegrees += currentDegrees[leg + 2][0];
    femurDegrees += currentDegrees[leg + 2][1];
    tibiaDegrees += currentDegrees[leg + 2][2];
  }

  moveLegToPosition(servos, leg, coxaDegrees, femurDegrees, tibiaDegrees);
}

void moveLegToPosition(Adafruit_PWMServoDriver& servos, int leg, int coxaDegrees, int femurDegrees, int tibiaDegrees) {
  int coxa = 0 + (4 * (leg - 1));
  int femur = 1 + (4 * (leg - 1));
  int tibia = 2 + (4 * (leg - 1));

  if (&servos == &leftServos) {
    currentDegrees[leg - 1][0] = coxaDegrees;
    coxaDegrees += offset[leg - 1][0];
    currentDegrees[leg - 1][1] = femurDegrees;
    femurDegrees += offset[leg - 1][1];
    currentDegrees[leg - 1][2] = tibiaDegrees;
    tibiaDegrees += offset[leg - 1][2];
  } else {
    currentDegrees[leg + 2][0] = coxaDegrees;
    coxaDegrees += offset[leg + 2][0];
    currentDegrees[leg + 2][1] = femurDegrees;
    femurDegrees += offset[leg + 2][1];
    currentDegrees[leg + 2][2] = tibiaDegrees;
    tibiaDegrees += offset[leg + 2][2];
  }

  int legHeight;
  if (&servos == &leftServos) legHeight = -(height);
  else legHeight = height;

  servos.setPWM(coxa, 0, angleToPulse(coxaDegrees));
  servos.setPWM(femur, 0, angleToPulse(femurDegrees + legHeight));
  servos.setPWM(tibia, 0, angleToPulse(tibiaDegrees + legHeight));
}

void smartDelay(int delayParam) {
  for (int i = 0; i < delayParam / 10; i++) {
    delay(10);
    Blynk.run();
    timer.run();
  }
}