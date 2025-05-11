#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>


char auth[] = "e7OzjMzulxmwu4DOAZxRhZ9gOdB1ivR2";
char ssid[] = "Alex's Pixel 9";
char pass[] = "nostradamus";

BlynkTimer timer;


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
}

void loop() {
  Blynk.run();
  timer.run();
  readDht();
  readMq135();
  readMq2();
  readMq5();
  delay(100);
}