#include <SoftwareSerial.h> // Serial Communication

// L298N motor driver
const int LS = 8;       // IR sensor on the left
const int RS = 12;      // IR sensor on the right
const int LM1 = 11;      // Left motor input 1
const int LM2 = 6;     // Left motor input 2
const int RM1 = 9;      // Right motor input 1
const int RM2 = 10;     // Right motor input 2
const int PWM1 = 3;     // PWM for the left motor speed
const int PWM2 = 5;     // PWM for the right motor speed
int Speed = 150;
int station = 0;
boolean statestation = HIGH;
int Push = 44;

// Button
int buttonState = HIGH;
int lastButtonState = HIGH;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 20;// the debounce time; increase if the output flickers


// Ultrasonic sensors
const int ECHO1 = A1;   // Echo pin for Ultrasonic sensor 1
const int TRIG1 = A0;   // Trigger pin for Ultrasonic sensor 1
const int ECHO2 = A3;   // Echo pin for Ultrasonic sensor 2
const int TRIG2 = A2;   // Trigger pin for Ultrasonic sensor 2
long duration1, duration2, cm1, cm2, distance;



// IR Sensor counter
const int IR_COUNT_UP = 2;
const int IR_COUNT_DOWN = 4;
int count = 0;
bool state_up = true;
bool state_down = true;

SoftwareSerial esp_serial(13, 7);  // RX, TX

void setup()
{
  // IR Sensor counter
  pinMode(IR_COUNT_UP, INPUT);
  pinMode(IR_COUNT_DOWN, INPUT);

  // Motor driver pins
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(ECHO1, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO2, INPUT); //สั่งให้ขา echo ใช้งานเป็น input
  pinMode(TRIG2, OUTPUT);

  // Set motor speed
  analogWrite(PWM1, 150);
  analogWrite(PWM2, 150);//  analogWrite(PWM1, 150);
  //  analogWrite(PWM2, 150);

  // Serial communication setup
  Serial.begin(115200);
  esp_serial.begin(4800);
}

void moveForward()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}

void turnRight()
{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}

void turnLeft()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}

void Stop()
{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
  station++;
}


void CountUP()
{
  count++;
  state_up = false;
  if (count > 10)
  {
    count = 10;
  }
}

void CountDown()
{
  count--;
  state_down = false;
  if (count < 0)
  {
    count = 0;
  }
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}


void Ultrasonic1() {

  pinMode(ECHO1, OUTPUT);
  digitalWrite(ECHO1, 0);
  delayMicroseconds(2);
  digitalWrite(ECHO1, 1);
  delayMicroseconds(5);
  digitalWrite(ECHO1, 0);
  pinMode(TRIG1, INPUT);
  duration1 = pulseIn(TRIG1, 1);
  cm1 = microsecondsToCentimeters(duration1);
  Serial.print(cm1);
  Serial.print("cm1:");
  Serial.println();

}

void Ultrasonic2() {

  pinMode(ECHO2, OUTPUT);
  digitalWrite(ECHO2, 0);
  delayMicroseconds(2);
  digitalWrite(ECHO2, 1);
  delayMicroseconds(5);
  digitalWrite(ECHO2, 0);
  pinMode(TRIG2, INPUT);
  duration2 = pulseIn(TRIG2, 1);
  cm2 = microsecondsToCentimeters(duration2);
  //  Serial.print(count);
  //  Serial.print("cm2:");
  //  Serial.println();


}

void esp() {
  // Send count to ESP8266
  esp_serial.print(count);
  esp_serial.print(" ");
  esp_serial.print(station);
  esp_serial.print("\n");
  // รับค่า ESP 8266
  while (esp_serial.available() > 0)
  {
    boolean val = esp_serial.parseFloat();
    if (esp_serial.read() == '\n')
    {
      Push = val;
      Serial.println(Push);
    }
  }
}
boolean Debounce(int button)

{
  int reading = digitalRead(button);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        lastButtonState = reading;
        return (LOW);
      }
    }
  }

  lastButtonState = reading;
  return (HIGH);
}

void loop()
{
  //Ultrasonic sensors
  Ultrasonic1();
  Ultrasonic2();

  if ( cm1 <= 20 ) {
    Stop();
  }
  else if (digitalRead(RS) == LOW && digitalRead(LS) == LOW)
  {
    moveForward();
  }
  else if (digitalRead(RS) == HIGH && digitalRead(LS) == LOW)
  {
    turnRight();
  }
  else if (digitalRead(RS) == LOW && digitalRead(LS) == HIGH)
  {
    turnLeft();
  }
  else if (digitalRead(RS) == HIGH && digitalRead(LS) == HIGH)
  {
    Stop();
  }


  // ir count
  if (!digitalRead(IR_COUNT_UP) && state_up)
  {
    CountUP();
  }
  if (digitalRead(IR_COUNT_UP))
  {
    state_up = true;
  }

  if (!digitalRead(IR_COUNT_DOWN) && state_down)
  {
    CountDown();
  }
  if (digitalRead(IR_COUNT_DOWN))
  {
    state_down = true;
  }

  if (Debounce(statestation) == LOW)
  {
    station ++ ;
  }

  Serial.print("Station is");
  Serial.println(station);
  esp();

}