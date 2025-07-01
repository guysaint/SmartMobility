#include <SoftwareSerial.h>


// 핀 설정
#define TRIG 8
#define ECHO 9
#define AA 10
#define AB 11
#define BA 5
#define BB 6
#define BUTTON_PIN 7

// PID 관련 변수
float Kp = 3.0;
float Ki = 0.1;
float Kd = 2.0;
float previous_error = 0;
float integral = 0;
const float targetDistance = 10.0;

// 버튼 제어 변수
bool isRunning = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// 블루투스 제어
String command = "";
bool isAutoMode = false;

// 함수 원형 선언
long getDistance();
void goForwardPWM(int speed);
void goBackwardPWM(int speed);
void stopMotors();

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(AA, OUTPUT);
  pinMode(AB, OUTPUT);
  pinMode(BA, OUTPUT);
  pinMode(BB, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);
 
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    command = String(c);
    Serial.print("Serial command: ");
    Serial.println(command);

    if (command == "A") {
      isAutoMode = true;
      isRunning = true;
      Serial.println("🔄 Auto mode: ON");
    } else if (command == "M") {
      isAutoMode = false;
      isRunning = false;
      Serial.println("⛔ Auto mode: OFF");
    } else if (command == "F") {
      goForwardPWM(100);
      Serial.println("▶️ Manual forward");
    } else if (command == "B") {
      goBackwardPWM(100);
      Serial.println("◀️ Manual backward");
    } else if (command == "S") {
      stopMotors();
      Serial.println("🛑 Manual stop");
    }
  }

  // --- 버튼 입력 ---
// --- 버튼 입력 ---
int reading = digitalRead(BUTTON_PIN);
static int lastStableState = HIGH;

if (reading != lastStableState) {
  lastDebounceTime = millis();
}

if ((millis() - lastDebounceTime) > debounceDelay) {
  if (reading != lastButtonState) {
    lastButtonState = reading;

    // LOW → HIGH 변화일 때만 토글
    if (lastButtonState == HIGH) {
      isRunning = !isRunning;
      isAutoMode = isRunning;

      Serial.print("Button toggle → isRunning: ");
      Serial.println(isRunning);

      if (isRunning) {
        Serial.println("🔄 Auto mode: ON");
      } else {
        Serial.println("⛔ Auto mode: OFF");
      }
    }
  }
}

lastStableState = reading;


  // --- PID 주행 ---
  if (isRunning && isAutoMode) {
    float distance = getDistance();
    float error = distance - targetDistance;
    integral += error;
    float derivative = error - previous_error;
    float output = Kp * error + Ki * integral + Kd * derivative;

    if (abs(error) < 2.0) {
      stopMotors();
    } else {
      int speed = constrain(abs(output), 70, 150);
      if (error > 0) goForwardPWM(speed);
      else goBackwardPWM(speed);
    }
    previous_error = error;
  } else if (!isAutoMode) {
    // 수동 모드에서는 블루투스 명령에 따라 동작 (이미 처리됨)
  } else {
    stopMotors();
  }

  delay(100);
}

// --- 거리 측정 함수 ---
long getDistance() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2;
}

// --- 전진 ---
void goForwardPWM(int speed) {
  speed = constrain(speed, 70, 150);
  analogWrite(AA, speed);
  analogWrite(AB, 0);
  analogWrite(BA, speed);
  analogWrite(BB, 0);
}

// --- 후진 ---
void goBackwardPWM(int speed) {
  speed = constrain(speed, 70, 150);
  analogWrite(AA, 0);
  analogWrite(AB, speed);
  analogWrite(BA, 0);
  analogWrite(BB, speed);
}

// --- 정지 ---
void stopMotors() {
  analogWrite(AA, 0);
  analogWrite(AB, 0);
  analogWrite(BA, 0);
  analogWrite(BB, 0);
}
