#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char* ssid = "SUNJAE";
const char* password = "jh9989jh@@";

IPAddress local_IP(192, 168, 173, 55);
IPAddress gateway(192, 168, 173, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

int messageCount = 0;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int servoPin1 = 18;
const int servoPin2 = 19;
const int servoPin3 = 21;
const int servoPin4 = 22;

int servoAngle1 = 90;
int servoAngle2 = 90;
int servoAngle3 = 90;
int servoAngle4 = 90;

const int PAN_CENTER = 90;
const int PAN_MIN = 20;
const int PAN_MAX = 160;
const int TILT_CENTER = 90;
const int TILT_MIN = 40;
const int TILT_MAX = 140;

const int FACE_DEADZONE_X = 25;
const int FACE_DEADZONE_Y = 20;
const float PAN_PIXEL_GAIN = 0.035;
const float TILT_PIXEL_GAIN = 0.035;
const float YAW_GAIN = 10.0;
const float PITCH_GAIN = 8.0;
const int MAX_STEP_PER_MESSAGE = 4;

const bool INVERT_PAN = false;
const bool INVERT_TILT = false;

void handleSet();
void setServo1Angle(int angle);
void setServo2Angle(int angle);
void setServo3Angle(int angle);
void setServo4Angle(int angle);
void lookLeft();
void lookCenter();
void lookRight();
void applyCommand(String command);
void setServo1ByYaw(float yaw);
void updateFaceTracking(String xArg, String yArg, String yawArg, String pitchArg);
bool isValidNumberArg(String value);
int moveToward(int current, int target, int maxStep);


void setup() {
  Serial.begin(115200);

  WiFi.config(local_IP, gateway, subnet);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  servo1.attach(servoPin1, 500, 2400);
  servo2.attach(servoPin2, 500, 2400);
  servo3.attach(servoPin3, 500, 2400);
  servo4.attach(servoPin4, 500, 2400);

  servo1.write(servoAngle1);
  servo2.write(servoAngle2);
  servo3.write(servoAngle3);
  servo4.write(servoAngle4);

  WiFi.begin(ssid, password);

  Serial.print("WiFi 연결중");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi 연결 완료");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  server.on("/set", handleSet);
  server.begin();

  Serial.println("HTTP 서버 시작");
}

void loop() {
  server.handleClient();
}

// Low level: write a raw angle to each servo.
void setServo1Angle(int angle) {
  servoAngle1 = constrain(angle, PAN_MIN, PAN_MAX);
  servo1.write(servoAngle1);
  Serial.print("servo1 angle: ");
  Serial.println(servoAngle1);
}

void setServo2Angle(int angle) {
  servoAngle2 = constrain(angle, TILT_MIN, TILT_MAX);
  servo2.write(servoAngle2);
  Serial.print("servo2 angle: ");
  Serial.println(servoAngle2);
}

void setServo3Angle(int angle) {
  servoAngle3 = constrain(angle, 0, 180);
  servo3.write(servoAngle3);
  Serial.print("servo3 angle: ");
  Serial.println(servoAngle3);
}

void setServo4Angle(int angle) {
  servoAngle4 = constrain(angle, 0, 180);
  servo4.write(servoAngle4);
  Serial.print("servo4 angle: ");
  Serial.println(servoAngle4);
}

// Intermediate level: machine motions built from low-level servo moves.
void lookLeft() {
  setServo1Angle(PAN_MIN);
}

void lookCenter() {
  setServo1Angle(PAN_CENTER);
  setServo2Angle(TILT_CENTER);
}

void lookRight() {
  setServo1Angle(PAN_MAX);
}

// High level: convert interpreted input into machine motions.
void applyCommand(String command) {
  command.trim();

  if (command == "left") {
    lookLeft();
  } else if (command == "center") {
    lookCenter();
  } else if (command == "right") {
    lookRight();
  }
}

void setServo1ByYaw(float yaw) {
  if (yaw < 0) {
    lookLeft();
  } else if (yaw > 0) {
    lookRight();
  } else {
    lookCenter();
  }
}

bool isValidNumberArg(String value) {
  value.trim();
  return value.length() > 0 && value != "none" && value != "null" && value != "NaN";
}

int moveToward(int current, int target, int maxStep) {
  if (target > current + maxStep) {
    return current + maxStep;
  }

  if (target < current - maxStep) {
    return current - maxStep;
  }

  return target;
}

void updateFaceTracking(String xArg, String yArg, String yawArg, String pitchArg) {
  float panCorrection = 0.0;
  float tiltCorrection = 0.0;

  if (isValidNumberArg(xArg)) {
    float errorX = xArg.toFloat();

    if (abs(errorX) > FACE_DEADZONE_X) {
      panCorrection += errorX * PAN_PIXEL_GAIN;
    }
  }

  if (isValidNumberArg(yArg)) {
    float errorY = yArg.toFloat();

    if (abs(errorY) > FACE_DEADZONE_Y) {
      tiltCorrection += errorY * TILT_PIXEL_GAIN;
    }
  }

  if (isValidNumberArg(yawArg)) {
    panCorrection += yawArg.toFloat() * YAW_GAIN;
  }

  if (isValidNumberArg(pitchArg)) {
    tiltCorrection += pitchArg.toFloat() * PITCH_GAIN;
  }

  if (INVERT_PAN) {
    panCorrection *= -1.0;
  }

  if (INVERT_TILT) {
    tiltCorrection *= -1.0;
  }

  int targetPan = constrain(servoAngle1 + (int)round(panCorrection), PAN_MIN, PAN_MAX);
  int targetTilt = constrain(servoAngle2 + (int)round(tiltCorrection), TILT_MIN, TILT_MAX);

  int nextPan = moveToward(servoAngle1, targetPan, MAX_STEP_PER_MESSAGE);
  int nextTilt = moveToward(servoAngle2, targetTilt, MAX_STEP_PER_MESSAGE);

  if (nextPan != servoAngle1) {
    setServo1Angle(nextPan);
  }

  if (nextTilt != servoAngle2) {
    setServo2Angle(nextTilt);
  }
}


void handleSet() {
  messageCount++;

  String x = server.arg("x");
  String y = server.arg("y");
  String yaw = server.arg("yaw");
  String pitch = server.arg("pitch");
  String roll = server.arg("roll");
  String command = server.arg("command");

  Serial.println();
  Serial.println("================================");
  Serial.print("MESSAGE #");
  Serial.println(messageCount);
  Serial.printf("#%d | x=%s y=%s yaw=%s pitch=%s roll=%s command=%s\n", messageCount, x.c_str(), y.c_str(), yaw.c_str(), pitch.c_str(), roll.c_str(), command.c_str());

  if (command.length() > 0) {
    applyCommand(command);
  }

  if (isValidNumberArg(x) || isValidNumberArg(y) || isValidNumberArg(yaw) || isValidNumberArg(pitch)) {
    updateFaceTracking(x, y, yaw, pitch);
  }

  if (server.arg("servo1").length() > 0) {
    setServo1Angle(server.arg("servo1").toInt());
  }

  if (server.arg("servo2").length() > 0) {
    setServo2Angle(server.arg("servo2").toInt());
  }

  if (server.arg("servo3").length() > 0) {
    setServo3Angle(server.arg("servo3").toInt());
  }

  if (server.arg("servo4").length() > 0) {
    setServo4Angle(server.arg("servo4").toInt());
  }

  server.send(200, "text/plain", "OK");
}
