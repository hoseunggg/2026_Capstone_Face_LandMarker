#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "SUNJAE";
const char* password = "jh9989jh@@";

IPAddress local_IP(192, 168, 252, 39);
IPAddress gateway(192, 168, 252, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

int messageCount = 0;

const int servoPin1 = 25;
const int servoPin2 = 33;
const int servoPin3 = 27;
const int servoPin4 = 22;
const int servoPin5 = 32;

int servoAngle1 = 90;
int servoAngle2 = 90;
int servoAngle3 = 90;
int servoAngle4 = 90;
int servoAngle5 = 90;

void handleSet();
void handlePosition();
void printCurrentAngles();
String currentAnglesText();

void setup() {
  Serial.begin(115200);

  WiFi.config(local_IP, gateway, subnet);
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

  Serial.println("서보모터는 움직이지 않고 현재 저장 각도만 출력합니다.");
  printCurrentAngles();

  server.on("/set", handleSet);
  server.on("/position", handlePosition);
  server.begin();

  Serial.println("HTTP 서버 시작");
}

void loop() {
  server.handleClient();
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
  Serial.printf("#%d | x=%s y=%s yaw=%s pitch=%s roll=%s command=%s\n",
                messageCount,
                x.c_str(),
                y.c_str(),
                yaw.c_str(),
                pitch.c_str(),
                roll.c_str(),
                command.c_str());

  printCurrentAngles();
  server.send(200, "text/plain", currentAnglesText());
}

void handlePosition() {
  printCurrentAngles();
  server.send(200, "text/plain", currentAnglesText());
}

void printCurrentAngles() {
  Serial.println("Current servo angles");
  Serial.print("servo1 pin ");
  Serial.print(servoPin1);
  Serial.print(": ");
  Serial.println(servoAngle1);

  Serial.print("servo2 pin ");
  Serial.print(servoPin2);
  Serial.print(": ");
  Serial.println(servoAngle2);

  Serial.print("servo3 pin ");
  Serial.print(servoPin3);
  Serial.print(": ");
  Serial.println(servoAngle3);

  Serial.print("servo4 pin ");
  Serial.print(servoPin4);
  Serial.print(": ");
  Serial.println(servoAngle4);

  Serial.print("servo5 pin ");
  Serial.print(servoPin5);
  Serial.print(": ");
  Serial.println(servoAngle5);
}

String currentAnglesText() {
  String body = "";
  body += "servo1=" + String(servoAngle1) + "\n";
  body += "servo2=" + String(servoAngle2) + "\n";
  body += "servo3=" + String(servoAngle3) + "\n";
  body += "servo4=" + String(servoAngle4) + "\n";
  body += "servo5=" + String(servoAngle5) + "\n";
  return body;
}
