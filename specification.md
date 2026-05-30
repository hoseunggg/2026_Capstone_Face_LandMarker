# Specification

## 프로젝트 개요

이 프로젝트는 카메라로 사용자의 얼굴 위치와 머리 방향을 인식하고, 그 값을 ESP32로 전송해 서보 모터를 움직이는 얼굴 추적 로봇 시스템이다.

현재 구현은 크게 세 부분으로 나뉜다.

- 웹 기반 얼굴 인식 페이지: 브라우저 카메라와 MediaPipe Face Landmarker로 얼굴 좌표와 yaw/pitch/roll 유사 값을 계산한다.
- Mac 중계 서버: 브라우저에서 받은 값을 HTTPS로 수신한 뒤 ESP32의 HTTP 서버로 전달한다.
- ESP32 펌웨어: Wi-Fi에 연결된 뒤 `/set` 요청을 받아 서보 모터를 제어한다.

Android 앱 코드도 포함되어 있으며, CameraX와 MediaPipe Face Landmarker로 얼굴 랜드마크를 감지하는 구조다. 다만 현재 Android 앱의 ESP32 통신 API는 ESP32 펌웨어의 실제 엔드포인트와 맞지 않아 추가 수정이 필요하다.

## 현재 동작 흐름

### 1. 웹 얼굴 인식

파일: `Code/index.html`

브라우저에서 카메라 영상을 받아 MediaPipe Face Landmarker를 실행한다. 얼굴이 감지되면 다음 랜드마크를 사용한다.

- 왼쪽 눈 바깥쪽: landmark 33
- 오른쪽 눈 바깥쪽: landmark 263
- 코 끝: landmark 1
- 입 중심: landmark 13

이 값으로 화면 중심 대비 얼굴 중심 오차와 머리 방향에 가까운 값을 계산한다.

- `errorX`: 화면 중심과 얼굴 중심의 x축 차이
- `errorY`: 화면 중심과 얼굴 중심의 y축 차이
- `yaw`: 코 위치와 양 눈 중심의 차이로 계산한 좌우 회전 유사 값
- `pitch`: 코와 입/눈의 상대 위치로 계산한 상하 회전 유사 값
- `roll`: 양쪽 눈의 기울기로 계산한 머리 기울기 각도

자동 전송이 켜져 있으면 지정한 주기마다 다음 형식으로 서버에 GET 요청을 보낸다.

```text
https://<target-ip>/set?x=<errorX>&y=<errorY>&yaw=<yaw>&pitch=<pitch>&roll=<roll>
```

### 2. Mac 중계 서버

파일: `Code/server.py`

Flask 서버가 HTTPS로 `/set` 요청을 받는다. 브라우저가 직접 ESP32에 HTTPS 요청을 보내기 어렵기 때문에, Mac 서버가 중간에서 요청을 받아 ESP32의 HTTP 서버로 전달한다.

수신 파라미터:

- `x`
- `y`
- `yaw`
- `pitch`
- `roll`
- `command`
- `servo1`
- `servo2`
- `servo3`
- `servo4`

`x`, `y`, `yaw`, `pitch`, `roll`은 기본값을 채워 ESP32로 전달한다. `command`, `servo1`, `servo2`, `servo3`, `servo4`는 값이 들어온 경우에만 ESP32로 전달한다.

전달 대상:

```text
http://192.168.173.55/set
```

서버 실행 시 `cert.pem`, `key.pem`을 사용해 HTTPS 서버를 `0.0.0.0:8443`에서 연다.

### 3. ESP32 펌웨어

파일: `Code/EC2_0512_ver/EC2_0512_ver.ino`

현재 원본 스케치는 백업본처럼 유지하고, 모듈화 실험 버전은 `Code/EC2_0512_modular/`에 별도로 둔다.

모듈화 버전 파일 구성:

- `EC2_0512_modular.ino`: Wi-Fi 연결, 서보 초기화, HTTP 서버 등록, `setup()`, `loop()`
- `global.h`: Wi-Fi 설정, 서버 객체, 서보 객체, 핀 번호, 현재 각도 전역값
- `SetupModules.ino`: 서보 초기화와 Wi-Fi 연결
- `LowLevel.ino`: 각 서보를 입력 각도로 직접 이동시키는 low level 제어
- `MachineMotion.ino`: `lookLeft()`, `lookCenter()`, `lookRight()` 같은 기계 동작
- `InputControl.ino`: `command`, `yaw` 같은 입력을 기계 동작으로 변환
- `HttpHandlers.ino`: `/set` 요청 파싱, 로그 출력, 입력별 제어 함수 호출

ESP32는 고정 IP로 Wi-Fi에 연결한 뒤 80번 포트에서 HTTP 서버를 실행한다.

현재 네트워크 설정:

- ESP32 IP: `192.168.173.55`
- Gateway: `192.168.173.1`
- Subnet: `255.255.255.0`
- Wi-Fi SSID와 비밀번호는 코드에 하드코딩되어 있음

서보 모터 설정:

- Servo 1: GPIO 18
- Servo 2: GPIO 19
- Servo 3: GPIO 21
- Servo 4: GPIO 22
- 초기 각도: 모두 90도
- PWM 주파수: 50 Hz
- 펄스 폭: 500-2400 us

ESP32는 `/set` 요청을 받으면 `x`, `y`, `yaw`, `pitch`, `roll`, `command` 값을 읽고 Serial Monitor에 출력한다. 현재 실제 제어에 사용하는 값은 `yaw`, `command`, `servo1`-`servo4` 직접 각도 파라미터다.

펌웨어 제어 코드는 세 레이어로 나누는 방향으로 정리한다.

- Low level: 각 서보를 특정 각도로 직접 이동시킨다.
- Intermediate level: 기계 동작 단위로 서보 움직임을 묶는다.
- High level: 입력값이나 명령어를 해석해 기계 동작으로 변환한다.

현재 low level 함수:

- `setServo1Angle(angle)`: Servo 1을 입력 각도로 이동
- `setServo2Angle(angle)`: Servo 2를 입력 각도로 이동
- `setServo3Angle(angle)`: Servo 3을 입력 각도로 이동
- `setServo4Angle(angle)`: Servo 4를 입력 각도로 이동

현재 intermediate level 함수:

- `lookLeft()`: Servo 1을 0도로 이동
- `lookCenter()`: Servo 1을 90도로 이동
- `lookRight()`: Servo 1을 180도로 이동

현재 high level 함수:

- `applyCommand(command)`: `left`, `center`, `right` 명령을 기계 동작으로 변환
- `setServo1ByYaw(yaw)`: yaw 값을 좌/중앙/우 동작으로 변환

현재 yaw 제어 방식:

- `yaw < 0`이면 `lookLeft()` 실행
- `yaw == 0`이면 `lookCenter()` 실행
- `yaw > 0`이면 `lookRight()` 실행

현재 command 제어 방식:

- `command=left`이면 `lookLeft()` 실행
- `command=center`이면 `lookCenter()` 실행
- `command=right`이면 `lookRight()` 실행

개별 서보 직접 제어 방식:

```text
/set?servo1=90
/set?servo2=120
/set?servo3=45
/set?servo4=135
```

요청 처리가 끝나면 ESP32는 다음 응답을 반환한다.

```text
OK
```

## Android 앱

파일:

- `app_android/app/src/main/java/edu/skku/cs/webcamera/MainActivity.java`
- `app_android/app/src/main/java/edu/skku/cs/webcamera/ESP32Communicator.java`

Android 앱은 CameraX로 카메라 프레임을 가져오고, MediaPipe Face Landmarker로 얼굴 랜드마크를 감지한다. 현재 UI에는 눈, 코, 입 위치와 얼굴 감지 여부가 표시된다.

현재 Android 앱에서 ESP32로 보내는 요청:

- 상태 확인: `GET /api/status`
- 제어 전송: `POST /api/control`
- 전송 데이터: JSON body

하지만 ESP32 펌웨어는 현재 다음 API만 제공한다.

- `GET /set?x=...&y=...&yaw=...&pitch=...&roll=...`

따라서 Android 앱을 실제 ESP32와 연결하려면 `ESP32Communicator`의 API 경로와 전송 방식을 ESP32 펌웨어에 맞추거나, 반대로 ESP32 펌웨어에 `/api/status`, `/api/control` 처리를 추가해야 한다.

## 현재 구현 상태

- 웹 페이지에서 얼굴 랜드마크 감지 가능
- 웹 페이지에서 얼굴 중심 오차와 yaw/pitch/roll 유사 값 계산 가능
- Flask 중계 서버를 통해 ESP32 `/set` 엔드포인트로 값 전달 가능
- ESP32에서 `/set` 요청 수신 및 Serial 출력 가능
- ESP32에서 yaw 값 기준으로 Servo 1 좌/중앙/우 제어 가능
- ESP32에서 command 값 기준으로 Servo 1 좌/중앙/우 제어 가능
- ESP32에서 `servo1`-`servo4` 파라미터로 각 서보 직접 각도 제어 가능
- Android 앱은 얼굴 감지 UI까지 구현되어 있으나 ESP32 펌웨어와 통신 규격이 불일치

## 보완 필요 사항

- 실제 기구 기준으로 Servo 1, 2, 3, 4의 안전 각도 범위를 정해야 한다.
- yaw 값을 단순 좌/우 방향이 아니라 연속적인 서보 각도로 매핑할지 결정해야 한다.
- `x`, `y`, `pitch`, `roll` 값을 어떤 서보에 어떻게 반영할지 정해야 한다.
- `left`, `center`, `right` 외에 음성 명령으로 사용할 명령어 목록을 정해야 한다.
- Android 앱과 ESP32 사이의 API 규격을 통일해야 한다.
- Wi-Fi SSID/비밀번호를 코드에 직접 넣는 방식을 개선하는 것이 좋다.
- ESP32 연결 실패, 잘못된 파라미터, 얼굴 미감지 상태에 대한 예외 처리를 정리해야 한다.
