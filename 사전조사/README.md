# 자율주행 RC카 프로젝트 사전 조사

## 1. **아두이노 보드와 ESC연결**
### 1.1 **PWM 신호 제어**
   - **VXL-3m ESC**는 PWM 신호를 통해 모터의 속도를 제어할 수 있다. 이 PWM 신호는 일반적으로 **서보모터와 비슷한 방식**으로 동작한다(servo.h만 사용해도 될 듯함).
   - PWM 신호의 기본 설정은 **1000μs에서 2000μs** 범위의 펄스 폭을 사용하여 속도를 제어한다(확실치 않음):
     - **1000μs**: 정지 상태 (중립).
     - **1500μs**: 절반 속도 (정방향 또는 후방향).
     - **2000μs**: 최대 속도 (정방향).

### 1.2 **PWM 핀을 통해 ESC에 신호 전달**
   - 아두이노에서 **PWM 핀**을 사용해 ESC의 신호선에 연결하여 PWM 신호를 보내면, ESC가 모터를 제어하게 된다.

### 1.3 **전압 수준**
   - 일반적인 서보모터 제어처럼 **5V 로직**을 사용하는 경우가 많다. 아두이노는 기본적으로 **5V PWM 신호**를 발생시킨다. VXL-3m ESC는 **5V 신호를 수용**하므로 아두이노의 PWM 핀에서 바로 연결할 수 있다.
   - ESC의 매뉴얼을 확인할 때는 **PWM 신호의 입력 전압 범위**를 확인해야 하지만, VXL-3m은 보통 아두이노와 같은 5V 신호를 수용한다.

### 1.4 **PWM 신호 주기**
   - PWM 신호의 주기는 대부분의 ESC에서 **50Hz(20ms 주기)**를 사용한다. 즉, **20ms마다 1000~2000μs**의 펄스를 보냅니다. 이 펄스 폭에 따라 모터의 속도와 방향이 결정된다.
   - 아두이노의 `Servo` 라이브러리를 사용하면 PWM 신호의 주기를 자동으로 맞출 수 있기 때문에 따로 설정할 필요는 없다.

### 1.5 **VXL-3m 명세에서 확인할 부분**
   VXL-3m ESC의 매뉴얼이나 명세서에서 확인해야 할 주요 사항은 다음과 같다.:
   - **PWM 신호 전압 범위**: 아두이노에서 사용하는 5V 신호를 ESC가 수용할 수 있는지 확인한다.
   - **PWM 신호 주파수**: 대부분의 ESC는 50Hz에서 동작하지만, 일부 ESC는 400Hz와 같은 더 높은 주파수도 수용할 수 있다.
   - **모터 제어 모드**: 일부 ESC는 **브레이크 모드**, **후진 모드** 등의 추가 설정을 제공한다.. 이 설정은 차량의 주행 방향에 영향을 줄 수 있다.

### 1.6 **ESC 초기화**
   대부분의 ESC는 **초기화 과정**이 필요하다. 이 과정에서 아두이노가 ESC에 초기화 신호를 보내고, ESC는 이를 인식하고 설정된다.

```cpp
#include <Servo.h>

Servo esc;
int escPin = 9;  // PWM 핀

void setup() {
  esc.attach(escPin);  // ESC와 연결된 핀 설정
  esc.writeMicroseconds(1000);  // 초기화 신호 (모터 정지)
  delay(2000);  // 초기화 대기
}

void loop() {
  // 모터 속도 제어
  esc.writeMicroseconds(1500);  // 중간 속도
  delay(2000);  // 2초 대기

  esc.writeMicroseconds(2000);  // 최대 속도
  delay(2000);  // 2초 대기

  esc.writeMicroseconds(1000);  // 정지
  delay(2000);  // 2초 대기
}
```

이 코드는 아두이노와 VXL-3m ESC 간의 기본적인 PWM 제어를 설명한다. PWM 신호만으로 ESC를 제어할 수 있으며, 시리얼 통신은 필요하지 않다.


NVIDIA 보드와 아두이노 보드를 연결하여 시리얼 통신을 통해 데이터를 주고받는 것은 프로젝트의 중요한 부분입니다. 여기서는 **NVIDIA 보드와 아두이노 보드 간의 연결 방법과 시리얼 통신 설정**에 대해 사전 조사를 진행하겠습니다.

## 2. **NVIDIA 보드와 아두이노 보드의 연결**

### 2.1 **하드웨어 연결**
   - **UART 통신**: NVIDIA 보드와 아두이노 보드는 **UART(Universal Asynchronous Receiver-Transmitter)**를 통해 데이터를 주고받는다. NVIDIA 보드의 **TX(Transmit)** 핀과 아두이노의 **RX(Receive)** 핀을 연결하고, **RX** 핀은 **TX**로 연결한다. 
   
   **핵심 연결 핀**:
   - **NVIDIA 보드 (Jetson Nano 기준)**
     - **Pin 8**: TX (데이터 전송)
     - **Pin 10**: RX (데이터 수신)
     - **Pin 6**: GND (접지)
   - **아두이노 (Uno 기준)**
     - **Pin 0 (RX)**: 데이터 수신
     - **Pin 1 (TX)**: 데이터 전송
     - **GND**: NVIDIA 보드와 공통 접지(Ground) 연결

   **연결 요약**:
   - **NVIDIA 보드 Pin 8 (TX)** → 아두이노 Pin 0 (RX)
   - **NVIDIA 보드 Pin 10 (RX)** → 아두이노 Pin 1 (TX)
   - **GND**: 두 보드의 GND 연결

### 2.2 **전압 레벨 조정**
   - 아두이노 Uno는 **5V 로직**을 사용하지만, NVIDIA 보드는 **3.3V 로직**을 사용한다. 두 보드 간 통신이 안전하게 이루어지기 위해서는 **레벨 시프터(Level Shifter)**를 사용해 **5V 신호를 3.3V로 변환**해야 한다.
     - NVIDIA 보드의 **RX 핀**에 **5V 신호가 직접 연결**되면 손상될 수 있으므로 **레벨 시프터**를 사용하여 **아두이노에서 오는 신호를 3.3V로 변환**해야 한다.

### 2.3 **NVIDIA 보드에서 시리얼 통신 설정 (Python 사용)**

   **PySerial 라이브러리**를 사용하여 NVIDIA 보드에서 시리얼 통신을 설정할 수 있다.

   **NVIDIA 보드 코드 (Python)**:
   ```python
   import serial
   import time

   # 시리얼 포트 설정 (Jetson Nano의 경우 /dev/ttyTHS1 사용)
   ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)  # 포트, Baud rate 설정
   time.sleep(2)  # 시리얼 연결 안정 대기

   while True:
       # 아두이노로 데이터 전송
       ser.write(b'Hello Arduino!\n')  # 아두이노로 'Hello Arduino!' 메시지 전송
       time.sleep(1)

       # 아두이노로부터 데이터 수신
       if ser.in_waiting > 0:
           data = ser.readline().decode('utf-8').rstrip()  # 수신한 데이터 디코딩
           print("Received from Arduino:", data)
   ```

   - **`/dev/ttyTHS1`**는 NVIDIA 보드에서 사용되는 기본 UART 포트입니다. Jetson Nano의 경우 **ttyTHS1**을 사용하며, 다른 보드일 경우 포트 이름이 다를 수 있으니 보드의 매뉴얼을 참조해야한다.
   - **9600bps**는 일반적인 Baud rate(전송 속도)입니다. 이 속도는 양쪽 보드에서 동일하게 설정되어야 한다.

### 2.4 **아두이노에서 시리얼 통신 설정**

   아두이노에서는 기본적으로 `Serial` 객체를 사용하여 시리얼 통신을 처리한다.

   **아두이노 코드**:
   ```cpp
   void setup() {
     // 시리얼 통신 초기화 (9600bps)
     Serial.begin(9600);
     while (!Serial) {
       ; // 시리얼 포트 연결 대기
     }
     Serial.println("Arduino Ready");  // 준비 완료 메시지 전송
   }

   void loop() {
     // NVIDIA 보드로부터 데이터 수신
     if (Serial.available() > 0) {
       String receivedData = Serial.readStringUntil('\n');  // 데이터 수신
       Serial.print("Received from Jetson: ");
       Serial.println(receivedData);  // 수신 데이터 출력

       // 응답 메시지 전송
       Serial.println("Hello Jetson!");  // 응답 데이터 전송
     }
     delay(100);
   }
   ```

   - **9600bps**로 시리얼 통신을 초기화하고, NVIDIA 보드로부터 받은 데이터를 처리한ㄷ.
   - 아두이노에서 데이터를 받은 후, 다시 응답 메시지를 보내도록 설정되어 있다.

### 2.5. **통신 속도 및 Baud Rate 설정**
   - **Baud Rate**는 양쪽 보드에서 동일하게 설정해야 통신이 가능하다. 일반적으로 **9600bps**가 안정적으로 동작하지만, 더 높은 속도(115200bps)로 설정할 수도 있다.
   - 통신 속도가 너무 높을 경우, **데이터 손실이 발생할 수 있으므로** **안정적인 속도를 먼저 사용**하고, 나중에 필요하면 속도를 높이는 방식으로 진행할 수 있다.

### 2.6 **NVIDIA 보드에서 UART 포트 활성화**
   - Jetson Nano 등 일부 보드에서는 **UART 포트**가 기본적으로 비활성화되어 있을 수 있으므로, 이를 활성화해야 한다. 이를 위해 설정 파일을 수정해야 할 수 있다.
   
   **Jetson Nano의 UART 설정 방법**:
   - `/boot/extlinux/extlinux.conf` 파일을 수정하여 `cbootargs` 항목에서 **console=none**을 추가하여 UART 포트를 활성화할 수 있다.

   ```bash
   sudo nano /boot/extlinux/extlinux.conf
   # cbootargs에서 console=none 추가
   ```

   이렇게 설정한 후, 보드를 재부팅하면 UART 포트를 사용할 수 있다.

### 2.7 **테스트 및 디버깅**
   - 두 보드 간의 통신이 정상적으로 이루어지는지 테스트하려면, **간단한 메시지**를 주고받는 테스트를 먼저 수행해야한다.
   - 양방향 통신이 정상적으로 이루어지면, 이후 모터 제어 신호나 딥러닝 추론 결과를 아두이노로 전송하여 제어할 수 있다.

---

### 결론
- **NVIDIA 보드와 아두이노 보드 간 연결**은 **UART 시리얼 통신**을 통해 이루어진다.
- 하드웨어 연결 시 **전압 레벨 차이**(NVIDIA: 3.3V, 아두이노: 5V)를 해결하기 위해 **레벨 시프터**를 사용해야 한다.
- **Baud Rate**는 양쪽에서 동일하게 설정해야 하며, 일반적으로 **9600bps**가 사용된다.
- 통신이 정상적으로 이루어지면, **NVIDIA 보드에서 딥러닝 처리 결과를 아두이노로 전송**하여 모터 및 서보모터를 제어하는 단계로 넘어갈 수 있다.

이 단계에서 두 보드 간 통신이 잘 이루어진다면, 이후 전체 시스템을 통합하여 본격적으로 자율주행 RC카 시스템을 개발할 수 있다.

### 3. UDP 소켓 통신의 개요
UDP는 **연결 지향**이 아닌 **비연결성 통신**으로, 데이터그램을 빠르게 주고받을 수 있다. **송신자**는 데이터를 수신자의 주소로 보내고, **수신자**는 해당 포트에서 데이터를 수신한다. 이 과정에서 **데이터가 손실**될 수는 있지만, 네트워크 지연이 적고 빠른 전송 속도를 보장한다.

### 3.1 **라즈베리파이에서 카메라 영상 캡처 및 전송 (UDP 송신자)**

**라즈베리파이**는 **UDP 송신자**로 설정되어 카메라로 캡처한 프레임을 **NVIDIA 보드**로 전송한다.

#### 과정:
1. 라즈베리파이가 **카메라 모듈**을 통해 영상을 실시간으로 캡처한다.
2. 캡처된 프레임을 **JPEG** 또는 **PNG** 형식으로 인코딩하여 전송할 수 있는 작은 데이터 패킷으로 만든다.
3. UDP를 사용하여 해당 데이터를 **NVIDIA 보드**로 전송한다.

**예시 코드 (라즈베리파이 측, Python UDP 송신자)**:
```python
import socket
import cv2
import pickle
import struct

# UDP 소켓 설정
server_ip = "192.168.1.20"  # NVIDIA 보드 IP 주소
port = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 카메라 모듈로 영상 캡처
cap = cv2.VideoCapture(0)  # Raspberry Pi 카메라 사용

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 프레임을 JPEG로 인코딩
    ret, buffer = cv2.imencode('.jpg', frame)
    data = pickle.dumps(buffer)

    # 데이터 크기 전송
    message_size = struct.pack("L", len(data))
    
    # UDP 소켓을 통해 데이터 전송 (NVIDIA 보드로)
    sock.sendto(message_size + data, (server_ip, port))

cap.release()
sock.close()
```

#### 주요 단계:
- **cv2.VideoCapture(0)**: 카메라 모듈을 초기화하고 실시간으로 영상을 캡쳐한다.
- **cv2.imencode('.jpg', frame)**: 각 프레임을 JPEG로 인코딩하여 전송할 수 있는 크기로 줄인다.
- **sock.sendto(data, (server_ip, port))**: 인코딩된 프레임 데이터를 **NVIDIA 보드의 IP 주소**로 전송한다.

### 3.2 **NVIDIA 보드에서 영상 수신 및 처리 (UDP 수신자)**

**NVIDIA 보드**는 **UDP 수신자**로 설정되어 라즈베리파이에서 전송된 프레임 데이터를 받아서 실시간으로 처리한다.

#### 과정:
1. **UDP 소켓**을 통해 영상을 수신한다.
2. 수신한 데이터를 **디코딩**하여 원래의 이미지로 복원한다.
3. 복원된 이미지를 **딥러닝 모델** 또는 **컴퓨터 비전 알고리즘**에 입력해 처리할 수 있다.

**예시 코드 (NVIDIA 보드 측, Python UDP 수신자)**:
```python
import socket
import cv2
import pickle
import struct

# UDP 소켓 설정
port = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", port))

while True:
    # 데이터 수신
    message, client_address = sock.recvfrom(4096)

    # 데이터 크기 추출
    message_size = struct.unpack("L", message[:struct.calcsize("L")])[0]
    data = message[struct.calcsize("L"):]

    # 데이터가 손상되지 않도록 남은 데이터 수신
    while len(data) < message_size:
        message, _ = sock.recvfrom(4096)
        data += message

    # 수신한 데이터를 디코딩
    frame_data = pickle.loads(data)
    frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

    # 화면에 표시 (또는 딥러닝 모델로 처리)
    cv2.imshow("Received Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

sock.close()
cv2.destroyAllWindows()
```

#### 주요 단계:
- **sock.recvfrom(4096)**: 라즈베리파이에서 전송한 데이터를 수신한다.
- **cv2.imdecode(frame_data, cv2.IMREAD_COLOR)**: 전송받은 프레임을 복원하여 이미지를 표시하거나 딥러닝 모델에 사용할 수 있다.

### 3.3 **네트워크 속도 및 설정**
- **대역폭**: 영상의 해상도와 전송 속도에 따라 UDP 패킷의 크기를 적절히 조정해야 합니다. 일반적으로, **640x480 해상도**에서 **30fps**로 영상을 전송하려면 높은 대역폭이 필요하다. 이를 위해 **JPEG 인코딩**을 사용해 데이터 크기를 줄이는 것이 좋다.
- **패킷 손실**: UDP는 데이터 손실에 민감하기 때문에, 중요한 데이터 전송에 적합하지 않다. 하지만 영상 전송의 경우, 약간의 프레임 손실은 허용될 수 있다.
- **NVIDIA 보드와 라즈베리파이 간 네트워크 환경**: 같은 **로컬 네트워크**에 연결되어 있거나 **Wi-Fi**로 연결된 경우 안정적인 통신이 가능하다.

### 3.4 **UDP 통신의 장단점**
#### 장점:
- **낮은 지연**: 실시간으로 영상을 빠르게 전송할 수 있습니다. 특히 실시간 제어가 중요한 상황에서 유리하다.
- **연결 설정이 간단**: 연결을 확립할 필요 없이 데이터를 바로 전송할 수 있어 네트워크 설정이 간단하다.

#### 단점:
- **데이터 손실 가능성**: 네트워크 상태가 좋지 않거나 데이터가 과부하 상태일 때 일부 프레임이 손실될 수 있다.
- **데이터 무결성 보장 없음**: TCP처럼 데이터의 순서를 보장하지 않기 때문에, 데이터의 무결성을 보장할 수 없다.

---

### 결론

**UDP 소켓 통신**을 사용하여 라즈베리파이에서 NVIDIA 보드로 실시간 영상을 전송하는 과정은 비교적 간단하게 설정할 수 있으며, **실시간성**이 중요한 프로젝트에 적합하다. **빠른 전송**과 **낮은 지연**을 제공하지만, **데이터 손실** 가능성이 있다는 점을 염두에 두어야 한다. 영상 전송을 실시간으로 처리해야 하는 자율주행 시스템에서는 UDP가 적합할 수 있으며, 전체 시스템이 통합되면 **딥러닝 모델**이나 **컴퓨터 비전 알고리즘**과 연계하여 주행 제어에 활용할 수 있다.

https://www.youtube.com/watch?v=HEEullyoVW8&ab_channel=%EA%B3%B5%EB%8C%80%EC%84%A0%EB%B0%B0