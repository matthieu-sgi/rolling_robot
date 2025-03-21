#include <Arduino.h>
#include <pthread.h>
#include <WiFi.h>
#include <secrets.h>

// Definition of pinout
#define RIGHT_MTR_FRWD 36
#define RIGHT_MTR_BCKWD 37
#define PWM_RIGHT_MTR 1

#define LEFT_MTR_FRWD 45
#define LEFT_MTR_BCKWD 35
#define PWM_LEFT_MTR 47

#define MOTOR_STANDBY 46

#define LIDAR_PWM 21
#define LIDAR_TX 14

HardwareSerial lidarSerial(2);
HardwareSerial usbSerial(0);

const char *ssid = SSID;
const char *password = PASSWORD;

WiFiClient tcpClient;
const char *host = "192.168.200.105"; // IP of the tcpClient(Computer with viz)
const int port = 3000;

// Following the datasheet of the Lidar
#define ANGLE_PER_FRAME 12
#define HEADER 0x54

struct LidarPointStructDef
{
  uint16_t distance;
  uint8_t confidence;
};

typedef struct __attribute__((packed))
{
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[ANGLE_PER_FRAME];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

void forward()
{
  digitalWrite(LEFT_MTR_FRWD, HIGH);
  digitalWrite(RIGHT_MTR_FRWD, HIGH);

  digitalWrite(LEFT_MTR_BCKWD, LOW);
  digitalWrite(RIGHT_MTR_BCKWD, LOW);
}

bool toggle = false;
void toggle_standby()
{
  toggle = !toggle;
  digitalWrite(MOTOR_STANDBY, toggle);
  analogWrite(PWM_LEFT_MTR, 150);
  analogWrite(PWM_RIGHT_MTR, 150);
}

void left()
{
  digitalWrite(LEFT_MTR_FRWD, LOW);
  digitalWrite(RIGHT_MTR_FRWD, HIGH);

  digitalWrite(LEFT_MTR_BCKWD, LOW);
  digitalWrite(RIGHT_MTR_BCKWD, LOW);
}

void right()
{
  digitalWrite(LEFT_MTR_FRWD, HIGH);
  digitalWrite(RIGHT_MTR_FRWD, LOW);

  digitalWrite(LEFT_MTR_BCKWD, LOW);
  digitalWrite(RIGHT_MTR_BCKWD, LOW);
}

void backward()
{
  digitalWrite(LEFT_MTR_FRWD, LOW);
  digitalWrite(RIGHT_MTR_FRWD, LOW);

  digitalWrite(LEFT_MTR_BCKWD, HIGH);
  digitalWrite(RIGHT_MTR_BCKWD, HIGH);
}

void *robotCommandListenner(void *var)
{
  while (true)
  {
  }
}

void setup()
{
  lidarSerial.begin(230400, SERIAL_8N1, LIDAR_TX, -1);
  usbSerial.begin(115200);
  WiFi.mode(WIFI_STA);
  usbSerial.println(ssid);
  usbSerial.println(password);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    usbSerial.println("Connecting to WiFi..");
  }

  usbSerial.println("Connected to WiFi network");
  usbSerial.print("IP Address: ");
  usbSerial.println(WiFi.localIP());
  if (!tcpClient.connect(host, port))
  {
    usbSerial.println("Connection to TCP tcpClient failed");
    // delay(100);
    return;
  }

  pinMode(RIGHT_MTR_FRWD, OUTPUT);
  pinMode(RIGHT_MTR_BCKWD, OUTPUT);
  pinMode(PWM_RIGHT_MTR, OUTPUT);

  pinMode(LEFT_MTR_FRWD, OUTPUT);
  pinMode(LEFT_MTR_BCKWD, OUTPUT);
  pinMode(PWM_LEFT_MTR, OUTPUT);

  pinMode(MOTOR_STANDBY, OUTPUT);
  digitalWrite(MOTOR_STANDBY, 0);

  pinMode(LIDAR_PWM, INPUT_PULLDOWN);
}

int counter = 3;
void ProcessTheData();

uint16_t bit_shift(uint8_t msb, uint8_t lsb)
{
  return (msb << 8) | lsb;
}

uint16_t read_2_bytes(HardwareSerial *mySerial)
{
  uint8_t lsb = mySerial->read();
  uint8_t msb = mySerial->read();
  return bit_shift(msb, lsb);
}

u_int8_t data = 0;
u_int8_t old_data = 0;

LiDARFrameTypeDef formatted_data;

void loop()
{

  // Working here
  // while (false)
  while (lidarSerial.available() > 0)
  {
    // analogWrite(LIDAR_PWM,50); // 40% of 255 -> 40% of duty cycle -> ~10hz
    // byte value = lidarSerial.read();
    data = lidarSerial.read();
    if (data == 44 && old_data == 84)
    { // 84 = starting header (0x54) | 44 = data length (0x2C)
      // usbSerial.print("start ");

      String msg = "";
      formatted_data.header = old_data;
      formatted_data.ver_len = data;
      // radar speed
      formatted_data.speed = read_2_bytes(&lidarSerial);
      // start angle
      formatted_data.start_angle = read_2_bytes(&lidarSerial);
      msg += "sa:" + String(formatted_data.start_angle) + " ";
      // distance data
      msg += "nbv:" + String(ANGLE_PER_FRAME) + " ";
      for (int i = 0; i < ANGLE_PER_FRAME; i++)
      {
        formatted_data.point[i].distance = read_2_bytes(&lidarSerial);
        formatted_data.point[i].confidence = lidarSerial.read();

        msg += "v" + String(i) + ":" + String(formatted_data.point[i].distance) + " ";
      }

      // End angle
      formatted_data.end_angle = read_2_bytes(&lidarSerial);
      msg += "ea:" + String(formatted_data.end_angle) + " ";

      // Timestamp
      formatted_data.timestamp = read_2_bytes(&lidarSerial);

      // CRC check
      formatted_data.crc8 = lidarSerial.read();

      msg += "\n";
      tcpClient.write(msg.c_str(), msg.length());
    }
    // usbSerial.println(data);
    old_data = data;
  }

  // while (usbSerial.available() > 0)
  // {
  //   char keyboard_event = usbSerial.read();
  //   // usbSerial.print(keyboard_event);
  //   // usbSerial.print('\n');
  //   switch (keyboard_event)
  //   {
  //   case 'u':
  //     forward();
  //     break;
  //   case 't':
  //     toggle_standby();
  //     break;
  //   case 'd':
  //     backward();
  //     break;
  //   case 'r':
  //     right();
  //     break;
  //   case 'l':
  //     left();
  //     break;
  //   default:
  //     break;
  //   }
  // }
}
