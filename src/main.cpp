#include <Arduino.h>

// Definition of pinout
#define RIGHT_MTR_FRWD 36
#define RIGHT_MTR_BCKWD 37
#define PWM_RIGHT_MTR 1

#define LEFT_MTR_FRWD 45
#define LEFT_MTR_BCKWD 35
#define LEFT_RIGHT_MTR 47

#define MOTOR_STANDBY 46

#define LIDAR_PWM 21
#define LIDAR_TX 14

HardwareSerial lidarSerial(2);
HardwareSerial usbSerial(0);

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

void setup()
{
  usbSerial.begin(230400);
  lidarSerial.begin(230400, SERIAL_8N1, LIDAR_TX, -1);

  pinMode(RIGHT_MTR_FRWD, OUTPUT);
  pinMode(RIGHT_MTR_BCKWD, OUTPUT);
  pinMode(PWM_RIGHT_MTR, OUTPUT);

  pinMode(LEFT_MTR_FRWD, OUTPUT);
  pinMode(LEFT_MTR_BCKWD, OUTPUT);
  pinMode(LEFT_RIGHT_MTR, OUTPUT);

  pinMode(MOTOR_STANDBY, OUTPUT);
  digitalWrite(MOTOR_STANDBY, 0);

  pinMode(LIDAR_PWM, INPUT_PULLDOWN);
  // digitalWrite(LIDAR_PWM, 0); // Grounded should also be 10Hz
  // analogWrite(LIDAR_PWM,102); // 40% of 255 -> 40% of duty cycle -> ~10hz
}

int counter = 3;
void ProcessTheData();

uint16_t bit_shift(uint8_t msb, uint8_t lsb)
{
  return (msb << 8) | lsb;
}

uint16_t read_2_bytes(HardwareSerial* mySerial ){
  uint8_t lsb = mySerial->read();
  uint8_t msb = mySerial->read();
  return bit_shift(msb, lsb);
}

u_int8_t data = 0;
u_int8_t old_data = 0;

LiDARFrameTypeDef formatted_data;


void loop()
{
  // digitalWrite(RIGHT_MTR_FRWD, 1);
  // digitalWrite(RIGHT_MTR_BCKWD, 0);
  // digitalWrite(LEFT_MTR_FRWD, 1);
  // digitalWrite(LEFT_MTR_BCKWD, 0);
  // analogWrite(PWM_RIGHT_MTR, 255);
  // analogWrite(LEFT_RIGHT_MTR, 255);
  // delay(1000);
  // digitalWrite(RIGHT_MTR_FRWD, 0);
  // digitalWrite(RIGHT_MTR_BCKWD, 1);
  // digitalWrite(LEFT_MTR_FRWD, 0);
  // digitalWrite(LEFT_MTR_BCKWD, 1);
  // delay(1000);

  // Working here
  while (lidarSerial.available() > 0)
  {
    // analogWrite(LIDAR_PWM,50); // 40% of 255 -> 40% of duty cycle -> ~10hz
    // byte value = lidarSerial.read();
    // usbSerial.println(value);
    data = lidarSerial.read();
    if(data == 44 && old_data == 84){ // 84 = starting header (0x54) | 44 = data length (0x2C)
      // usbSerial.print("start ");

      formatted_data.header = old_data;
      formatted_data.ver_len = data;
      // radar speed 
      formatted_data.speed = read_2_bytes(&lidarSerial);
      // start angle
      formatted_data.start_angle = read_2_bytes(&lidarSerial);
      usbSerial.print("sa:");
      usbSerial.print(formatted_data.start_angle);
      usbSerial.print(" ");

      // distance data
      usbSerial.print("nbv:");
      usbSerial.print(ANGLE_PER_FRAME);
      usbSerial.print(" ");
      for(int i = 0; i<ANGLE_PER_FRAME; i++)
      {
        formatted_data.point[i].distance = read_2_bytes(&lidarSerial);
        formatted_data.point[i].confidence = lidarSerial.read();
        // usbSerial.print("v");
        usbSerial.print(i);
        usbSerial.print(":");
        usbSerial.print(formatted_data.point[i].distance);
        usbSerial.print(" ");
      }

      // End angle
      formatted_data.end_angle = read_2_bytes(&lidarSerial);
      usbSerial.print("ea:");
      usbSerial.print(formatted_data.end_angle);
      // usbSerial.print(" ");

      // Timestamp
      formatted_data.timestamp = read_2_bytes(&lidarSerial);

      // CRC check
      formatted_data.crc8 = lidarSerial.read();

      usbSerial.print("\n");
    }
    // usbSerial.println(data);
    old_data = data;
  }


}


