#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>

#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_BNO08x.h>

#include "leg.hpp"
#include "quaternion.hpp"
#include "robot.hpp"

#define UPPER_LEN_M 0.05f
#define LOWER_LEN_M 0.05f
#define FOOT_RADIUS_M 0.01f
#define SHOULDER_OFFSET_M 0.0119685f

Adafruit_BNO08x bno08x;
sh2_SensorValue_t bno08x_value;

TaskHandle_t gyroTask, bluetoothTask;

BluetoothSerial bt;

struct
{
  unsigned vertical;
  unsigned horizontal;
  bool right;
  bool down;
  bool left;
  bool up;
  bool sel;
} stick_state;

// #define SERVO_TESTING

leg right_front = {(servo_t){9, 248 - 180, 248 + 180, M_PI_2, -M_PI_2}, (servo_t){10, 456 - 360, 456, -M_PI_2, M_PI_2}, (servo_t){11, 458 - 360, 458, M_PI, 0}},
    left_front = {(servo_t){6, 272 - 180, 272 + 180, M_PI_2, -M_PI_2}, (servo_t){7, 64, 64 + 360, M_PI_2, -M_PI_2}, (servo_t){8, 96, 96 + 360, 0, M_PI}},
    right_back = {(servo_t){3, 220 - 180, 220 + 180, -M_PI_2, M_PI_2}, (servo_t){4, 430 - 360, 430, -M_PI_2, M_PI_2}, (servo_t){5, 450 - 360, 450, M_PI, 0}},
    left_back = {(servo_t){0, 294 - 180, 294 + 180, -M_PI_2, M_PI_2}, (servo_t){1, 76, 76 + 360, M_PI_2, -M_PI_2}, (servo_t){2, 92, 92 + 360, 0, M_PI}};

// 0- mid 294
// 1- down 76
// 2- down 92
// 3- mid 220
// 4- down 430
// 5- down 450
// 6- mid 272
// 7- down 68
// 8- down 96
// 9- mid 248
// 10- down 456
// 11- down 458

constexpr quaternion offset = {0.0f, 0.036672f, 0.0434685f, 0.0f};

Robot robot(right_front, left_front, right_back, left_back, offset, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M, SHOULDER_OFFSET_M);

void setup()
{
  Serial.begin(115200);
  bt.begin("Robodog");
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
  }
  else
    esp_now_register_recv_cb(data_receive);

  while (!bno08x.begin_I2C())
    delay(1000);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  xTaskCreate(runGyro, "Gyro Task", 10000, NULL, 1, &gyroTask);
  xTaskCreate(runBluetooth, "Bluetooth Task", 10000, NULL, 2, &gyroTask);
}

#ifdef SERVO_TESTING
int servo_testing = NULL;
#endif

void loop()
{
#ifdef SERVO_TESTING
  while (!Serial.available())
    delay(10);
  String str = Serial.readString();
  if (str.equalsIgnoreCase("testing"))
  {
    while (!Serial.available())
      delay(10);
    servo_testing = Serial.parseInt();
    Serial.printf("Changed to servo %d\n", servo_testing);
  }
  else
  {
    int pulse = str.toInt();
    pwm.setPWM(servo_testing, 0, pulse);
    Serial.printf("Servo %d set to %d\n", servo_testing, pulse);
  }
#else
  robot.process();
#endif
}

void data_receive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&stick_state, incomingData, sizeof(stick_state));
  float fwd = -((signed)stick_state.vertical - 512) / 512.0f;
  float right = ((signed)stick_state.horizontal - 512) / 512.0f;
  robot.v_x = abs(fwd) > .02 ? 0.05f * fwd : 0.0f;     // Max at 0.25 m/s
  robot.v_y = abs(right) > .02 ? 0.05f * right : 0.0f; // Max at 0.25 m/s
}

void runGyro(void *args)
{
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
    Serial.println("Could not enable rotation vector");

  while (1)
  {
    if (bno08x.wasReset())
    {
      Serial.println("sensor was reset ");
      if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
        Serial.println("Could not enable rotation vector");
    }

    if (!bno08x.getSensorEvent(&bno08x_value))
      continue;

    switch (bno08x_value.sensorId)
    {
    case SH2_ROTATION_VECTOR:
      sh2_RotationVectorWAcc val = bno08x_value.un.rotationVector;
      euler eu = (quaternion){val.real, val.i, val.j, val.k};
      eu = {0.0f, -eu.pitch, eu.roll};
      robot.orientation = eu;
      euler eu2 = robot.correction;
      // robot.correction = (euler){0.0f, MAX(MIN(eu2.pitch + eu.pitch / M_PI, M_PI_4), -M_PI_4), MAX(MIN(eu2.roll + eu.roll / M_PI, M_PI_4), -M_PI_4)};
      robot.relax = fabsf(eu.pitch) > M_PI_4 || fabsf(eu.roll) > M_PI_4;
      delay(20);
    }
  }
}

void runBluetooth(void *args)
{
  struct data
  {
    float x_offset, y_offset, shoulder, upper, lower;
  };

  while (1)
  {
    for (leg *cur_leg : robot.legs)
    {
      uint8_t foo[sizeof(data)];
      data d = (data){cur_leg->offset.i, cur_leg->offset.j, cur_leg->shoulder_theta, cur_leg->upper_theta, cur_leg->lower_theta};
      Serial.printf("%f,%f,%f,%f,%f\n", cur_leg->offset.i, cur_leg->offset.j, cur_leg->shoulder_theta, cur_leg->upper_theta, cur_leg->lower_theta);
      memcpy(foo, &d, sizeof(d));
      bt.write(foo, sizeof(foo));
    }
    delay(500);
  }
}