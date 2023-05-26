#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO08x.h>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

#include "leg.hpp"
#include "quaternion.hpp"
#include "robot.hpp"

#define UPPER_LEN_M 0.05f
#define LOWER_LEN_M 0.05f
#define FOOT_RADIUS_M 0.01f
#define SHOULDER_OFFSET_M 0.0119685f

using namespace Eigen;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t bno08x_value;

TaskHandle_t gyroTask, sendDataTask;

struct
{
  float v_x, v_y, yaw, pitch, roll;
  bool poweroff;
} input;

// #define SERVO_TESTING

leg right_front = {(servo_t){9, 248 - 180, 248 + 180, M_PI_2, -M_PI_2}, (servo_t){10, 456 - 360, 456, -M_PI_2, M_PI_2}, (servo_t){11, 458 - 360, 458, M_PI, 0}},
    left_front = {(servo_t){6, 272 - 180, 272 + 180, M_PI_2, -M_PI_2}, (servo_t){7, 64, 64 + 360, M_PI_2, -M_PI_2}, (servo_t){8, 96, 96 + 360, 0, M_PI}},
    right_back = {(servo_t){3, 220 - 180, 220 + 180, -M_PI_2, M_PI_2}, (servo_t){4, 400 - 360, 400, -M_PI_2, M_PI_2}, (servo_t){5, 450 - 360, 450, M_PI, 0}},
    left_back = {(servo_t){0, 294 - 180, 294 + 180, -M_PI_2, M_PI_2}, (servo_t){1, 80, 80 + 360, M_PI_2, -M_PI_2}, (servo_t){2, 92, 92 + 360, 0, M_PI}};

// 0- mid 294
// 1- down 80
// 2- down 92
// 3- mid 220
// 4- down 400
// 5- down 450
// 6- mid 272
// 7- down 68
// 8- down 96
// 9- mid 248
// 10- down 456
// 11- down 458

// constexpr quaternion offset = {0.0f, 0.036672f, 0.0434685f, 0.0f};

constexpr Eigen::Quaternionf offset{0.0f, 0.036672f, 0.0434685f, 0.0f};

Robot robot(right_front, left_front, right_back, left_back, offset, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M, SHOULDER_OFFSET_M);

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
  }
  else
    esp_now_register_recv_cb(data_receive);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

#ifdef SERVO_TESTING
  for (leg *cur_leg : robot.legs)
  {
    cur_leg->shoulder = 0.0f;
    cur_leg->upper_leg = (float)M_PI_2;
    cur_leg->lower_leg = 0.0f;
  }
#else
  // xTaskCreate(runGyro, "Gyro Task", 10000, NULL, 1, &gyroTask);
  // xTaskCreate(sendData, "Data Sending Task", 10000, NULL, 1, &sendDataTask);
#endif
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
  memcpy(&input, incomingData, sizeof(input));
  if (input.poweroff)
    esp_deep_sleep_start();
  robot.v_x = input.v_x;
  robot.v_y = input.v_y;
  robot.orientation = (euler){input.yaw, input.pitch, input.roll};
  if (robot.v_x != 0.0f || robot.v_y != 0.0f)
    vTaskSuspend(gyroTask);
  else
  {
    if (eTaskGetState(gyroTask) == eTaskState::eSuspended)
    {
      robot.correction = {1.0f};
      vTaskResume(gyroTask);
    }
  }
}

void runGyro(void *args)
{
  while (!bno08x.begin_I2C())
    delay(1000);

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
    {
      sh2_RotationVectorWAcc val = bno08x_value.un.rotationVector;
      quaternion q = {val.real, -val.i, val.j, -val.k}; // flip y axis
      float z_angle_adjustment = -0.5f * atan2f(2.0f * (q.w * q.k + q.i * q.j), (1.0f - 2.0f * (q.j * q.j + q.k * q.k)));
      q = (quaternion){cosf(z_angle_adjustment), 0.0f, 0.0f, sinf(z_angle_adjustment)} * q; // remove z axis rotataion
      // robot.correction = !q;
      // printEuler(q)
      // printEuler(slerp((euler){0, M_PI_4, 0}, (quaternion){1.0f, 0.0f, 0.0f, 0.0f}, 0.f)); //* robot.correction;
      robot.correction = q * !slerp(q, {1.0f}, 0.05f) * robot.correction;
      printEuler(robot.correction);
    }
    break;

      // euler eu = (quaternion){val.real, val.i, val.j, val.k};
      // eu = {0.0f, -eu.pitch, eu.roll};
      // euler eu2 = robot.correction;
      // robot.correction = (euler){0.0f, MAX(MIN(eu2.pitch + eu.pitch / M_PI, M_PI_4), -M_PI_4), MAX(MIN(eu2.roll + eu.roll / M_PI, M_PI_4), -M_PI_4)};
      // robot.relax = fabsf(eu.pitch) > M_PI_4 || fabsf(eu.roll) > M_PI_4;
    default:
      delay(20);
    }
  }
}

void sendData(void *args)
{
  struct data
  {
    float x_offset, y_offset, shoulder, upper, lower;
    unsigned char delimeter;
  };
  while (1)
  {
    for (leg *cur_leg : robot.legs)
    {
      // uint8_t foo[sizeof(data)];
      // data d = (data){cur_leg->offset.i, cur_leg->offset.j, cur_leg->shoulder_theta, cur_leg->upper_theta, cur_leg->lower_theta};
      // memcpy(foo, &d, sizeof(d));
      // Serial.write(foo, sizeof(foo));

      Serial.printf("%f,%f,%f,%f,%f\n", cur_leg->offset.i, cur_leg->offset.j, cur_leg->shoulder_theta, cur_leg->upper_theta, cur_leg->lower_theta);
    }
    delay(100);
  }
}