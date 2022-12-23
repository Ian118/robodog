#include <esp_now.h>
#include <WiFi.h>

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

TaskHandle_t gyroTask, robotTask;

struct js_state
{
  unsigned vertical;
  unsigned horizontal;
} stick_state;

float fwd;

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
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
  }
  else
    esp_now_register_recv_cb(data_receive);

  delay(200);

  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to find BNO08x chip");
    while (1)
      delay(10);
  }
  Serial.println("BNO08x Found!");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  xTaskCreate(runGyro, "Gyro Task", 10000, NULL, 1, &gyroTask);
  // xTaskCreate(robot.process, "Robot Task", 10000, NULL, 1, &robotTask);

  // robot.correction = (euler){0, -M_PI / 8.0f, 0};

  // for (leg *cur_leg : robot.legs)
  //   *cur_leg = cur_leg->offset;
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
  Serial.print(stick_state.horizontal);
  Serial.print(", ");
  Serial.println(stick_state.vertical);
  robot.v_x = abs((signed)stick_state.vertical - 512) > 10 ? -((signed)stick_state.vertical - 512) / 512.0f : 0.0f;
}

void runGyro(void *args)
{
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
    Serial.println("Could not enable rotation vector");

  // while (1)
  // {

  //   //   robot.correction = (euler){0.0f, -M_PI / 16.0f, 0.0f};
  //   //   delay(2000);
  //   //   robot.correction = (euler){0.0f, M_PI / 16.0f, 0.0f};
  //   //   delay(2000);

  //   // robot.correction = (euler){0.0f, 0.0f, 0.0f};
  //   float target = M_PI / 6.0f;
  //   int time = 500;
  //   float increment = .01;
  //   unsigned delay_time = time * increment;

  //   for (float i = -target; i <= target; i += target * increment)
  //   {
  //     robot.correction = (euler){0.0f, i, 0.0f};
  //     delay(delay_time);
  //   }
  //   for (float i = target; i >= -target; i -= target * increment)
  //   {
  //     robot.correction = (euler){0.0f, i, 0.0f};
  //     delay(delay_time);
  //   }
  // }

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
      robot.correction = (euler){0.0f, MAX(MIN(eu2.pitch + eu.pitch / M_PI, M_PI_2), -M_PI_2), MAX(MIN(eu2.roll + eu.roll / M_PI, M_PI_2), -M_PI_2)};
      robot.relax = fabsf(eu.pitch) > M_PI_2 || fabsf(eu.roll) > M_PI_2;
      Serial.printf("%f, %f\n", eu.pitch, eu.roll);
      delay(20);

      // printEuler(eu);

      // robot.correction = (quaternion)eu * robot.correction;

      // rotation = eulerToQuaternion(eu);

      // quaternion q_prime = {q.w, -q.i, -q.j, -q.k}; // conjugate
      // float theta = atan2f(sqrtf(q.i * q.i + q.j * q.j + q.k * q.k), q.w);
      // quaternion q_new = (quaternion){cos(0.5f * theta), 0, sin(0.5f * theta), 0};
      // quaternion q_new_prime = {q_new.w, -q_new.i, -q_new.j, -q_new.k};
      // quaternion vx = {0, 1, 0, 0};                   // 0, x, y, z
      // quaternion vy = {0, 0, 1, 0};                   // 0, x, y, z
      // quaternion vz = {0, 0, 0, 1};                   // 0, x, y, z
      // quaternion vx_prime = q_new * vx * q_new_prime; // new position
      // quaternion vy_prime = q_new * vy * q_new_prime; // new position
      // quaternion vz_prime = q_new * vz * q_new_prime; // new position

      // quaternion fl = {0.0f, 0.036672f, 0.0434685f, 0.08f};
      // quaternion fr = {0.0f, 0.036672f, -0.0434685f, 0.08f};
      // quaternion bl = {0.0f, -0.036672f, 0.0434685f, 0.08f};
      // quaternion br = {0.0f, -0.036672f, -0.0434685f, 0.08f};

      // printEuler(q);

      // printVector(q * fl * q_prime);
      // Serial.print(" ");
      // printVector(q * fr * q_prime);
      // Serial.print(" ");
      // printVector(q * bl * q_prime);
      // Serial.print(" ");
      // printVector(q * br * q_prime);
      // Serial.println();

      // float roll = 2.0f * atan2f(val.real * val.i + val.j * val.k, 1.0f - 2.0f * (val.i * val.i + val.j * val.j));
      // float sinp = 2.0f * (val.real * val.j - val.k * val.i);
      // float pitch = abs(sinp) >= 1 ? copysignf(M_PI_2, sinp) : asinf(sinp);
      // float yaw = atan2f(val.real * val.k + val.i * val.j, 1.0f - 2.0f * (val.j * val.j + val.k * val.k));

      // float roll = atan2(2 * (r.real * r.i + r.j * r.k), 1 - 2 * (r.i * r.i + r.j * r.j));
      // Serial.print(roll / M_PI);
      // Serial.println("PI");
      // if (roll > 0)
      //   pwm.setPWM(0, 0, RAD_TO_SERVO(roll));
      // break;
    }
  }
}

// void runPWM(void *args)
// {
//   // left_back.move_to(0, SHOULDER_OFFSET_M, .1);
//   // vTaskDelete(pwmTask);

//   unsigned long start_time, current_time;
//   float x, y, z, t;

//   const unsigned cycle_time = 600U;
//   unsigned i = 0;
//   const float nominal_path_radius = .035f;
//   float path_radius = nominal_path_radius;
//   quaternion rotation_conjugate, current;

//   leg legs[] = {right_front, left_back, left_front, right_back};

//   start_time = millis();
//   while (1)
//   {
//     i = 0;
//     path_radius = nominal_path_radius * fwd;
//     for (leg cur_leg : legs)
//     {
//       t = (float)((current_time + i * cycle_time / 4U) % cycle_time) / cycle_time;
//       x = path_radius * cosf(2 * M_PI * t);
//       z = -abs(path_radius) * 0.5f * (sinf(2 * M_PI * t) - 0.25f * cosf(4 * M_PI * t) + 0.75f);
//       y = SHOULDER_OFFSET_M;

//       current = {0, (i % 2 == 1 ? -1 : 1) * 0.036672f, (i == 1 || i == 2 ? -1 : 1) * 0.0434685f, 0.095f};
//       current = rotation * current * (quaternion){rotation.w, -rotation.i, -rotation.j, -rotation.k};
//       current = {0, current.i - (i % 2 == 1 ? -1 : 1) * 0.036672f, current.j - (i == 1 || i == 2 ? -1 : 1) * 0.0434685f, current.k};
//       x += current.i;
//       y += current.j;
//       z += current.k;
//       // Serial.print(x);
//       // Serial.print(" ");
//       // Serial.print(y);
//       // Serial.print(" ");
//       // Serial.print(z);
//       // cur_leg.move_to(current.i - (i % 2 == 1 ? -1 : 1) * 0.036672f, current.j - (i == 1 || i == 2 ? -1 : 1) * 0.0434685f, current.k);
//       // printVector({0, current.i - (i % 2 == 1 ? -1 : 1) * 0.036672f, current.j - (i == 1 || i == 2 ? -1 : 1) * 0.0434685f, current.k});
//       // Serial.print(";");

//       // cur_leg.move_to(x, y, z);

//       // cur_leg.move_to(x, y, z);
//       i++;
//     }
//     // Serial.print("Quaternion: ");
//     // Serial.print(rotation.w);
//     // Serial.print(", ");
//     // Serial.print(rotation.i);
//     // Serial.print(", ");
//     // Serial.print(rotation.j);
//     // Serial.print(", ");
//     // Serial.println(rotation.k);

//     // Serial.println();
//     current_time = millis() - start_time;
//     delay(2);
//   }
// }
