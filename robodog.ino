#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_BNO08x.h>

#include "leg.h"

#define SERVOMIN 60  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 420 // This is the 'maximum' pulse length count (out of 4096)
#define UPPER_LEN_M 0.0685
#define LOWER_LEN_M 0.06382494
#define FOOT_RADIUS_M 0.01
#define SHOULDER_OFFSET_M 0.0119685

// E8:9F:6D:25:3B:2C

// #define RAD_TO_SERVO_MID(rad) (MAX(SERVOMIN, MIN(SERVOMAX, (SERVOMAX - SERVOMIN) / M_PI * (rad + M_PI_2) + SERVOMIN)))
// #define RAD_TO_SERVO(rad) (MAX(SERVOMIN, MIN(SERVOMAX, (SERVOMAX - SERVOMIN) / M_PI * rad + SERVOMIN)))

Adafruit_BNO08x bno08x;
sh2_SensorValue_t bno08x_value;

TaskHandle_t gyroTask, pwmTask;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct js_state
{
  unsigned vertical;
  unsigned horizontal;
} stick_state;

float fwd;

// servos 7 and 10 will need extra calibration
// #define SERVO_TESTING 10
leg right_front((servo_t){9, 70, 442, M_PI_2, -M_PI_2}, (servo_t){10, 76, 436, M_PI_2, -M_PI_2}, (servo_t){11, 84, 436, M_PI, 0}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M, SHOULDER_OFFSET_M),
    left_front((servo_t){6, 68, 432, M_PI_2, -M_PI_2}, (servo_t){7, 120, 480, -M_PI_2, M_PI_2}, (servo_t){8, 80, 420, 0, M_PI}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M, SHOULDER_OFFSET_M),
    right_back((servo_t){3, 72, 440, -M_PI_2, M_PI_2}, (servo_t){4, 60, 416, M_PI_2, -M_PI_2}, (servo_t){5, 48, 404, M_PI, 0}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M, SHOULDER_OFFSET_M),
    left_back((servo_t){0, 52, 412, -M_PI_2, M_PI_2}, (servo_t){1, 58, 422, -M_PI_2, M_PI_2}, (servo_t){2, 60, 420, 0, M_PI}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M, SHOULDER_OFFSET_M);

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

  Serial.println(WiFi.macAddress());

  // xTaskCreatePinnedToCore(runServer, "WiFi Task", 10000, NULL, 1, &wifiTask, 0);
  // xTaskCreate(runGyro, "Gyro Task", 10000, NULL, 1, &gyroTask);
  xTaskCreate(runPWM, "PWM Task", 10000, NULL, 2, &pwmTask);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void)
{
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
    Serial.println("Could not enable rotation vector");
}

void loop()
{
  delay(100);
}

void data_receive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&stick_state, incomingData, sizeof(stick_state));
  Serial.print(stick_state.horizontal);
  Serial.print(", ");
  Serial.println(stick_state.vertical);
  fwd = abs((signed)stick_state.vertical - 512) > 10 ? -((signed)stick_state.vertical - 512) / 512.0f : 0.0f;
}

void runGyro(void *args)
{
  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to find BNO08x chip");
    while (1)
      delay(10);
  }
  Serial.println("BNO08x Found!");

  setReports();

  while (1)
  {
    delay(10);

    if (bno08x.wasReset())
    {
      Serial.println("sensor was reset ");
      setReports();
    }

    if (!bno08x.getSensorEvent(&bno08x_value))
      continue;

    switch (bno08x_value.sensorId)
    {
    case SH2_ROTATION_VECTOR:
      sh2_RotationVectorWAcc r = bno08x_value.un.rotationVector;
      // Serial.print("Quaternion: ");
      // Serial.print(r.real);
      // Serial.print(", ");
      // Serial.print(r.i);
      // Serial.print(", ");
      // Serial.print(r.j);
      // Serial.print(", ");
      // Serial.println(r.k);

      // float roll = atan2(2 * (r.real * r.i + r.j * r.k), 1 - 2 * (r.i * r.i + r.j * r.j));
      // Serial.print(roll / M_PI);
      // Serial.println("PI");
      // if (roll > 0)
      //   pwm.setPWM(0, 0, RAD_TO_SERVO(roll));
      // break;
    }
  }
}

void runPWM(void *args)
{
#ifdef SERVO_TESTING
  while (1)
  {
    while (!Serial.available())
      delay(10);
    pwm.setPWM(SERVO_TESTING, 0, Serial.parseInt());
  }
#endif
  // left_back.move_to(0, SHOULDER_OFFSET_M, .1);
  // vTaskDelete(pwmTask);

  unsigned long start_time, current_time;
  float x, y, z, t;

  const unsigned cycle_time = 600U;
  unsigned i = 0;
  const float nominal_path_radius = .035f;
  float path_radius = nominal_path_radius;

  leg legs[] = {right_front, left_back, left_front, right_back};

  start_time = millis();
  while (1)
  {
    i = 0;
    path_radius = nominal_path_radius * fwd;
    Serial.println(path_radius);
    for (leg cur_leg : legs)
    {
      t = (float)((current_time + i * cycle_time / 4U) % cycle_time) / cycle_time;
      x = -path_radius * cosf(2 * M_PI * t);
      z = .095f - abs(path_radius) * 0.5f * (sinf(2 * M_PI * t) - 0.25f * cosf(4 * M_PI * t) + 0.75f);
      y = SHOULDER_OFFSET_M;
      if (i == 0 || i == 3)
        y -= 0.01f;
      else
        y += 0.01f;
      if (i % 2)
      {
        z += .01f * fwd;
        x -= .02f;
      }
      else
      {
        z -= .01f * fwd;
        x += .02f;
      }
      cur_leg.move_to(x, y, z);
      i++;
    }
    current_time = millis() - start_time;
    delay(2);
  }
}
