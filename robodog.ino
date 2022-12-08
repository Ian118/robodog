#include <esp_now.h>

#include <Adafruit_PWMServoDriver.h>

#include <Adafruit_BNO08x.h>

#include "leg.h"

#define SSID "robodog"
#define PASS NULL
#define SERVOMIN 60  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 420 // This is the 'maximum' pulse length count (out of 4096)
#define UPPER_LEN_M 0.0685
#define LOWER_LEN_M 0.06382494
#define FOOT_RADIUS_M 0.01

// #define RAD_TO_SERVO_MID(rad) (MAX(SERVOMIN, MIN(SERVOMAX, (SERVOMAX - SERVOMIN) / M_PI * (rad + M_PI_2) + SERVOMIN)))
// #define RAD_TO_SERVO(rad) (MAX(SERVOMIN, MIN(SERVOMAX, (SERVOMAX - SERVOMIN) / M_PI * rad + SERVOMIN)))

Adafruit_BNO08x bno08x;
sh2_SensorValue_t bno08x_value;

TaskHandle_t wifiTask, gyroTask, pwmTask;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// WiFiServer server(80);

// servos 7 and 10 will need extra calibration
// #define SERVO_TESTING 4
leg right_front((servo_t){9, 70, 442, M_PI_2, -M_PI_2}, (servo_t){10, 76, 436, M_PI_2, -M_PI_2}, (servo_t){11, 84, 436, M_PI, 0}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M),
    left_front((servo_t){6, 68, 432, M_PI_2, -M_PI_2}, (servo_t){7, 68, 428, -M_PI_2, M_PI_2}, (servo_t){8, 80, 420, 0, M_PI}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M),
    right_back((servo_t){3, 72, 440, -M_PI_2, M_PI_2}, (servo_t){4, 60, 416, M_PI_2, -M_PI_2}, (servo_t){5, 48, 404, M_PI, 0}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M),
    left_back((servo_t){0, 52, 412, -M_PI_2, M_PI_2}, (servo_t){1, 58, 422, -M_PI_2, M_PI_2}, (servo_t){2, 60, 420, 0, M_PI}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M);

unsigned char broadcastAddress[] = {};

void setup()
{
  Serial.begin(115200);

  // Wait for Serial port to open
  while (!Serial)
    delay(10);
  delay(500);

  // if (esp_now_init() != ESP_OK)
  // {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  // xTaskCreatePinnedToCore(runServer, "WiFi Task", 10000, NULL, 1, &wifiTask, 0);
  // xTaskCreate(runGyro, "Gyro Task", 10000, NULL, 1, &gyroTask);
  xTaskCreate(runPWM, "PWM Task", 10000, NULL, 2, &pwmTask);

  // Serial.println("Reading events");
  delay(100);
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

// void printWifiStatus()
// {
//   // print the SSID of the network you're attached to:
//   Serial.print("SSID: ");
//   Serial.println(WiFi.softAPSSID());

//   // print your board's IP address:
//   IPAddress ip = WiFi.softAPIP();
//   Serial.print("IP Address: ");
//   Serial.println(ip);

//   // print the received signal strength:
//   long rssi = WiFi.RSSI();
//   Serial.print("signal strength (RSSI):");
//   Serial.print(rssi);
//   Serial.println(" dBm");
// }

void runServer(void *args)
{
  // printWifiStatus();
  // server.begin();

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  vTaskDelete(wifiTask);
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
  // pwm.setPWM(1, 0, RAD_TO_SERVO_MID(0));
  // pwm.setPWM(2, 0, RAD_TO_SERVO(M_PI));

  unsigned long start_time, t, temp;
  float x, z;

  const unsigned cycle_time = 400U;
  unsigned phase = 0;
  const float path_radius = .02f;

  leg legs[] = {right_front, left_back, left_front, right_back};

  start_time = millis();
  while (1)
  {
    for (leg cur_leg : legs)
    {
      temp = t + phase;
      if (temp % cycle_time <= cycle_time / 2)
      {
        temp %= cycle_time / 2;
        x = -path_radius * cos(2.0f * temp / cycle_time * M_PI);
        z = .12f - path_radius * 0.5f * sin(2.0f * temp / cycle_time * M_PI);
      }
      else
      {
        temp %= cycle_time / 2;
        x = path_radius - 4.0f * path_radius / cycle_time * temp;
        z = .12f;
      }
      if (phase % (cycle_time / 2))
        x -= .02f;
      else
        x += .02f;
      cur_leg.move_to(x, 0, z);
      phase += cycle_time / 4;
    }
    phase = 0;
    t = millis() - start_time;
    delay(2);
  }
}
