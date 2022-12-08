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
#define SERVO_TESTING 11
leg right_front((servo_t){9, 70, 442, M_PI_2, -M_PI_2}, (servo_t){10, 76, 436, M_PI_2, -M_PI_2}, (servo_t){11, 84, 436, M_PI, 0}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M),
    left_front((servo_t){6, 68, 432, M_PI_2, -M_PI_2}, (servo_t){7, 68, 428, -M_PI_2, M_PI_2}, (servo_t){8, 70, 420, 0, M_PI}, &pwm, UPPER_LEN_M, LOWER_LEN_M, FOOT_RADIUS_M),
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
  // while (1)
  // {
  //   while (!Serial.available())
  //     delay(10);
  //   pwm.setPWM(SERVO_TESTING, 0, Serial.parseInt());
  //   // for (float i = .03; i < .1; i += .001)
  //   // {
  //   //   left_back.move_to(0, 0, i);
  //   //   delay(30);
  //   // }
  // }
  // pwm.setPWM(1, 0, RAD_TO_SERVO_MID(0));
  // pwm.setPWM(2, 0, RAD_TO_SERVO(M_PI));

  // while (1)
  // {
  //   right_front.move_to(0, 0, .1);
  //   left_back.move_to(0, 0, .1);
  //   left_front.move_to(0, 0, .1);
  //   right_back.move_to(0, 0, .1);

  //   delay(1000);

  //   right_front.move_to(.04, 0, .1);
  //   left_back.move_to(.04, 0, .1);
  //   left_front.move_to(.04, 0, .1);
  //   right_back.move_to(.04, 0, .1);

  //   delay(1000);

  //   right_front.move_to(-.04, 0, .1);
  //   left_back.move_to(-.04, 0, .1);
  //   left_front.move_to(-.04, 0, .1);
  //   right_back.move_to(-.04, 0, .1);

  //   delay(1000);

  //   right_front.move_to(0, 0, .1);
  //   left_back.move_to(0, 0, .1);
  //   left_front.move_to(0, 0, .1);
  //   right_back.move_to(0, 0, .1);

  //   delay(1000);

  //   right_front.move_to(0, 0, .06);
  //   left_back.move_to(0, 0, .06);
  //   left_front.move_to(0, 0, .06);
  //   right_back.move_to(0, 0, .06);

  //   delay(1000);

  //   right_front.move_to(0, 0, .13);
  //   left_back.move_to(0, 0, .13);
  //   left_front.move_to(0, 0, .13);
  //   right_back.move_to(0, 0, .13);

  //   delay(1000);
  // }

  unsigned long start_time, t, temp;
  float x, z;

  const unsigned cycle_time = 300;
  unsigned phase = 0;
  const float path_radius = .02f;

  leg legs[] = {right_front, left_back, left_front, right_back};

  start_time = millis();
  while (1)
  {
    for (leg cur_leg : legs)
    {
      temp = (t + phase) % cycle_time;
      if (temp <= cycle_time / 2)
      {
        x = -path_radius * cos((float)temp / cycle_time * M_PI);
        z = .08f - path_radius * sin((float)temp / cycle_time * M_PI);
      }
      else
      {
        temp = temp % (cycle_time / 2);
        x = path_radius - 2.0f * path_radius / cycle_time * temp;
        z = .08f;
      }
      cur_leg.move_to(x, 0, z);
      phase += cycle_time / 4;
    }
    phase = 0;
    t = millis() - start_time;
    delay(5);
  }

  vTaskDelete(pwmTask);

  // int t = 0;
  // float x = t - sin(t), z = 1 - cos(t);
  for (float i = .12; i >= .03; i -= 0.001)
  {
    // float x = 0.0, z = i;
    // float d2 = x * x + z * z; // Distance b/w circles squared (for efficiency)
    // float c1 = UPPER_LEN_M * UPPER_LEN_M - LOWER_LEN_M * LOWER_LEN_M + d2;
    // float c2 = c1 / (2.0 * d2);
    // if (4 * UPPER_LEN_M * UPPER_LEN_M >= c1 * c1 / d2) // Is there a solution? To optimize later
    // {
    //   float c3 = sqrtf(4 * UPPER_LEN_M * UPPER_LEN_M - c1 * c1 / d2) / (2 * sqrtf(d2));
    //   float servo1 = -atanf((c2 * z + c3 * x) / (c2 * x - c3 * z));
    //   float servo2 = M_PI - acos((UPPER_LEN_M * UPPER_LEN_M + LOWER_LEN_M * LOWER_LEN_M - d2) / (2.0 * UPPER_LEN_M * LOWER_LEN_M));
    //   Serial.print(i);
    //   Serial.print(" ");
    //   Serial.print(c2 * z + c3 * x);
    //   Serial.print(" ");
    //   Serial.print(c2 * x - c3 * z);
    //   Serial.print(" ");
    //   Serial.print(servo1);
    //   Serial.print(" ");
    //   Serial.println(servo2);
    //   pwm.setPWM(0, 0, RAD_TO_SERVO_MID(-servo1));
    //   pwm.setPWM(1, 0, RAD_TO_SERVO(servo2));
    // }
    // else
    // {
    //   pwm.setPWM(0, 0, RAD_TO_SERVO_MID(0));
    //   pwm.setPWM(1, 0, RAD_TO_SERVO(0));
    // }
    // // pwm.setPWM(0, 0, RAD_TO_SERVO(M_PI_4));
    // // pwm.setPWM(1, 0, RAD_TO_SERVO(M_PI_4));
    // delay(30);
  }
  vTaskDelete(pwmTask);
}
