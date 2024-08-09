#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// アナログ入力ピンの定義
#define ANALOG_PIN_X A0 // x軸用インスツルメンテーションアンプの出力
#define ANALOG_PIN_Y A1 // y軸用インスツルメンテーションアンプの出力
 
// サーボモータのピンの定義
#define SERVO_PIN_X1 9
#define SERVO_PIN_X2 10
#define SERVO_PIN_Y1 11
#define SERVO_PIN_Y2 12

#define BIAS 2047
#define KP 0.1 // 比例ゲイン
#define KI 0.01 // 積分ゲイン
#define KD 0.05 // 微分ゲイン
#define INTEGRAL_LIMIT 1000.0 // 積分項の制限
#define N 3.0 // 比例航法定数

// サーボオブジェクトの作成
Servo servoX1, servoX2, servoY1, servoY2;

// IMUオブジェクトの作成
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// PID制御用の変数
float integralX = 0, integralY = 0;
float lastErrorX = 0, lastErrorY = 0;
unsigned long lastTime = 0;

// 関数プロトタイプ
void adjustServo(Servo &servo1, Servo &servo2, float controlSignal);
float calculateTargetAngle(uint16_t adcValue, float bias);
float readIMUAngularRate();
float calculateProportionalNavigation(float currentAngle, float targetAngle, float deltaTime);

void setup() {
  // シリアル通信の初期化
  Serial.begin(115200);

  // アナログ入力ピンの初期化
  pinMode(ANALOG_PIN_X, INPUT);
  pinMode(ANALOG_PIN_Y, INPUT);

  // サーボモータの初期化
  servoX1.attach(SERVO_PIN_X1);
  servoX2.attach(SERVO_PIN_X2);
  servoY1.attach(SERVO_PIN_Y1);
  servoY2.attach(SERVO_PIN_Y2);

  // IMUの初期化
  if(!bno.begin()) {
    Serial.println("BNO055が検出されませんでした。配線やI2Cアドレスを確認してください。");
    while (1);
  }

  bno.setExtCrystalUse(true);

  // 時間の初期化
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // 秒に変換
  lastTime = currentTime;

  // アナログ入力からのデータを読み取る
  uint16_t adcValueX = analogRead(ANALOG_PIN_X);
  uint16_t adcValueY = analogRead(ANALOG_PIN_Y);

  // 目標角度を計算
  float targetAngleX = calculateTargetAngle(adcValueX, BIAS);
  float targetAngleY = calculateTargetAngle(adcValueY, BIAS);

  // IMUデータ（角速度）の読み取り
  float imuAngularRate = readIMUAngularRate();

  // PID制御信号の計算
  float errorX = targetAngleX;
  integralX += errorX * deltaTime;
  integralX = constrain(integralX, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivativeX = (errorX - lastErrorX) / deltaTime;
  float controlSignalX = KP * errorX + KI * integralX + KD * derivativeX;
  lastErrorX = errorX;

  float errorY = targetAngleY;
  integralY += errorY * deltaTime;
  integralY = constrain(integralY, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivativeY = (errorY - lastErrorY) / deltaTime;
  float controlSignalY = KP * errorY + KI * integralY + KD * derivativeY;
  lastErrorY = errorY;

  // サーボモータの調整
  adjustServo(servoX1, servoX2, controlSignalX);
  adjustServo(servoY1, servoY2, controlSignalY);

  delay(10); // ループの間に小さな遅延を追加
}

void adjustServo(Servo &servo1, Servo &servo2, float controlSignal) {
  // サーボの制御信号を適用し、同じ軸の2つのサーボを同じ角度で動かす
  float adjustedControlSignal = controlSignal;
  adjustedControlSignal = constrain(adjustedControlSignal, -10, 10); // サーボ角度を±10度に制限
  servo1.write(90 + adjustedControlSignal); // サーボを90度に中心にして動かす
  servo2.write(90 + adjustedControlSignal); // サーボを90度に中心にして動かす
}

float calculateTargetAngle(uint16_t adcValue, float bias) {
  // バイアスを用いて目標角度を計算
  return (float)(adcValue - bias);
}

float readIMUAngularRate() {
  // IMUから角速度を読み取る（ここではx軸の角速度を使用）
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
  return event.gyro.x;
}

float calculateProportionalNavigation(float currentAngle, float targetAngle, float deltaTime) {
  // 比例航法の計算
  return N * (targetAngle - currentAngle) / deltaTime;
}
