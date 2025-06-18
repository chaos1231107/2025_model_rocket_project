// MPU6050 모듈 사용

#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;
#define FILTER_SIZE 10
#define ALPHA 0.2

// X, Y, Z 방향 사출 조건 판단 변수 
bool flag_X = false;
bool flag_Y = false;
bool flag_Z = false;

float angleX_buffer[FILTER_SIZE] = {0};
float angleY_buffer[FILTER_SIZE] = {0};
float angleZ_buffer[FILTER_SIZE] = {0};

float angleX_ema_buffer[FILTER_SIZE] = {0};
float angleY_ema_buffer[FILTER_SIZE] = {0};
float angleZ_ema_buffer[FILTER_SIZE] = {0};

int ignitor_relay_pin = 6;
bool ejectionStarted = false;
unsigned long ejectionStartTime = 0;

int filter_index = 0;
// 이동평균 필터 함수 (x, y, z방향 적용 모듈화)
float MovingAverage(float *buffer, float new_value)
{
  buffer[filter_index] = new_value;
  float sum = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++)
    {
      sum += buffer[i];
    }
    return sum / FILTER_SIZE;
}

float lpf(float prev_ema, float new_value, float alpha)
{
  return (1.0 - alpha) * new_value + alpha * prev_ema;
}
// 실시간 이동 표준편차
float MovingStd(float *buffer)
{
  float mean = MovingAverage(buffer, buffer[filter_index]);
  float var = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++)
  {
    var += (buffer[i] - mean) * (buffer[i] - mean);
  }
  var /= FILTER_SIZE;
  return sqrt(var);
}


int16_t ax, ay, az, gx, gy, gz;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;

float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
float angleX_ema = 0.0, angleY_ema = 0.0, angleZ_ema = 0.0;
unsigned long startTime = 0; // 프로그램 시작 시간 저장
unsigned long lastTime = 0;

void Parachute_ejection() {
  if (ejectionStarted == false) {
    // 사출 시작
    ejectionStarted = true;
    ejectionStartTime = millis();
    digitalWrite(ignitor_relay_pin, HIGH); // 릴레이 ON (사출)
  }

  // 사출 시작 후 2초가 지났다면 릴레이 OFF
  if (ejectionStarted == true && millis() - ejectionStartTime >= 2000) {
    digitalWrite(ignitor_relay_pin, LOW); // 릴레이 OFF
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 연결 실패");
    while (1);
  }

  Serial.println("영점 조절 중... MPU6050을 고정하세요.");
  delay(2000);

  // 자이로 오프셋 측정
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < 100; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(5);
  }

  gx_offset = sumX / 100;
  gy_offset = sumY / 100;
  gz_offset = sumZ / 100;

  Serial.println("자이로 영점 설정 완료");

  pinMode(ignitor_relay_pin, OUTPUT);

  startTime = millis(); // 시작 시간 저장
  lastTime = millis();
}

void loop() {
  // 경과 시간 계산 (millis() 사용)
  unsigned long elapsedTime = millis() - startTime;
  unsigned long minutes = elapsedTime / 60000; // 분 계산
  unsigned long seconds = (elapsedTime % 60000) / 1000; // 초 계산
  unsigned long milliseconds = (elapsedTime % 1000); //  밀리초 그대로 표시

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 센서 값 읽기
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 자이로 오프셋 보정
  int gx_corr = gx - gx_offset;
  int gy_corr = gy - gy_offset;
  int gz_corr = gz - gz_offset;

  // 각속도 → °/s
  float gx_dps = gx_corr / 131.0;
  float gy_dps = gy_corr / 131.0;
  float gz_dps = gz_corr / 131.0;

  // 각도 적분 (단순 자이로)
  angleX += gx_dps * dt;
  angleY += gy_dps * dt;
  angleZ += gz_dps * dt;

  // x, y, z 이동평균 필터
  float angleX_avg = MovingAverage(angleX_buffer, angleX);
  float angleY_avg = MovingAverage(angleY_buffer, angleY);
  float angleZ_avg = MovingAverage(angleZ_buffer, angleZ);
  //float alpha = 0.2;

  // x, y, z 로우패스필터 출력값 
  angleX_ema = lpf(angleX_ema, angleX, ALPHA);
  angleY_ema = lpf(angleY_ema, angleY, ALPHA);
  angleZ_ema = lpf(angleZ_ema, angleZ, ALPHA);

  angleX_ema_buffer[filter_index] = angleX_ema;
  angleY_ema_buffer[filter_index] = angleY_ema;
  angleZ_ema_buffer[filter_index] = angleZ_ema;

  // lpf 기반 이동 평균 -> 이동 표준 편차 구하는 용도
  float angleX_ema_avg = MovingAverage(angleX_ema_buffer, angleX_ema);
  float angleY_ema_avg = MovingAverage(angleY_ema_buffer, angleY_ema);
  float angleZ_ema_avg = MovingAverage(angleZ_ema_buffer, angleZ_ema);

  float angleX_std = MovingStd(angleX_ema_buffer);
  float angleY_std = MovingStd(angleY_ema_buffer);
  float angleZ_std = MovingStd(angleZ_ema_buffer);

  if (fabs(angleX_ema - angleX_ema_avg) > 1.0 * angleX_std)
  {
    flag_X = true;
    Serial.println("X Deploy!");
  }
  // 예외처리 : bool 값 초기화
  else
  {
    flag_X = false;
  }

  if (fabs(angleY_ema - angleY_ema_avg) > 1.0 * angleY_std)
  {
    flag_Y = true;
    Serial.println("Y Deploy!");
  }

  else
  {
    flag_Y = false;
  }

  if (fabs(angleZ_ema - angleZ_ema_avg) > 1.0 * angleZ_std)
  {
    flag_Z = true;
    Serial.println("Z Deploy!");
  }

  else
  {
    flag_Z = false;
  }

  if (flag_X && flag_Y && flag_Z)
  {
    Serial.println("Deploy!");
    Parachute_ejection();
  }

  filter_index = (filter_index + 1) % FILTER_SIZE;
  // 가속도 값 → g → m/s²
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float ax_ms2 = ax_g * 9.80665;
  float ay_ms2 = ay_g * 9.80665;
  float az_ms2 = az_g * 9.80665;

  // 출력
  Serial.print("[Launch Monitor] | ");
  Serial.print("Elapsed Time: "); Serial.print(elapsedTime); Serial.print("ms | ");
  Serial.print("Roll(X): "); Serial.print(angleX, 2); Serial.print(" | ");
  Serial.print("Pitch(Y): "); Serial.print(angleY, 2); Serial.print(" | ");
  Serial.print("Yaw(Z): "); Serial.print(angleZ, 2); Serial.print(" | ");

  Serial.print("SMA Value: ");
  Serial.print("Roll(X): "); Serial.print(angleX_avg, 2); Serial.print(" | ");
  Serial.print("Pitch(Y): "); Serial.print(angleY_avg, 2); Serial.print(" | ");
  Serial.print("Yaw(Z): "); Serial.print(angleZ_avg, 2); Serial.print(" | ");

  Serial.print("LPF Value: ");
  Serial.print("Roll(X): "); Serial.print(angleX_ema, 2); Serial.print(" | ");
  Serial.print("Pitch(Y): "); Serial.print(angleY_ema, 2); Serial.print(" | ");
  Serial.print("Yaw(Z): "); Serial.print(angleZ_ema, 2); Serial.print(" | ");

  Serial.print("X: "); Serial.print(ax_ms2, 2); Serial.print(" | ");
  Serial.print("Y: "); Serial.print(ay_ms2, 2); Serial.print(" | ");
  Serial.print("Z: "); Serial.print(az_ms2, 2); Serial.println(" | ");
  
  
  delay(100);
}
