// MPU6050 모듈 사용

#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;
#define FILTER_SIZE 10
#define ALPHA 0.2

float angleX_buffer[FILTER_SIZE] = {0};
float angleY_buffer[FILTER_SIZE] = {0};
float angleZ_buffer[FILTER_SIZE] = {0};
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

int16_t ax, ay, az, gx, gy, gz;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;

float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
float angleX_ema = 0.0, angleY_ema = 0.0, angleZ_ema = 0.0;
unsigned long startTime = 0; // 프로그램 시작 시간 저장
unsigned long lastTime = 0;



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
