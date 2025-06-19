#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu;

#define TRIGGER_THRESHOLD 45.0
#define PI 3.14159265358979323846

int ignitor_relay_pin = 6;
bool ejectionStarted = false;
unsigned long ejectionStartTime = 0;

// 센서 안정화
bool isStabilizing = true;
unsigned long stabilizeStartTime = 0;

int16_t ax, ay, az, gx, gy, gz;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;

unsigned long startTime = 0;
unsigned long lastTime = 0;

float KalAngleX = 0.0, KalBiasX = 0.0;
float KalAngleY = 0.0, KalBiasY = 0.0;
float P_X[2][2] = {{1,0}, {0,1}};
float P_Y[2][2] = {{1,0}, {0,1}};

float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;

float prev_KalX = 0.0;
float prev_KalY = 0.0;

bool firstRun = true;

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias, float P[2][2]) {
    newRate -= bias;
    angle += newRate * dt;
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias  += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

void Parachute_ejection() {
    if (!ejectionStarted) {
        ejectionStarted = true;
        ejectionStartTime = millis();
        digitalWrite(ignitor_relay_pin, HIGH);
    }
    if (ejectionStarted && millis() - ejectionStartTime >= 2000) {
        digitalWrite(ignitor_relay_pin, LOW);
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU Connection Failed!");
        while(1);
    }

    long sumX = 0, sumY = 0, sumZ = 0;
    delay(2000);
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

    pinMode(ignitor_relay_pin, OUTPUT);
    digitalWrite(ignitor_relay_pin, LOW);   // 릴레이 OFF

    startTime = millis();
    lastTime = millis();

    // 안정화 시간 기록
    stabilizeStartTime = millis();
    isStabilizing = true;

    // 칼만 초기화
    KalAngleX = 0.0;
    KalAngleY = 0.0;
    KalBiasX = 0.0;
    KalBiasY = 0.0;
    firstRun = true;

    // P 행렬 초기화
    P_X[0][0] = 1; P_X[0][1] = 0;
    P_X[1][0] = 0; P_X[1][1] = 1;
    P_Y[0][0] = 1; P_Y[0][1] = 0;
    P_Y[1][0] = 0; P_Y[1][1] = 1;
}

void loop() {
    unsigned long currentTime = millis();

    // 2초간 센서 안정화
    if (isStabilizing) {
        if (currentTime - stabilizeStartTime >= 2000) {
            isStabilizing = false;
            lastTime = currentTime;
            prev_KalX = 0.0;
            prev_KalY = 0.0;
        } else {
            // 안정화 중에는 이그나이터 OFF
            digitalWrite(ignitor_relay_pin, LOW);
            Serial.println("Sensor Stabilizing 2s...");
            // 50Hz 주기 유지
            while (millis() - currentTime < 20);
            return;
        }
    }

    float dt = (currentTime - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;
    lastTime = currentTime;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int gx_corr = gx - gx_offset;
    int gy_corr = gy - gy_offset;

    float gx_dps = gx_corr / 131.0;
    float gy_dps = gy_corr / 131.0;

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    // 좌표계 변환 : 구형 좌표계로 변환(각도와 각속도의 값을 얻기 위해)
    // Roll 값 : y, z평면에서 x축 기울기(x축 회전) roll = arctan(Ay/Ax)
    float accel_angleX = atan2(ay_g, az_g) * 180.0 / PI;
    //pitch 값 : x, z축의 가속도의 합성 -> x벡터와 z벡터의 내적
    float denom = sqrt(ay_g * ay_g + az_g * az_g);
    // 너무 값이 작아지는 것 및 오버플로우로 인한 메모리 누수 방지
    if (denom < 1e-6) denom = 1e-6;
    float accel_angleY = atan2(-ax_g, denom) * 180.0 / PI;

    if (firstRun) {
        KalAngleX = accel_angleX;
        KalAngleY = accel_angleY;
        firstRun = false;
    }

    float KalX = kalmanFilter(accel_angleX, gx_dps, dt, KalAngleX, KalBiasX, P_X);
    float KalY = kalmanFilter(accel_angleY, gy_dps, dt, KalAngleY, KalBiasY, P_Y);

    float angularVelocity_X = (KalX - prev_KalX) / dt;
    float angularVelocity_Y = (KalY - prev_KalY) / dt;

    bool trigger_X = fabs(KalX) > TRIGGER_THRESHOLD && fabs(angularVelocity_X) > TRIGGER_THRESHOLD;
    bool trigger_Y = fabs(KalY) > TRIGGER_THRESHOLD && fabs(angularVelocity_Y) > TRIGGER_THRESHOLD;

    if (trigger_X || trigger_Y) {
        //Serial.println("Deployed!");
        Parachute_ejection();
    }

    prev_KalX = KalX;
    prev_KalY = KalY;

    // 시리얼 플로터 전용 출력 (4개 값: 원시X, 칼만X, 원시Y, 칼만Y)
    Serial.print(accel_angleX); Serial.print(",");
    Serial.print(KalX); Serial.print(",");
    Serial.print(accel_angleY); Serial.print(",");
    Serial.println(KalY);

    // 50Hz 주기 유지 (20ms)
    while (millis() - currentTime < 20);
}
