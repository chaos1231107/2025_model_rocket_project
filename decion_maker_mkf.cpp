#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#define ALPHA 0.873

MPU6050 mpu;

#define TRIGGER_THRESHOLD 20.0
#define PI 3.14159265358979323846

int ignitor_relay_pin = 6;

//int falling_count = 0;
bool ejectionStarted = false;
unsigned long ejectionStartTime = 0;

bool isStabilizing = true;
unsigned long stabilizeStartTime = 0;

int16_t ax, ay, az, gx, gy, gz;
int gx_offset = 0, gy_offset = 0, gz_offset = 0;
float Yaw_prev = 0.0f;

unsigned long startTime = 0;
unsigned long lastTime = 0;

float KalAngleX = 0.0, KalBiasX = 0.0;
float KalAngleY = 0.0, KalBiasY = 0.0;
float KalAngleZ = 0.0, KalBiasZ = 0.0;

float P_X[2][2] = {{1,0}, {0,1}};
float P_Y[2][2] = {{1,0}, {0,1}};
float P_Z[2][2] = {{1,0}, {0,1}};

float Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;

float prev_Kal_Roll = 0.0f;
float prev_Kal_Pitch = 0.0f;
float prev_Kal_Yaw = 0.0f;

bool firstRun = true;

float lpf_angularVelocity_Roll = 0.0f;
float lpf_angularVelocity_Pitch = 0.0f;
float lpf_angularVelocity_Yaw = 0.0f;

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
    if (ejectionStarted && millis() - ejectionStartTime >= 2000.0f) {
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
    
    Serial.println("I2C Connection Succeded!");

    delay(2000);
    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < 100; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        sumX += gx;
        sumY += gy;
        sumZ += gz;
        delay(5);
    }

    gx_offset = sumX / 100.0f;
    gy_offset = sumY / 100.0f;
    gz_offset = sumZ / 100.0f;

    pinMode(ignitor_relay_pin, OUTPUT);
    digitalWrite(ignitor_relay_pin, LOW);

    startTime = millis();
    lastTime = millis();

    stabilizeStartTime = millis();
    isStabilizing = true;

    KalAngleX = 0.0f;
    KalAngleY = 0.0f;
    KalAngleZ = 0.0f;
    KalBiasX = 0.0f;
    KalBiasY = 0.0f;
    KalBiasZ = 0.0f;

    firstRun = true;
}

void loop() {
    unsigned long currentTime = millis();

    if (isStabilizing) {
        if (currentTime - stabilizeStartTime >= 3000.0f) {
            isStabilizing = false;
            lastTime = currentTime;
            prev_Kal_Roll = 0.0f;
            prev_Kal_Pitch = 0.0f;
            prev_Kal_Yaw = 0.0f;
        } else {
            digitalWrite(ignitor_relay_pin, LOW);
            Serial.println("Sensor Stabilizing 3s...");
            while (millis() - currentTime < 20);
            return;
        }
    }

    float dt = (currentTime - lastTime) / 1000.0f;
    if (dt <= 0) dt = 0.001;
    lastTime = currentTime;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int gx_corr = gx - gx_offset;
    int gy_corr = gy - gy_offset;
    int gz_corr = gz - gz_offset;

    float gx_dps = gx_corr / 131.0f;
    float gy_dps = gy_corr / 131.0f;
    float gz_dps = gz_corr / 131.0f;

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float Roll = atan2(ay_g, az_g) * 180.0f / PI;
    float denom = sqrt(ay_g * ay_g + az_g * az_g);
    if (denom < 1e-6) denom = 1e-6;
    float Pitch = atan2(-ax_g, denom) * 180.0f / PI;

    // Yaw : 자이로 적분 방식 사용
    float Yaw = Yaw_prev + gz_dps * dt;
    Yaw_prev = Yaw;

    if (Yaw < 0) Yaw += 360.0f;
    if (Yaw >= 360.0f) Yaw -= 360.0f;

    if (firstRun) {
        KalAngleX = Roll;
        KalAngleY = Pitch;
        KalAngleZ = Yaw;
        firstRun = false;
    }

    float Kal_Roll = kalmanFilter(Roll, gx_dps, dt, KalAngleX, KalBiasX, P_X);
    float Kal_Pitch = kalmanFilter(Pitch, gy_dps, dt, KalAngleY, KalBiasY, P_Y);
    float Kal_Yaw = kalmanFilter(Yaw, gz_dps, dt, KalAngleZ, KalBiasZ, P_Z);

    float angularVelocity_Roll = (Kal_Roll - prev_Kal_Roll) / dt;
    float angularVelocity_Pitch = (Kal_Pitch - prev_Kal_Pitch) / dt;
    float angularVelocity_Yaw = (Kal_Yaw - prev_Kal_Yaw) / dt;

    lpf_angularVelocity_Roll = ALPHA * lpf_angularVelocity_Roll + (1 - ALPHA) * angularVelocity_Roll;
    lpf_angularVelocity_Pitch = ALPHA * lpf_angularVelocity_Pitch + (1 - ALPHA) * angularVelocity_Pitch;
    lpf_angularVelocity_Yaw = ALPHA * lpf_angularVelocity_Yaw + (1 - ALPHA) * angularVelocity_Yaw;

    bool trigger_X = fabs(Kal_Roll) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Roll) > TRIGGER_THRESHOLD;
    bool trigger_Y = fabs(Kal_Pitch) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Pitch) > TRIGGER_THRESHOLD;
    bool trigger_Z = fabs(Kal_Yaw) > TRIGGER_THRESHOLD && fabs(lpf_angularVelocity_Yaw) > TRIGGER_THRESHOLD;

    if (trigger_X + trigger_Y + trigger_Z >= 2) {
        Serial.println("Deployed!");
        Parachute_ejection();
    }

    prev_Kal_Roll = Kal_Roll;
    prev_Kal_Pitch = Kal_Pitch;
    prev_Kal_Yaw = Kal_Yaw;

    Serial.print(Roll); Serial.print(",");
    Serial.print(Kal_Roll); Serial.print(",");
    Serial.print(Pitch); Serial.print(",");
    Serial.print(Kal_Pitch); Serial.print(",");
    Serial.print(Yaw); Serial.print(",");
    Serial.print(Kal_Yaw); Serial.println(",");
    //Serial.println(angularVelocity_Roll); Serial.print(",");
    //Serial.println(lpf_angularVelocity_Roll);

    while (millis() - currentTime < 20);
}
