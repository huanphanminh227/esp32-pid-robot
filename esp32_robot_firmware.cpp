/**
 * @file main.cpp
 * @brief Autonomous & Semi-Autonomous Competition Robot Firmware
 * @author Phan Minh Huan
 * @details Real-time closed-loop PID control and FSM running on ESP32.
 */

#include <Arduino.h>

// --- PIN DEFINITIONS ---
// Motor Driver (TB6612FNG / L298N)
#define MOTOR_LEFT_PWM      18
#define MOTOR_RIGHT_PWM     19
#define MOTOR_LEFT_DIR1     16
#define MOTOR_LEFT_DIR2     17
#define MOTOR_RIGHT_DIR1    21
#define MOTOR_RIGHT_DIR2    22

// PWM Channels & Parameters
#define PWM_FREQ            10000 // 10 kHz
#define PWM_RES             8     // 8-bit resolution (0 - 255)
#define PWM_CHAN_LEFT       0
#define PWM_CHAN_RIGHT      1
#define BASE_SPEED          160

// Ultrasonic Sensor (HC-SR04)
#define PIN_TRIG            13
#define PIN_ECHO            12
#define OBSTACLE_DISTANCE_CM 15.0

// 5-Channel IR Sensor Array
const uint8_t irPins[5] = {34, 35, 32, 33, 25};

// --- FINITE STATE MACHINE (FSM) ---
enum RobotState {
    STATE_IDLE,
    STATE_TRACKING,
    STATE_AVOIDING,
    STATE_MANUAL
};

volatile RobotState currentState = STATE_TRACKING;

// --- PID CONTROLLER STRUCTURE ---
struct PIDController {
    float kp = 28.0f;
    float ki = 0.05f;
    float kd = 14.0f;
    float integral = 0.0f;
    float lastError = 0.0f;
} pid;

// --- FUNCTION PROTOTYPES ---
void initHardware();
void setMotorSpeeds(int16_t left, int16_t right);
int8_t readLinePosition();
float measureDistanceCM();
void updatePIDControl();
void executeObstacleAvoidance();

void setup() {
    Serial.begin(115200);
    initHardware();
    Serial.println(F("[SYSTEM] ESP32 Robot Initialized Successfully."));
}

void loop() {
    float distance = measureDistanceCM();

    // State Transitions
    if (distance > 0 && distance < OBSTACLE_DISTANCE_CM && currentState == STATE_TRACKING) {
        currentState = STATE_AVOIDING;
    }

    switch (currentState) {
        case STATE_TRACKING:
            updatePIDControl();
            break;

        case STATE_AVOIDING:
            executeObstacleAvoidance();
            currentState = STATE_TRACKING; // Resume tracking after maneuver
            break;

        case STATE_IDLE:
        default:
            setMotorSpeeds(0, 0);
            break;
    }

    delay(5); // 200 Hz loop frequency
}

// --- HARDWARE INITIALIZATION ---
void initHardware() {
    pinMode(MOTOR_LEFT_DIR1, OUTPUT);
    pinMode(MOTOR_LEFT_DIR2, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR2, OUTPUT);

    ledcSetup(PWM_CHAN_LEFT, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CHAN_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHAN_LEFT);
    ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHAN_RIGHT);

    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

    for (uint8_t i = 0; i < 5; ++i) {
        pinMode(irPins[i], INPUT);
    }
}

// --- SENSOR READING & ERROR CALCULATION ---
int8_t readLinePosition() {
    // Sensor readings: 1 = line detected, 0 = background
    uint8_t s[5];
    for (uint8_t i = 0; i < 5; ++i) {
        s[i] = digitalRead(irPins[i]);
    }

    // Weighted position calculation: weights = [-2, -1, 0, 1, 2]
    int8_t sum = 0;
    int8_t activeCount = 0;
    const int8_t weights[5] = {-2, -1, 0, 1, 2};

    for (uint8_t i = 0; i < 5; ++i) {
        if (s[i] == 1) {
            sum += weights[i];
            activeCount++;
        }
    }

    return (activeCount > 0) ? (sum / activeCount) : pid.lastError;
}

// --- PID CONTROL ALGORITHM ---
void updatePIDControl() {
    float error = (float)readLinePosition();

    pid.integral += error;
    pid.integral = constrain(pid.integral, -100.0f, 100.0f); // Anti-windup
    float derivative = error - pid.lastError;

    float correction = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);
    pid.lastError = error;

    int16_t leftSpeed = constrain((int16_t)(BASE_SPEED + correction), 0, 255);
    int16_t rightSpeed = constrain((int16_t)(BASE_SPEED - correction), 0, 255);

    setMotorSpeeds(leftSpeed, rightSpeed);
}

// --- MOTOR ACTUATION ---
void setMotorSpeeds(int16_t left, int16_t right) {
    digitalWrite(MOTOR_LEFT_DIR1, left >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_LEFT_DIR2, left >= 0 ? LOW : HIGH);
    ledcWrite(PWM_CHAN_LEFT, abs(left));

    digitalWrite(MOTOR_RIGHT_DIR1, right >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, right >= 0 ? LOW : HIGH);
    ledcWrite(PWM_CHAN_RIGHT, abs(right));
}

// --- ULTRASONIC RANGING ---
float measureDistanceCM() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duration = pulseIn(PIN_ECHO, HIGH, 25000); // 25ms timeout
    if (duration == 0) return -1.0f;
    return (float)duration * 0.0343f / 2.0f;
}

// --- OBSTACLE AVOIDANCE ROUTINE ---
void executeObstacleAvoidance() {
    setMotorSpeeds(0, 0);       // Halt
    delay(100);
    setMotorSpeeds(-140, 140);   // Pivot right
    delay(280);
    setMotorSpeeds(150, 150);   // Forward bypass
    delay(400);
    setMotorSpeeds(140, -140);   // Pivot back to line
    delay(280);
}