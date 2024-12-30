#include <MPU6050_light.h>
#include <Wire.h>
#include <Servo.h>

// Pin Definitions
#define TRIGGER_PIN_LEFT 11
#define ECHO_PIN_LEFT 2
#define TRIGGER_PIN_FRONT 13
#define ECHO_PIN_FRONT 3
#define LIFT_FAN_PIN 5
#define THRUST_FAN_PIN 6
#define SERVO_PIN 9

// Thresholds for Obstacle Detection
const int THRESHOLD_FRONT = 20;
const int THRESHOLD_SIDE = 60;

// Global Variables
Servo servoMotor;
MPU6050 mpu(Wire);
float refAngle = 0.0; // Reference angle for stabilization
int distanceFront = 0;
int distanceSide = 0;

// Function Prototypes
void initializeHardware();
void stabilizeHovercraft();
float calculateDistance(int triggerPin, int echoPin);
int getYawAngle();
void shutDownHovercraft();
void startUpHovercraft();
void turnLeft(float initialYaw);
void turnRight(float initialYaw);
void goStraight(float distance);
void printSensorData(float frontDistance, float sideDistance);
bool isHovercraftStuck(float previousYaw);
bool isHovercraftRotating(float initialYaw, bool rotatingRight);

// Arduino Setup
void setup() {
    Serial.begin(9600);
    initializeHardware();
    Serial.println("Calculating offsets. Please do not move MPU6050.");
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println("Offsets calculated!");
    refAngle = getYawAngle();
    delay(2000);
}

// Main Loop
void loop() {
    mpu.update();
    stabilizeHovercraft();

    // Get sensor distances
    distanceFront = calculateDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
    distanceSide = calculateDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
    printSensorData(distanceFront, distanceSide);

    // Obstacle avoidance logic
    if (distanceFront <= THRESHOLD_FRONT) {
        shutDownHovercraft();
        delay(1000);

        distanceSide = calculateDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
        if (distanceSide > THRESHOLD_SIDE) {
            Serial.println("Turning left...");
            turnLeft(getYawAngle());
        } else {
            Serial.println("Turning right...");
            turnRight(getYawAngle());
        }
    } else {
        goStraight(distanceFront);
    }
}

// Function Implementations

void initializeHardware() {
    pinMode(TRIGGER_PIN_LEFT, OUTPUT);
    pinMode(ECHO_PIN_LEFT, INPUT);
    pinMode(TRIGGER_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);

    digitalWrite(LIFT_FAN_PIN, LOW);
    digitalWrite(THRUST_FAN_PIN, LOW);

    servoMotor.attach(SERVO_PIN);
    servoMotor.write(90);

    Wire.begin();
    mpu.begin();
    Wire.setWireTimeout(3000, true);
}

void stabilizeHovercraft() {
    int yaw = getYawAngle();
    int servoAngle;

    if (yaw >= 300) {
        servoAngle = 90 - (360 - yaw);
    } else if (yaw <= 60) {
        servoAngle = 90 + yaw;
    } else if (yaw > 60 && yaw < 120) {
        servoAngle = yaw;
    } else if (yaw > 240 && yaw < 300) {
        servoAngle = 180 - (360 - yaw);
    } else {
        servoAngle = 90 - (180 - yaw);
    }

    servoMotor.write(servoAngle % 360);
    delay(10);
}

float calculateDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    return (duration / 2) * 0.0343; // Convert duration to distance in cm
}

int getYawAngle() {
    mpu.update();
    int yaw = (int)mpu.getAngleZ() % 360;
    return (yaw < 0) ? (360 + yaw) : yaw;
}

void shutDownHovercraft() {
    Serial.println("Shutting down hovercraft!");
    digitalWrite(LIFT_FAN_PIN, LOW);
    digitalWrite(THRUST_FAN_PIN, LOW);
}

void startUpHovercraft() {
    analogWrite(LIFT_FAN_PIN, 255);
    analogWrite(THRUST_FAN_PIN, 220);
}

void turnLeft(float initialYaw) {
    Serial.println("Turning left...");
    float yaw = initialYaw;
    long startTime = millis();

    servoMotor.write(17); // Rotate servo left
    analogWrite(LIFT_FAN_PIN, 255);
    analogWrite(THRUST_FAN_PIN, 175);

    while (isHovercraftRotating(initialYaw, false)) {
        if (isHovercraftStuck(yaw) && millis() - startTime > 3000) break;
        yaw = getYawAngle();
    }

    shutDownHovercraft();
    delay(500);
    startUpHovercraft();
}

void turnRight(float initialYaw) {
    Serial.println("Turning right...");
    float yaw = initialYaw;
    long startTime = millis();

    servoMotor.write(163); // Rotate servo right
    analogWrite(LIFT_FAN_PIN, 255);
    analogWrite(THRUST_FAN_PIN, 175);

    while (isHovercraftRotating(initialYaw, true)) {
        if (isHovercraftStuck(yaw) && millis() - startTime > 3000) break;
        yaw = getYawAngle();
    }

    shutDownHovercraft();
    delay(500);
    startUpHovercraft();
}

void goStraight(float distance) {
    if (distance >= 75 && distance < 83) {
        analogWrite(LIFT_FAN_PIN, 0);
        delay(1000);
    } else {
        startUpHovercraft();
    }
}

void printSensorData(float frontDistance, float sideDistance) {
    Serial.println("Sensor Data:");
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
    Serial.print("Side Distance: ");
    Serial.println(sideDistance);
    Serial.print("Yaw Angle: ");
    Serial.println(getYawAngle());
}

bool isHovercraftStuck(float previousYaw) {
    return abs(getYawAngle() - previousYaw) <= 1;
}

bool isHovercraftRotating(float initialYaw, bool rotatingRight) {
    float yaw = getYawAngle();
    if (rotatingRight) return yaw >= 290 || yaw <= 70;
    return yaw <= 250 || yaw >= 110;
}
