/*
   arduino ==> 셀프밸런싱로봇을 만들어 실험해본 코드
 */
//------------------------------------------------------------
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
#define OUTPUT_TEAPOT 1                 // MPU6050 sensor visualizer 
#define INTERRUPT_PIN 2                 // pin for MPU6050
#define OUTPUT_READABLE_YAWPITCHROLL    // yaw, pitch, roll
#define MIN_ABS_SPEED 30                // min velocity of motor


/* MPU */ 
// MPU Instantiation
MPU6050 mpu;

bool dmpReady = false;       // set true if DMP init was successful
uint8_t mpuIntStatus;        // holds actual interrupt status byte from MPU
uint8_t devStatus;           // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;         // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;          // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];      // FIFO storage buffer
  
// MPU6050 Yaw, Pitch, Roll values
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/* Interrupt */ 
volatile bool mpuInterrupt = false; 


/* Motor control */
int STBY = 13; // 
int PWMA = 10;
int PWMB = 11;
int AIN1 = 12;
int AIN2 = 6;
int BIN1 = 8;
int BIN2 = 9;
  

/* PID control */
// PID control hyperparameter
double kp = 21;
double ki = 140;
double kd = 0.8;
double input, output;
 
// variables for target angle
double originalSetpoint = 184.0;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
 
// PID instantiation
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
 

/* DMP : motion processor that computes acceleration */
void dmpDataReady() {
    mpuInterrupt = true;
}
 
void setup(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
 
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
 
    // mpu initialization
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    // mpu commication check
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
    // mpu default offset
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    // dmp initilization
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
    // devStatus 
    if (devStatus == 0){
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // Interrupt init
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // mpu packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
 
        // PID init
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    } else {   
        // mpu 6050 malfunctioning
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 
    /* Motor */
    pinMode (STBY, OUTPUT);
    pinMode (PWMA, OUTPUT);
    pinMode (AIN2, OUTPUT);
    pinMode (AIN1, OUTPUT);
    pinMode (PWMB, OUTPUT);
    pinMode (BIN1, OUTPUT);
    pinMode (BIN2, OUTPUT);

    // Motor off
    digitalWrite(AIN2,LOW);
    digitalWrite(AIN1,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);
}
 
 
void loop(){
    // error
    if (!dmpReady) return; 
 
    // PID control
    while (!mpuInterrupt && fifoCount < packetSize){
        pid.Compute(); 

        // logging
        Serial.print(input); 
        Serial.print(" =>"); Serial.println(output);
               
        if (input>50 && input<300){
            if (output>0) {
                Forward(); 
            }
            else if (output<0) {
                Reverse(); 
            }
        else {
            Stop();
        }
    }
 
    /* MPU processing */
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
 
    if ((mpuIntStatus & 0x10) || fifoCount == 1024){
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02){
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 
        // PID input
        input = ypr[1] * 180/M_PI + 180;
    }
}

void Forward() {
    Serial.print("F"); 
    Move(1,output,1);
    Move(2,output,1);
}

void Reverse() {
    Serial.print("R");
    Move(1,output,0);
    Move(2,output,0); 
}

void Stop() {
    Serial.print("S");
    digitalWrite(STBY, LOW); 
}

void Move(int motor, double speed, int direction) {
    digitalWrite(STBY, HIGH); 

    boolean inPin1;
    boolean inPin2;

    if (direction == 1) {
        inPin1 = HIGH;
        inPin2 = LOW;
    } else {
        inPin1 = LOW;
        inPin2 = HIGH;
    }

    analogWrite(PWMA, speed);
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMB, speed);
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
}
