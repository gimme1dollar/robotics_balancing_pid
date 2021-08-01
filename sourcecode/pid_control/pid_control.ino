/*
   arduino ==> 셀프밸런싱로봇을 만들어 실험해본 코드
 */
//------------------------------------------------------------
// PID 제어, 모터 제어, MPU6050센서 값을 받기 위한 선언문
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
 
//------------------------------------------------------------
#define OUTPUT_TEAPOT 1                    // Processing을 통해 MPU6050 센서를 Visualize 하고 싶은 경우 1, 아니면 0으로 선언합니다
#define MIN_ABS_SPEED 30                  // 모터의 최저속도를 설정합니다.   0 ~ 255 값 중 선택
#define OUTPUT_READABLE_YAWPITCHROLL    // Yaw, Pitch, Roll 값을 얻기 위해 선언합니다
#define INTERRUPT_PIN 2                  // MPU6050 센서의 INT 핀이 꽂혀있는 번호를 설정합니다. 보통 2번

 
 
//------------------------------------------------------------
//MPU 객체를 선언합니다
MPU6050 mpu;
// MPU control/status vars

bool dmpReady = false;        // set true if DMP init was successful
uint8_t mpuIntStatus;        // holds actual interrupt status byte from MPU
uint8_t devStatus;            // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;            // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];        // FIFO storage buffer
 
 
//------------------------------------------------------------
// MPU6050 센서를 통해 쿼터니언과 오일러각, Yaw, Pitch, Roll 값을 얻기 위해 선언합니다
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
 
//------------------------------------------------------------
// Processing으로 MPU6050 센서를 Visualize 하기 위한 변수
// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
 
 
//------------------------------------------------------------
// PID 제어용 변수 선언
double kp = 21;
double ki = 140;
double kd = 0.8;
 
 
// 기울일 각도 선택 
// 제가 만든 밸런싱로봇에는 184.0도가 가장 최적의 평형각도였습니다
// 각도가 180도를 기준으로 +-를 설정해주시면 됩니다
double originalSetpoint = 184.0;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
 
// PID 제어용 input, output 변수를 선언합니다
double input, output;
 
// PID값을 설정해준다
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
 
 
// 모터 제어용 변수 선언
// EnA, EnB는 속도제어용(pwm), IN1,2,3,4는 방향제어용 핀입니다
int STBY=13;
int PWMA = 10;
int AIN1 = 12;
int AIN2 = 6;
int BIN1 = 8;
int BIN2 = 9;
int PWMB = 11;
  
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // MPU6050의 인터럽트 발생유무 확인
 
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
 
    // MPU6050 초기화
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // MPU6050 통신확인
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
    // 키입력을 기다리는 코드. 주석처리했습니다
    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
 
 
    //DMP 초기화
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
    // 기본 옵셋값 설정
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip
 
    // devStatus 값이 0 이면 정상작동, 0이 아니면 오작동입니다
    // make sure it worked (returns 0 if so)
    if (devStatus == 0){
        //]DMP 가동
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // 아두이노 인터럽트 설정
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // DMP 사용가능 상태 설정
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // 패킷사이즈 가져오기
        packetSize = mpu.dmpGetFIFOPacketSize();
 
 
        // MPU6050 센서가 정삭작동하면 PID 제어용 코드를 초기화합니다
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else{   // MPU6050 센서가 오작동한 경우
        // ERROR!
        // 1 = 초기 메모리 에러
        // 2 = DMP 설정 오류
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 
        //모터 출력핀 초기화
    pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode (AIN2, OUTPUT);
    pinMode (AIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode (BIN1, OUTPUT);
    pinMode (BIN2, OUTPUT);

   //모터 동작 OFF
    digitalWrite(AIN2,LOW);
    digitalWrite(AIN1,LOW);
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,LOW);
}
 
 
void loop(){
    // 오류시 작업중지
    if (!dmpReady) return;
 
    // MPU 인터럽트나 패킷 대기
    while (!mpuInterrupt && fifoCount < packetSize){
        //MPU6050 데이터가 없는 경우 PID 계산
 
        pid.Compute(); // 루프를 돌면서 pid 값을 업데이트합니다
        //시리얼 모니터로 현재 상태 출력
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
               
        if (input>50 && input<300){//로봇이 기울어지는 경우(각도 범위내에서만)
          
        if (output>0) //앞으로 기울어지는 경우
        Forward(); //전진
        else if (output<0) //뒤로 기울어지는 경우
        Reverse(); //후진
        }
        else //로봇이 기울어지지 않은 경우
        Stop(); //모터 정지
    }
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // MPU6050 센서가 정상작동하는 경우에만 PID제어를 해야하므로 아래와 같이 if-else문을 작성합니다
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024){
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    // MPU6050 센서가 정상작동하는 경우
    else if (mpuIntStatus & 0x02){
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
 
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 
/*#ifdef OUTPUT_READABLE_YAWPITCHROLL  // 센서를 통해 구한 Yaw, Pitch, Roll 값을 Serial Monitor에 표시합니다
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
#endif*/
        // PID 제어를 하기 위해 input 변수에 Pitch 값을 넣습니다
        input = ypr[1] * 180/M_PI + 180;
    }
}
/*#ifdef OUTPUT_TEAPOT  // Processing으로 MPU6050센서의 움직임을 Visualize 하기 위한 코드
        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        Serial.write(teapotPacket, 14);
        teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
    }
}*/
void Forward() //전진
{
   move(1,output,1);
   move(2,output,1);
    Serial.print("F"); 
}

void Reverse() //후진
{
  move(1,output,0);
  move(2,output,0); 
    Serial.print("R");
}

void Stop() //정지
{
   stop();
    Serial.print("S");
}
void move(int motor, double speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}
