//MPU6050
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 
#define MPU6050_ADDRESS_AD0_HIGH    0x69 
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  
Simple_MPU6050 mpu(Six_Axis_Quaternions);
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) 
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
int x_rotation;//angulo de rotacion
float ypr[3] = { 0, 0, 0 };
float xyz[3] = { 0, 0, 0 };
//pines por definir
//motores
const int ENA_MT1 = 10;
const int ENA_MT2 = 11;
const int IN1_MT = 2;
const int IN2_MT = 3;
const int IN3_MT = 4;
const int IN4_MT = 5;
int SPEED_MT=200;
//sensor de color
const int s0=6
const int s1=7
const int s2=8
const int s3=9
const int outTCS=12
//delay time
int dt=500;

void setup() {
  // put your setup code here, to run once:
//MPU6050 Inicializacion
  pinMode(7,OUTPUT);
  Serial.begin(115200);
  Serial.println(F("Start:"));
  mpu.begin();
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
    mpu.Set_DMP_Output_Rate_Hz(10);           // Set the DMP output rate from 200Hz to 5 Minutes.
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  mpu.CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  mpu.CalibrateMPU().Enable_Reload_of_DMP(Three_Axis_Quaternions).load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(print_Values);
//Motores inicializacion
  pinMode(IN1_MT,OUTPUT);
  pinMode(IN2_MT,OUTPUT);
  pinMode(IN3_MT,OUTPUT);
  pinMode(IN4_MT,OUTPUT);
  pinMode(ENA_MT1,OUTPUT);
  pinMode(ENA_MT2,OUTPUT);
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT);
  //sensor de color
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(outTCS,INPUT);
  digitalWrite(s0,1);
  digitalWrite(s1,0); 

}

void loop() {
  // put your main code here, to run repeatedly:
  //codigo de prueba de motores con giroscopio, sin encoders(usando tiempo), se pretende usar el avance con distancia no con tiempo.
  loop_mpu();
  ahead();
  delay(dt);
  stop(200);
  while(x_rotation<90){
    right();
  }
  stop(200);
  ahead();
  delay(dt);
  while(x_rotation>0){
    left();
  }
  stop(200);
  ahead();
  delay(dt);
  stop(500);

}



//Funciones de motores
void ahead(){
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
}

void back(){
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);

}
void stop(int x){
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);
  delay(x);

}
void right(){
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);

}
void left(){
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);

}
//senosr de color
void getcolor(){//devuelve el color
// 1:morado
// 2:rosa
// 3:amarillo
// 4:negro
  digitalWrite(s2,0);
  digitalWrite(s3,0);
  int red=pulseIn(outTCS,0);
  delay(20);
  
  digitalWrite(s2,1);
  digitalWrite(s3,1);
  int green=pulseIn(outTCS,0);
  delay(20);

  digitalWrite(s2,0);
  digitalWrite(s3,1);
  int blue=pulseIn(outTCS,0);
  delay(20);

if(green>red && green>blue && red>blue && blue<200){
  // Serial.println("Color MORADO");
  return 1;
}

else if(green>red && green>blue && blue>red){
  // Serial.println("Color ROSA");
  return 2;

}

else if(blue>green && blue>red && green>red && red<200 ){
  // Serial.println("Color AMARILLO");
  return 3;
}

else if(green>200 && blue>200 && red>200){
  // Serial.println("Color NEGRO");
    return 4;
}

// else if(green<100 && blue<100 && red<100 && (green-red)<20 && (green-blue)<20){
//   Serial.println("Color Blanco");
// }

//else{
    //Serial.println("Color no encontrado");
//}

void setColors(){//devuelve valores RGB
  digitalWrite(s2,0);
  digitalWrite(s3,0);
  int red=pulseIn(outTCS,1);
  delay(200);
  
  digitalWrite(s2,1);
  digitalWrite(s3,1);
  int green=pulseIn(outTCS,1);
  delay(200);

  digitalWrite(s2,0);
  digitalWrite(s3,1);
  int blue=pulseIn(outTCS,1);
  delay(200);

Serial.print("Red:");
Serial.print(red);
Serial.print(" Green:");
Serial.print(green);
Serial.print(" Blue:");
Serial.println(blue);
}
//funciones de MPU6050
void loop_mpu(){
  digitalWrite(8, !digitalRead(8));
  digitalWrite(9, !digitalRead(9));
  mpu.dmp_read_fifo(0);
  x_rotation=xyz[1];
}

void PrintValues(int32_t *quat, uint16_t SpamDelay = 100) {
  uint8_t WhoAmI;
  Quaternion q;
  VectorFloat gravity;
  
  spamtimer(SpamDelay) {
    mpu.WHO_AM_I_READ_WHOAMI(&WhoAmI);
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);

  }
}

void ChartValues(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
  }
}

void PrintAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
  }
}

void ChartAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
  }
}

void PrintQuaternion(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
  }
}

void PrintEuler(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  float euler[3];         
  float eulerDEG[3];         
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
    mpu.GetEuler(euler, &q);
    mpu.ConvertToDegrees(euler, eulerDEG);
  }
}

void PrintRealAccel(int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  VectorInt16 aa, aaReal;
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
  }
}


void PrintWorldAccel(int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  VectorInt16 aa, aaReal, aaWorld;
  spamtimer(SpamDelay) {
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
    mpu.GetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }
}

void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  uint8_t Spam_Delay = 10; // Built in Blink without delay timer preventing Serial.print SPAM
  PrintValues(quat, Spam_Delay);
}
