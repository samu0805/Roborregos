#include <PID_v1.h>//libreria PID
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
int z_rotation;//angulo de rotacion
float ypr[3] = { 0, 0, 0 };
float xyz[3] = { 0, 0, 0 };
//Motores
const int ENA_MT1 = 50;
const int ENA_MT2 = 51;
const int IN1_MT = 52;
const int IN2_MT = 53;
const int IN3_MT = 4;
const int IN4_MT = 5;
int SPEED_MT=100;//velocidad inicial
//encoders-variables de control
const int encoder=3;
const int encoder2;//por definir
int state;
int c;
int last;
int frenado;//avance del motor en el frenado


// Variables para PID
double setpoint; // Valor deseado de velocidad
double input;    // Valor actual de velocidad
double output;   // Salida del PID
unsigned long time1;


// Parámetros del PID
double Kp = 1.0; // Ganancia proporcional
double Ki = 2.0; // Ganancia integral
double Kd = 0.001; // Ganancia derivativa

// Inicializa el PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int dt=500;
int orientacion=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  inicializarMPU6050();//MPU6050 Inicializacion
  //motores
  pinMode(IN1_MT,OUTPUT);
  pinMode(IN2_MT,OUTPUT);
  pinMode(IN3_MT,OUTPUT);
  pinMode(IN4_MT,OUTPUT);
  pinMode(ENA_MT1,OUTPUT);
  pinMode(ENA_MT2,OUTPUT);
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT);
  //encoder
  pinMode(encoder,INPUT);
  attachInterrupt(1,interruption,RISING);//interrupcion para contar pulsos del encoder

    //PID
    //30cm en 1.5s=58RPM
    //en 2s=43RPM
  setpoint = 110; //RPM deseadas
  myPID.SetMode(AUTOMATIC);
  calibracion_MT();  //calibracion de velocidad con PID
}

void loop() {
  //codigo prueba
  loop_mpu();
  ahead(1);
  stop(300);
  right();
  stop(200);
  back(1);
  stop(300);
  left();
  stop(200); 
}
//CALIBRACION DE VELOCIDAD CON PID
void calibracion_MT(){
  time1=millis();
  while((millis()-time1)<5000){
    input = (c * 15); // Convertir pulsos a RPM (ajusta según tu encoder)
    c = 0; // Reinicia el contador para la siguiente medición

    // Calcula el PID
    myPID.Compute();

    // Controla el motor
    analogWrite(ENA_MT1, output); // Establece la velocidad del motor
    digitalWrite(IN1_MT, HIGH); // Establece la dirección
    digitalWrite(IN2_MT, LOW);

    // Imprime información en el monitor serie
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" | Input: ");
    Serial.print(input);
    Serial.print(" | Output: ");
    Serial.println(output);

    delay(100); // Espera un poco antes de la siguiente lectura

  }
  //calculo del avance del motor en el frenado
  c=0;
  while(c<=40){
    Serial.print("");
  }
  stop(500);
  frenado=c-40;
  SPEED_MT=output;//establecer velocidad ideal
}
//funcion contabilizadora de pulsos del encoder
void interruption(){
  state=digitalRead(encoder);
  if(state!=last){
    c++;
    last=state;
    Serial.println(c);
  }
}
//
void units(int x){//establece el numero de unidades que avanzara el robot 40pulsos=rev(20.7cm), 58pulsos=30cm
  c=0;
  while(c<=x*(58-frenado)){
    Serial.print("");
  }
  c=0;
}
void ahead(int x){//avance adelnate, la variable x indica cuantas unidades desea que avance
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
  units(x);
  stop(0);
}

void back(int x){//retroceso
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
  units(x);
  stop(0);
}
void stop(int x){
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);
  delay(x);

}
void setright(){//setear orientacion de motores
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
}
void setleft(){//setear orientacion de motores
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);
}
void right(){//girar a la derecha
  if(orientacion==0){
    while(z_rotation<90){
      loop_mpu();  
      setright();
    }
    stop(0);
    orientacion=90;
  }
  else if(orientacion==90){
    while(z_rotation<=179.9){
      loop_mpu();
      setright();
    }
    stop(0);
    orientacion=180;    
  }
  else if(orientacion==180){
    while(z_rotation>-90.1 && z_rotation<-89.9){
      loop_mpu();
      setright();
    }
    stop(0);
    orientacion=-90;    
  }
  else if(orientacion==-90){
    while(z_rotation<0){
      loop_mpu();
      setright();
    }
    stop(0);
    orientacion=180;    
  }  

}
void left(){//girar a la izquierda
    if(orientacion==0){
    while(z_rotation>-90){
      loop_mpu();
      setleft();
    }
    stop(0);
    orientacion=-90;
  }
  else if(orientacion==-90){
    while(z_rotation>=-179.9){
      loop_mpu();
      setleft();
    }
    stop(0);
    orientacion=180;    
  }
  else if(orientacion==180){
    while(z_rotation>90){
      loop_mpu();
      setleft();
    }
    stop(0);
    orientacion=90;    
  }
  else if(orientacion==90){
    while(z_rotation>0){
      loop_mpu();
      setright();
    }
    stop(0);
    orientacion=90;    
  }  

}
//////////////7funciones de MPU6050/////////////////////
void loop_mpu(){
  digitalWrite(8, !digitalRead(8));
  digitalWrite(9, !digitalRead(9));
  mpu.dmp_read_fifo(0);
  z_rotation=xyz[0];
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
void inicializarMPU6050(){
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
}
