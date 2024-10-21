#include <Servo.h>
#include <PID_v1.h>//libreria PID
//MPU6050
//PINES: SCL-21,SDA-20
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 
#define MPU6050_ADDRESS_AD0_HIGH    0x69 
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  
Simple_MPU6050 mpu(Six_Axis_Quaternions);
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) 
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
float z_rotation;//angulo de rotacion
float ypr[3] = { 0, 0, 0 };
float xyz[3] = { 0, 0, 0 };
//Motores
const int ENA_MT1 = 4;
const int ENA_MT2 = 5;
const int ENA_MT3 = 6;
const int ENA_MT4 = 7;
const int IN1_MT = 52;
const int IN2_MT = 53;
const int IN3_MT = 48;
const int IN4_MT = 49;

const int IN5_MT = 26;
const int IN6_MT = 27;
const int IN7_MT = 29;
const int IN8_MT = 28;
int SPEED_MT=140;//velocidad inicial
int SPEED_MT2=140;
//servo
Servo servo;
//encoders-variables de control
const int encoder=3;
const int encoder2=2;//por definir
int c;
int c2;
int cplus;
int cplus2;
int frenado;//avance del motor en el frenado
int frenado2;
int error_giro=5;
// Variables para PID
double setpoint; // Valor deseado de velocidad
double input;    // Valor actual de velocidad
double output;   // Salida del PID
unsigned long time1;

// Parámetros del PID
double Kp = 1; // Ganancia proporcional
double Ki = 2; // Ganancia integral
double Kd = 0.01; // Ganancia derivativa

// Inicializa el PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
//ultrasonico
const int trig = 9;
const int echo = 10;
long duration;
long distance;

//sensor de color
const int s0=41;
const int s1=42;
const int s2=43;
const int s3=45;
const int outTCS=46;

int dt=500;
int orientacion=0;
int m;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //motores
  pinMode(IN1_MT,OUTPUT);
  pinMode(IN2_MT,OUTPUT);
  pinMode(IN3_MT,OUTPUT);
  pinMode(IN4_MT,OUTPUT);
  pinMode(IN5_MT,OUTPUT);
  pinMode(IN6_MT,OUTPUT);
  pinMode(IN7_MT,OUTPUT);
  pinMode(IN8_MT,OUTPUT);
  pinMode(ENA_MT1,OUTPUT);
  pinMode(ENA_MT2,OUTPUT);
  pinMode(ENA_MT3,OUTPUT);
  pinMode(ENA_MT4,OUTPUT);
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT2);
  analogWrite(ENA_MT3,SPEED_MT);
  analogWrite(ENA_MT4,SPEED_MT2);
  //servo
  servo.attach(11);
  //encoder
  pinMode(encoder,INPUT);
  pinMode(encoder2,INPUT);
  attachInterrupt(1,interruption,RISING);//interrupcion para contar pulsos del encoder
  attachInterrupt(0,interruption2,RISING);
  //ultrasonico
  pinMode(trig, OUTPUT);  
  pinMode(echo, INPUT); 
  //sensor de color
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(outTCS,INPUT);
  digitalWrite(s0,1);
  digitalWrite(s1,0); 
    //PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  ahead();
  stop(1000);
}


//funcion contabilizadora de pulsos del encoder
void interruption() {
  cplus++;
  c++;
  Serial.println(c);
}
void interruption2() {
  cplus2++;
  c2++;
  Serial.println(c);
}
int corregir_giro(){
  int kerror=7;
  int corregir;
  loop_mpu();
  if(z_rotation<=-179.9 && z_rotation>-170){
    corregir=kerror*(z_rotation+orientacion);
  }
  else{
    corregir=kerror*(z_rotation-orientacion);
  }
  return corregir;
}
void ahead(){
  setahead();
  while (true) {
    // Mapeo de velocidad basado en el contador de pulsos
    int corregir=corregir_giro();
    SPEED_MT = map(c, 0, 30, 120-corregir,60);
    SPEED_MT2 = map(c2, 0, 30, 120+corregir,60);
    set_speed(); 
    //PID();
    if (c >= 38 ) {// Para detener el bucle cuando se alcanzan los 38 pulsos
      break;
    }
  }
  stop(0);
  c = 0;
  c2=0;
}
void back(){
  setback();
  while (true) {
    // Mapeo de velocidad basado en el contador de pulsos
    int corregir=corregir_giro();
    SPEED_MT = map(c, 0, 30, 120-corregir,60);
    SPEED_MT2 = map(c2, 0, 30, 120+corregir,60); 
    set_speed();
    //PID();
    if (c >= 38 ) {// Para detener el bucle cuando se alcanzan los 38 pulsos
      break;
    }
  }
  stop(0);
  c = 0;
  c2=0;
}
void right(){
  inicializarMPU6050();
  setright();
  while (true) {
    loop_mpu();
    // Mapeo de velocidad basado giroscopio
    if(orientacion==0){ 
      SPEED_MT = map(z_rotation, 0, (90-error_giro), 130,70);
      if (z_rotation >= 90-error_giro) {
        orientacion=90;
        break;
      }
    }
    else if(orientacion==90){
      SPEED_MT = map(z_rotation, 90, (180-error_giro), 130,70);
      if (z_rotation >= 180-error_giro) {
        orientacion=180;
        break; 
      }
    }   
    else if(orientacion==180){
      if(z_rotation>-50){
        SPEED_MT==130;
      }
      else{
      SPEED_MT = map(z_rotation, -180, (-90-error_giro), 130,70);
      }
      if (z_rotation >= -90-error_giro) {
        orientacion=-90;
        break; 
      }
    }  
    else if(orientacion==-90){
      SPEED_MT = map(z_rotation, -90,(0-error_giro) , 130,70);
      if (z_rotation >= 0-error_giro) {
        orientacion=0;
        break; 
      }
    }   
    SPEED_MT2 = SPEED_MT;
    set_speed();
    Serial.println(z_rotation);
  }
  stop(0);
  c = 0;
  c2=0;
}
void left(){
  inicializarMPU6050();
  setleft();
  while (true) {
    loop_mpu();
    // Mapeo de velocidad basado giroscopio
    if(orientacion==0){ 
      SPEED_MT = map(z_rotation, 0, (-90+error_giro), 130,70);
      if (z_rotation <= -90+error_giro) {
        orientacion=-90;
        break;
      }
    }
    else if(orientacion==-90){ 
      SPEED_MT = map(z_rotation, -90, (-180+error_giro), 130,70);
      if (z_rotation <= -180+error_giro) {
        orientacion=180;
        break;
      }
    } 
    else if(orientacion==180){ 
      if(z_rotation<50){
        SPEED_MT==130;
      }
      else{
        SPEED_MT = map(z_rotation, 180, (90+error_giro), 130,70);
      }
      if (z_rotation <= 90+error_giro) {
        orientacion=90;
        break;
      }
    }  
    else if(orientacion==90){ 
      SPEED_MT = map(z_rotation, 90, (0+error_giro), 130,70);
      if (z_rotation <= 0+error_giro) {
        orientacion=0;
        break;
      }
    }  
    SPEED_MT2 = SPEED_MT;
    set_speed();
    Serial.println(z_rotation);
  }
  stop(0);
  c = 0;
  c2=0;
}
void PID(){
//MOTORES DERECHA
  input = (cplus * 15); // Convertir pulsos a RPM 
  cplus = 0; // Reinicia el contador para la siguiente medición
  setpoint=SPEED_MT;
  myPID.Compute();
  analogWrite(ENA_MT1, output); // Establece la velocidad del motor
  analogWrite(ENA_MT2, output);
//MOTORES IZQUIERDA
  input = (cplus2 * 15); // Convertir pulsos a RPM 
  cplus2 = 0; // Reinicia el contador para la siguiente medición
  setpoint=SPEED_MT2;
  myPID.Compute();
  analogWrite(ENA_MT2, output); // Establece la velocidad del motor
  analogWrite(ENA_MT4, output);
  delay(100);
}

void setahead(){//avance adelnate, la variable x indica cuantas unidades desea que avance
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
  digitalWrite(IN5_MT,1);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,1);
  digitalWrite(IN8_MT,0);
}

void setback(){//retroceso
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
  digitalWrite(IN5_MT,0);
  digitalWrite(IN6_MT,1);
  digitalWrite(IN7_MT,0);
  digitalWrite(IN8_MT,1);
}
void stop(int x){
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);
  digitalWrite(IN5_MT,0);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,0);
  digitalWrite(IN8_MT,0);
  delay(x);
}
void setright(){//setear orientacion de motores
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);//GIRAR SOBRE EL EJE
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
  digitalWrite(IN5_MT,1);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,0);
  digitalWrite(IN8_MT,1);
}
void setleft(){//setear orientacion de motores
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
  digitalWrite(IN5_MT,0);
  digitalWrite(IN6_MT,1);
  digitalWrite(IN7_MT,1);
  digitalWrite(IN8_MT,0);
}
void set_speed(){
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT2);
  analogWrite(ENA_MT3,SPEED_MT);
  analogWrite(ENA_MT4,SPEED_MT2);
}
// void right(){//girar a la derecha
//   analogWrite(ENA_MT1,SPEED_MT-(SPEED_MT*0.2));
//   analogWrite(ENA_MT2,SPEED_MT2-(SPEED_MT2*0.2));
//   analogWrite(ENA_MT3,SPEED_MT-(SPEED_MT*0.2));
//   analogWrite(ENA_MT4,SPEED_MT2-(SPEED_MT2*0.2));
//   if(orientacion==0){
//     while(z_rotation<(90-error_giro)){
//       loop_mpu(); 
//       Serial.println(z_rotation); 
//       setright();
//     }
//     stop(0);
//     orientacion=90;
//   }
//   else if(orientacion==90){
//     while(z_rotation<=179.5-error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setright();
//     }
//     stop(0);
//     orientacion=180;    
//   }
//   else if(orientacion==180){
//     while(z_rotation>-90.3 || z_rotation<-89.8-error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setright();
//     }
//     stop(0);
//     orientacion=-90;    
//   }
//   else if(orientacion==-90){
//     while(z_rotation<0-error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setright();
//     }
//     stop(0);
//     orientacion=0;    
//   }  
//   analogWrite(ENA_MT1,SPEED_MT);
//   analogWrite(ENA_MT2,SPEED_MT2);
//   analogWrite(ENA_MT3,SPEED_MT);
//   analogWrite(ENA_MT4,SPEED_MT2);
// }
// void left(){//girar a la izquierda
//   analogWrite(ENA_MT1,SPEED_MT-(SPEED_MT*0.2));
//   analogWrite(ENA_MT2,SPEED_MT2-(SPEED_MT2*0.2));
//   analogWrite(ENA_MT3,SPEED_MT-(SPEED_MT*0.2));
//   analogWrite(ENA_MT4,SPEED_MT2-(SPEED_MT2*0.2));
//   if(orientacion==0){
//     while(z_rotation>-90+error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setleft();
//     }
//     stop(0);
//     orientacion=-90;
//   }
//   else if(orientacion==-90){
//     while(z_rotation>=-179.5+error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setleft();
//     }
//     stop(0);
//     orientacion=180;    
//   }
//   else if(orientacion==180){
//     while(z_rotation<89.7 || z_rotation>90.2+error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setleft();
//     }
//     stop(0);
//     orientacion=90;    
//   }
//   else if(orientacion==90){
//     while(z_rotation>0+error_giro){
//       loop_mpu();
//       Serial.println(z_rotation); 
//       setleft();
//     }
//     stop(0);
//     orientacion=0;    
//   }  
//   analogWrite(ENA_MT1,SPEED_MT);
//   analogWrite(ENA_MT2,SPEED_MT2);
//   analogWrite(ENA_MT3,SPEED_MT);
//   analogWrite(ENA_MT4,SPEED_MT2);
// }
int distancia(){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  // Calculamos la distancia (en cm)
  distance = (duration * 0.034) / 2; 
  // Serial.print(distance);
  delay(100);
  return distance;
}
int getcolor(){//devuelve el color
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
}

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
