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
int z_rotation;//angulo de rotacion
float ypr[3] = { 0, 0, 0 };
float xyz[3] = { 0, 0, 0 };
//Motores
const int ENA_MT1 = 50;
const int ENA_MT2 = 51;
const int IN1_MT = 52;
const int IN2_MT = 53;
const int IN3_MT = 48;
const int IN4_MT = 49;
int SPEED_MT=100;//velocidad inicial
int SPEED_MT2=100;
//encoders-variables de control
const int encoder=3;
const int encoder2=2;//por definir
int state;
int state2;
int c;
int c2;
int last;
int last2;
int frenado;//avance del motor en el frenado
int frenado2;
int error_giro;
// Variables para PID
double setpoint; // Valor deseado de velocidad
double input;    // Valor actual de velocidad
double output;   // Salida del PID
unsigned long time1;
int output1;

// Parámetros del PID
double Kp = 1.0; // Ganancia proporcional
double Ki = 2.0; // Ganancia integral
double Kd = 0.001; // Ganancia derivativa

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
  pinMode(ENA_MT1,OUTPUT);
  pinMode(ENA_MT2,OUTPUT);
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT2);
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
    //30cm en 1.5s=58RPM
    //en 2s=43RPM
  setpoint = 90; //RPM deseadas
  myPID.SetMode(AUTOMATIC);
  calibracion_MT();  //calibracion de velocidad con PID
}

void loop() {
  //codigo prueba
  
  loop_mpu();

  // ahead(1);
  // stop(1000);
  // ahead(1);
  // stop(1000);
  // ahead(1);
  // stop(1000);
  // m=getcolor();
  // Serial.println(m);
  // distancia();
  // Serial.println(distance);
  // delay(100);
  //}
  right();
  stop(1000);
  // Serial.println(z_rotation);
  // back(1);
  // stop(300);
  // left();
  // stop(20000000); 
}
//CALIBRACION DE VELOCIDAD CON PID
void calibracion_MT(){
  time1=millis();
  while((millis()-time1)<4000){
    input = (c * 15); // Convertir pulsos a RPM 
    c = 0; // Reinicia el contador para la siguiente medición

    // Calcula el PID
    myPID.Compute();

    // Controla el motor1
    analogWrite(ENA_MT1, output); // Establece la velocidad del motor
    digitalWrite(IN1_MT, HIGH); // Establece la dirección
    digitalWrite(IN2_MT, LOW);
    output1=output;

    input = (c2 * 15); // Convertir pulsos a RPM 
    c2 = 0; // Reinicia el contador para la siguiente medición

    // Calcula el PID
    myPID.Compute();
    // Controla el motor2
    analogWrite(ENA_MT2, output); // Establece la velocidad del motor
    digitalWrite(IN3_MT, HIGH); // Establece la dirección
    digitalWrite(IN4_MT, LOW);

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
  c2=0;
  while(c<=40){
    Serial.println(c);
  }
  stop(500);
  frenado=c-40;
  frenado2=c2-40;
  SPEED_MT=output1;//establecer velocidad ideal
  SPEED_MT2=output;
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT2);

  inicializarMPU6050();//MPU6050 Inicializacion
  analogWrite(ENA_MT1,SPEED_MT-30);
  analogWrite(ENA_MT2,SPEED_MT2-30);
  if(orientacion==0){
    while(z_rotation<90){
      loop_mpu(); 
      Serial.println(z_rotation); 
      setright();
    }
    stop(0);
    loop_mpu();
    error_giro=z_rotation-90;
    orientacion=90;
  }
}
//funcion contabilizadora de pulsos del encoder
void interruption(){
  state=digitalRead(encoder);
  if(state!=last){
    c++;
    last=state;
    //Serial.println(c);
  }
}

void interruption2(){
  state2=digitalRead(encoder2);
  if(state2!=last2){
    c2++;
    last2=state2;
    //Serial.println(c2);
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
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);//GIRAR SOBRE EL EJE
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
}
void setleft(){//setear orientacion de motores
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
}
void right(){//girar a la derecha
  analogWrite(ENA_MT1,SPEED_MT-30);
  analogWrite(ENA_MT2,SPEED_MT2-30);
  if(orientacion==0){
    while(z_rotation<90-error_giro-error_giro){
      loop_mpu(); 
      Serial.println(z_rotation); 
      setright();
    }
    stop(0);
    orientacion=90;
  }
  else if(orientacion==90){
    while(z_rotation<=179.9-error_giro){
      loop_mpu();
      Serial.println(z_rotation); 
      setright();
    }
    stop(0);
    orientacion=180;    
  }
  else if(orientacion==180){
    while(z_rotation>-90.1 || z_rotation<-89.9+error_giro){
      loop_mpu();
      Serial.println(z_rotation); 
      setright();
    }
    stop(0);
    orientacion=-90;    
  }
  else if(orientacion==-90){
    while(z_rotation<0-error_giro){
      loop_mpu();
      Serial.println(z_rotation); 
      setright();
    }
    stop(0);
    orientacion=180;    
  }  
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT2);
}
void left(){//girar a la izquierda
  analogWrite(ENA_MT1,SPEED_MT-30);
  analogWrite(ENA_MT2,SPEED_MT2-30);
  if(orientacion==0){
    while(z_rotation>-90+error_giro){
      loop_mpu();
      setleft();
    }
    stop(0);
    orientacion=-90;
  }
  else if(orientacion==-90){
    while(z_rotation>=-179.9+error_giro){
      loop_mpu();
      setleft();
    }
    stop(0);
    orientacion=180;    
  }
  else if(orientacion==180){
    while(z_rotation>90+error_giro){
      loop_mpu();
      setleft();
    }
    stop(0);
    orientacion=90;    
  }
  else if(orientacion==90){
    while(z_rotation>0+error_giro){
      loop_mpu();
      setright();
    }
    stop(0);
    orientacion=90;    
  }  
  analogWrite(ENA_MT1,SPEED_MT);
  analogWrite(ENA_MT2,SPEED_MT2);
}
void distancia(){
    digitalWrite(trig, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  duration = pulseIn(echo, HIGH);
  
  // Calculamos la distancia (en cm)
  distance = (duration * 0.034) / 2; 
  
  // Serial.print("Distancia: ");
  // Serial.print(distance);
  // Serial.println(" cm");
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
