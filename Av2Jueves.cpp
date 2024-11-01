#include <Servo.h>
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
float angulo;
float inclinacion;
float ypr[3] = { 0, 0, 0 };
float xyz[3] = { 0, 0, 0 };
float anguloAnterior=0;
float tiempoAnterior=0;
int16_t *gyro;
//todos los pines nombrados con numero par son lado izquierdo  e impares derecho
//Motores
const int ENA_MT1 = 10;//atras derecha
const int ENA_MT2 = 5;//atras izq
const int ENA_MT3 = 6;//adelante deracha
const int ENA_MT4 = 7;//adelante izq
const int IN1_MT = 26;//atras der
const int IN2_MT = 27;
const int IN3_MT = 28;//atras izq
const int IN4_MT = 29;

const int IN5_MT = 30;//adelante deracha
const int IN6_MT = 31;
const int IN7_MT = 32;//adelante izq
const int IN8_MT = 33;
int SPEED_MT=190;//velocidad inicial
int SPEED_MT2=190;
//servo
Servo servo;
//encoders-variables de control
const int encoder=19; // der
const int encoder2=18;//izq
int c;//contador de pulsos der
int c2;//contador de pulsos izq
int cplus;//contador de pulsos der
int cplus2;//contador de pulsos izq
int frenado;//avance del motor en el frenado
int frenado2;
int error_giro=8;
///////////////////////////////////////
// Parámetros del PID
double Kp = 30; // Ganancia proporcional
double Ki = 15; // Ganancia integral
double Kd = 0.1; // Ganancia derivativa
double setpoint = 4; // Velocidad deseada
double setpoint2 = 4; // Velocidad deseada
double input1;
double input2;
double output1;
double output2; // Salidas del control

double integral1 = 0;
double integral2 = 0; // Integrales
double lastError1 = 0;
double lastError2 = 0; // Errores anteriores
///////////////////////////
//ultrasonico
const int trig = 2;
const int echo = 3;
long duration;
long distance;
//sensor de color
const int s0=34;
const int s1=35;
const int s2=36;
const int s3=37;
const int outTCS=38;
int colors[3] = {0};
//leds
const int purpleLed = 51;
const int pinkLed = 53;
const int yellowLed = 50;
const int blackLed = 52;
int purple, pink, yellow, black;
// infrarrojo
const int infrared1=24;//der
const int infrared2=25;//izq
//limit switch
int limitS1=48;
int limitS2=49;
bool lineaNegra=false;
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
  //encoder
  pinMode(encoder,INPUT);
  pinMode(encoder2,INPUT);
  attachInterrupt(4,interruption,RISING);//interrupcion para contar pulsos del encoder
  attachInterrupt(5,interruption2,RISING);
  //ultrasonico
  pinMode(trig, OUTPUT);  
  pinMode(echo, INPUT); 
  //sensor de color
  pinMode(purpleLed,OUTPUT);
  pinMode(pinkLed,OUTPUT);
  pinMode(yellowLed,OUTPUT);
  pinMode(blackLed,OUTPUT);
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT);
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  pinMode(outTCS,INPUT);
  digitalWrite(s0,1);
  digitalWrite(s1,0); 
  // calibrar_col();
  //MPU6050
  inicializarMPU6050();
  //servo
  servo.attach(11);
  servo.write(90);
  //INFRARROHOS
  pinMode(infrared1,INPUT);
  pinMode(infrared2,INPUT);
  //limitstich
  pinMode(limitS1,INPUT);
  pinMode(limitS2,INPUT);
}
void loop() {
  // calibrar_col();
  // rampa();
  zonaA();
  // ahead();

}
//funcion contabilizadora de pulsos del encoder
void interruption() {
  cplus++;
  c++;
  // Serial.println(c);
}
void interruption2() {
  cplus2++;
  c2++;
  // Serial.println(c2);
}
//MOVIMIENTO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
bool choque(){
  int limitState1=digitalRead(limitS1);           
  int limitState2=digitalRead(limitS2); 
  if(limitState1==1 && limitState2==1){
    setback();
    wait(250);
    stop(0);
    ahead();
    return true;
  }          
  if(limitState1==1){
    setBackleft();
    wait(250);
    stop(0);
    ahead();
    return true;
  }
  else if(limitState2==1){
    setBackRight();
    wait(250);
    ahead();
    return true;
  }
  return false;
}
void corregir_giro(){
  for(int i=1; i>0; i++){ 
    Serial.println("corrijiendo");
    SPEED_MT=60;
    SPEED_MT2=60; 
    set_speed();
    loop_mpu();
    getAngulo();
    if(orientacion!=0){ 
      if(angulo>orientacion){
        // SPEED_MT=100;
        // SPEED_MT2=190;   
        // set_speed(); 
        setleft();
      }
      else if(angulo<orientacion){
        // SPEED_MT=190;
        // SPEED_MT2=190;   
        // set_speed(); 
        setright();
      }
      if(angulo<orientacion+1 && angulo>orientacion-1){
        Serial.println("................");
        break;
      }
    }
    else{
      if(z_rotation>orientacion){
          // SPEED_MT=190;
          // SPEED_MT2=190;   
          // set_speed(); 
          setleft();
        }
      else if(z_rotation<orientacion){
        // SPEED_MT=190;
        // SPEED_MT2=190;   
        // set_speed(); 
        setright();
      }
      if(z_rotation<orientacion+1 && z_rotation>orientacion-1 && i>10){
        Serial.println(z_rotation);
        Serial.println("................");
        break;
      }
    }
  }
  stop(0);
  wait(700);
}
int corregir_avanza(){
  int kerror=20;
  int corregir;
  getAngulo();
  if(orientacion==0){
    corregir=kerror*z_rotation;
  }
  else{
    corregir=kerror*(angulo-orientacion);
  }
  return corregir;
}
void ahead(){
  c = 0;
  c2=0;
  for(int i=1; i>0; i++) {
    int pul=100;
    distancia();
    if(i>0){ 
      if(c >= pul || c2>=pul || distance<20){
        distancia();
        if(c >= pul || c2>=pul || distance<20){
          break;
        }
      }
    }
    setahead();
    // Mapeo de velocidad basado en el contador de pulsos
    int corregir=corregir_avanza();
    // SPEED_MT = map(c, 0, 35, 190+corregir,90);
    // SPEED_MT2 = map(c2, 0, 35, 190-corregir,90);
    SPEED_MT =190+corregir;
    SPEED_MT2 = 190-corregir;
    SPEED_MT = constrain(SPEED_MT, 120, 255);
    SPEED_MT2 = constrain(SPEED_MT2, 120, 255);
    set_speed(); 
    // PID();
    if(i>2){ 
       if(c >= pul || c2>=pul || distance<20){
        distancia();
        if(c >= pul || c2>=pul || distance<20){
          break;
        }
      }
    }
    bool impacto=choque();
    if(impacto==true){
      break;
    }
    // bool linea = lineaAbajo();
    // if(linea == true){
    //   lineaNegra = true;
    //   break;
    // }
  }
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);
  digitalWrite(IN5_MT,0);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,0);
  digitalWrite(IN8_MT,0);
  wait(400);
  SPEED_MT=120;
  SPEED_MT2=120;
  set_speed(); 
  c = 0;
  c2=0;
}
void wait(unsigned long interval) {
  static unsigned long previousMillis = 0; // Almacena el tiempo anterior

  unsigned long currentMillis = millis(); // Obtiene el tiempo actual

  // Verifica si ha pasado el intervalo
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Actualiza el tiempo anterior
    // Aquí no hacemos nada, solo actualizamos el tiempo
  }
}
void back(int pul){
  c = 0;
  c2=0;
  for(int i=1; i>0; i++) {   
    distancia();
    if(i>0){ 
      if(c >= pul || c2>=pul){
        distancia();
        if(c >= pul || c2>=pul){
          break;
        }
      }
    }
    setback();
    // Mapeo de velocidad basado en el contador de pulsos
    int corregir=corregir_avanza();
    // SPEED_MT = map(c, 0, 35, 190+corregir,90);
    // SPEED_MT2 = map(c2, 0, 35, 190-corregir,90);
    SPEED_MT =190-corregir;
    SPEED_MT2 = 190+corregir;
    SPEED_MT = constrain(SPEED_MT, 120, 255);
    SPEED_MT2 = constrain(SPEED_MT2, 120, 255);
    set_speed(); 
    // PID();
    if(i>2){ 
       if(c >= pul || c2>=pul ){
        distancia();
        if(c >= pul || c2>=pul ){
          break;
        }
      }
    }
  }
}

void rampa(){
  // ahead();
  // while(true){ 
  //   int corregir=corregir_avanza();
    SPEED_MT = 150;
    SPEED_MT2 = 150;
    set_speed();
  //   setahead();
  //   Serial.println("avance");
  //     // SPEED_MT = 170;
  //     // SPEED_MT2 = 170;
  //     // set_speed();
  //     setahead();
      int time=millis();
      float stateB;
      while(true){
        float stateA=stateB-inclinacion;
        stateB= inclinacion;
        loop_mpu();
        if(stateA>0.5){
          setahead();
          Serial.println("back");
        }
        else if(stateA<-0.5){
          setback();
          Serial.println("ahead");
        }
        else if(inclinacion==0){
          stop(0);
        }
        delay(50);
      }
      Serial.println("stop");
      stop(20000);
  // }
}
// --------------------------------------------------------
void right(){
  SPEED_MT2=100;
  SPEED_MT = SPEED_MT2;
  set_speed();
  if(orientacion==0){
    for(int i=1; i>0; i++){
      getAngulo();  
      Serial.println(angulo);
      // SPEED_MT = map(z_rotation, 0, (90-error_giro), 130,70);
      setright();
      if(z_rotation>90-error_giro){
        break;
      }
    }
    wait(100);
    orientacion=90;
    corregir_giro();
  }
  else if(orientacion==90){
    for(int i=1; i>0; i++){
      getAngulo();
      Serial.println(angulo);
      // SPEED_MT = map(z_rotation, 90, (180-error_giro), 130,70);
      setright();
      if(angulo>180-error_giro){
        break;
      }
    }
    wait(100);
    orientacion=180; 
    corregir_giro();   
  }
  else if(orientacion==180){
    for(int i=1; i>0; i++){
      getAngulo();
      Serial.println(z_rotation);
      // SPEED_MT = map(z_rotation, -180, (-90-error_giro), 130,70);
      setright();
      if(angulo>270-error_giro){
        break;
      }
    }
    wait(100);
    orientacion=270;   
    corregir_giro(); 
  }
  else if(orientacion==270){
    for(int i=1; i>0; i++){
      getAngulo();
      Serial.println(z_rotation);
      // SPEED_MT = map(z_rotation, -90,(0-error_giro) , 130,70);
      // SPEED_MT2 = SPEED_MT;
      // set_speed();
      setright();
      if(angulo>360-error_giro){
        break;
      }
    }
    wait(100);
    orientacion=0;
    corregir_giro();   
  }
  wait(400);
  c=0;
  c2=0; 
}   
void left(){
  SPEED_MT2=100;
  SPEED_MT = SPEED_MT2;
  set_speed();
  if(orientacion==0){
    for(int i=1; i>0; i++){
      getAngulo();  
      Serial.println(angulo);
      // SPEED_MT = map(z_rotation, 0, (90-error_giro), 130,70);
      setleft();
      if(z_rotation<-90+error_giro){
        break;
      }
    }
    wait(100);
    orientacion=270;
    corregir_giro();
  }
  else if(orientacion==270){
    for(int i=1; i>0; i++){
      getAngulo();
      Serial.println(angulo);
      // SPEED_MT = map(z_rotation, 90, (180-error_giro), 130,70);
      setleft();
      if(angulo<180+error_giro){
        break;
      }
    }
    wait(100);
    orientacion=180; 
    corregir_giro();   
  }
  else if(orientacion==180){
    for(int i=1; i>0; i++){
      getAngulo();
      Serial.println(z_rotation);
      // SPEED_MT = map(z_rotation, -180, (-90-error_giro), 130,70);
      setleft();
      if(angulo<90+error_giro){
        break;
      }
    }
    wait(100);
    orientacion=90;   
    corregir_giro(); 
  }
  else if(orientacion==90){
    for(int i=1; i>0; i++){
      getAngulo();
      Serial.println(z_rotation);
      // SPEED_MT = map(z_rotation, -90,(0-error_giro) , 130,70);
      // SPEED_MT2 = SPEED_MT;
      // set_speed();
      setleft();
      if(angulo<0+error_giro){
        break;
      }
    }
    wait(100);
    orientacion=0;
    corregir_giro();   
  }
  wait(400);
  c=0;
  c2=0; 
}
void PID(){
  input1 = cplus; 
  input2 = cplus2;

  //  Motor 1-3
  double error1 = (setpoint/20) - input1;
  integral1 += error1;
  double derivative1 = error1 - lastError1;
  output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;
  lastError1 = error1;

  // Motor 2-4
  double error2 = (setpoint2/20) - input2;
  integral2 += error2;
  double derivative2 = error2 - lastError2;
  output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;
  lastError2 = error2;

  // Limitar salida
  output1 = constrain(output1, 0, 255);
  output2 = constrain(output2, 0, 255);

  analogWrite(ENA_MT1,output1);
  analogWrite(ENA_MT2,output2);
  analogWrite(ENA_MT3,output1);
  analogWrite(ENA_MT4,output2);
  // Reiniciar contadores
  cplus = 0;
  cplus2 = 0;
  delay(50); 
}
void setahead(){//avance adelnate, la variable x indica cuantas unidades desea que avance
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
  digitalWrite(IN5_MT,0);
  digitalWrite(IN6_MT,1);
  digitalWrite(IN7_MT,0);
  digitalWrite(IN8_MT,1);
}
void setback(){//retroceso
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
  digitalWrite(IN5_MT,1);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,1);
  digitalWrite(IN8_MT,0);
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
void setBackRight(){
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,1);
  digitalWrite(IN4_MT,0);
  digitalWrite(IN5_MT,1);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,0);
  digitalWrite(IN8_MT,0);
}
void setBackleft(){
  digitalWrite(IN1_MT,1);
  digitalWrite(IN2_MT,0);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,0);
  digitalWrite(IN5_MT,0);
  digitalWrite(IN6_MT,0);
  digitalWrite(IN7_MT,1);
  digitalWrite(IN8_MT,0);
}
//////////////7funciones de MPU6050/////////////////////
void loop_mpu(){
  digitalWrite(8, !digitalRead(8));
  digitalWrite(9, !digitalRead(9));
  mpu.dmp_read_fifo(0);
  z_rotation=xyz[0];
  inclinacion = xyz[1];
}
void getAngulo(){
  loop_mpu();
  angulo = 0; 
  if(z_rotation <0 ){
    angulo = 180 +  (180+z_rotation);
  }
  else{
    angulo = z_rotation;
  }
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






// ALGORITMOS (LÓGICA) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// bool lineaAbajo(){
//   int state_infra1=digitalRead(infrared1);
//   int state_infra2=digitalRead(infrared2);
//   if( state_infra1==1|| state_infra2 == 1){
//     lineaNegra=true;
//     return true;
//   }
//   else{
//     return 0;
//   }
// }
void distancia(){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  // Calculamos la distancia (en cm)
  distance = (duration * 0.034) / 2; 
  Serial.println(distance);
  delay(100);
  //return distance;
}
bool paredAdelante(){
  distancia();
  if(distance < 15){
    return true;
  }
  distancia();
  if(distance < 15){
    return true;
  }
  return false; 
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
  int color=green+red+blue;
  Serial.println(color);
  if(color<pink-(pink-yellow)/2 ){
    // Serial.println("Color AMARILLO");
    return 3;
  }
  else if(color>purple+(black-purple)/2){
    // Serial.println("Color NEGRO");
    return 4;
  }
  else{
    if(color>pink+(purple-pink)/2){ 
      return 1;
    }
    else if(color<purple-(purple-pink)/2){

      return 2;
    }
  }
// else if(green<100 && blue<100 && red<100 && (green-red)<20 && (green-blue)<20){
//   Serial.println("Color Blanco");
// }

// else{
//     Serial.println("Color no encontrado");
// }
}
void calibrar_col(){
  for(int i=1;i<=4;i++){ 
    if(i==1){ 
      Serial.println("morado");
      analogWrite(purpleLed,125);
      delay(3000);
      purple=setColors();
      digitalWrite(purpleLed,0);
      Serial.println(purple);
    }
    else if(i==2){ 
      analogWrite(pinkLed,125);
      delay(3000);
      pink=setColors();
      digitalWrite(pinkLed,0);
      Serial.println(pink);
    }
    else if(i==3){ 
      analogWrite(yellowLed,125);
      delay(3000);
      yellow=setColors();
      digitalWrite(yellowLed,0);
      Serial.println(yellow);
    }
    else if(i==4){ 
      Serial.println("negro");
      analogWrite(blackLed,125);
      delay(3000);
      black=setColors();
      digitalWrite(blackLed,0);
      Serial.println(black);
    }
  }
}
int setColors(){//devuelve valores RGB
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
  return (red+green+blue);
}

// ZONA C !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void girar(int directions[4][2]){
  int tempx = directions[0][0];
  int tempy = directions[0][1];
  for (int i = 0; i< 3; i++){
    directions[i][0] = directions[i+1][0];
    directions[i][1] = directions[i+1][1];
  }
  directions[3][0] = tempx;
  directions[3][1] = tempy;
  right();
  c=0;
  c2=0;
}
void colorDet(){
  int col = getcolor();
  if(col == 1){
    analogWrite(purpleLed,125);
    delay(1000);
    digitalWrite(purpleLed,0);
  }
  else if(col == 2){
    analogWrite(pinkLed,125);
    delay(1000);
    digitalWrite(pinkLed,0);
  }
  else if(col == 3){
    analogWrite(yellowLed,125);
    delay(1000);
    digitalWrite(yellowLed,0);
  }
  colors[col-1]++;
}
//arriba, izquierda, abajo, derecha
void search(bool visited[3][5], int x, int y, int directions[4][2], int backstep[3][5], int& cnt, bool& pathFound){
  // colorDet();
  // visited[x][y] = true;
  // if(pathFound == false){
  //     cnt++;
  //     backstep[x][y] = cnt;
  // }
  if(x == 3 && y == 5){
    pathFound = true; 
  }
  for (int i = 0; i<4; i++){
    int newX = x + directions[0][0];
    int newY = y + directions[0][1];
    // Serial.println(newY);
    // delay(100);
    if((newX >= 0 && newY >= 0 && newX<3 && newY <5) && (paredAdelante() == false)){
      if(visited[newX][newY] == false){
        ahead();
        c=0;
        c2=0;
        if(lineaNegra == false){
          search(visited, newX, newY, directions, backstep, cnt, pathFound);
          back(120);
        }
        else{
          visited[newX][newY] = true;
          analogWrite(blackLed,125);
          delay(1000);
          digitalWrite(blackLed,0);
        }
      }
    }
    girar(directions);
  }
  if(pathFound == false){
    backstep[x][y] = 30;
    cnt--;
  }
}
void fuga(int cnt, int x, int y, int Mcolor, int directions[4][2], int backstep[3][5]){
  bool foundColor = false;
  for(int i = 1; i < cnt; i++){
    if(foundColor == false){
      int col = getcolor();
      if (col== Mcolor){
        Mcolor = 30;
        foundColor = true;
        colorDet();
      }
    }
    for (int j = 0; j < 4; j++){
      int newX = x + directions[0][0];
      int newY = y + directions[0][1];
      if (newX >= 0 && newY >= 0 && newX < 3 && newY <5 && backstep[newX][newY] == i+1){
        c = 0;
        c2=0;
        ahead();
        j=4;
        x = newX;
        y = newY;
      }else{
        girar(directions);
      }
    }
  }
  //salir
  for(int i = 0; i< 4; i++){
    int newX = x + directions[0][0];
    int newY = y + directions[0][1];
    if(newX == 2 && newY == 5){
      ahead();
    }
  }
}
void zonaC() {
  ahead();
  //adelante, derecha, atras, izquierda
  int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
  bool visited[3][5] = {{false}}; 
  int backstep[3][5] = {{30}};
  int cnt = 0;
  bool pathFound = false;
  int Mcolor = 0;
  int start_x = 1, start_y = 0;
  search(visited, start_x, start_y, directions, backstep, cnt, pathFound);
  for(int i = 0; i<2; i++){
    if(colors[i]< colors[i+1]){
      Mcolor = i+1;
    }
  }
  fuga(cnt, start_x, start_y, Mcolor, directions, backstep);
}


//Zona A !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void pelotaEncontrada(){
    right();
    right();
    corregir_giro();
    c=0;
    c2=0;
    back(120);
    stop(0);
    wait(1000);
    servo.write(0);
}
void zonaA(){
    bool foundPelota = false; 
    bool foundSalida = false;
    int countD= 0;
    int countL=0;
    //entrada Laberinto 
    ahead();
    //busquda pelota
    while (foundPelota == false && lineaNegra == false){
        if(paredAdelante() == true){
            right();
            ahead();
            if(lineaNegra == false){
                left();
            }else{
                lineaNegra = true;
                left();
            }
            ahead();
            if(lineaNegra == false){
                left();
            }else{
                lineaNegra = true;
                left();
                ahead();
                right();
            }
            countD++;
        }else{
            pelotaEncontrada();
            foundPelota = true;
        }
    }
    while (foundPelota == false){
        if(paredAdelante() == true){
            left();
            ahead();
            right();
            ahead();
            right();
            countL++;
        }else{
            pelotaEncontrada();
        } 
    }
    //fuga 
    int pos = countD-countL;
    ahead();
    if(lineaNegra == true){
        if(abs(pos) == 2){
            ahead();
        }
        else if(pos == -3){
            left();
            ahead();
            left();
            ahead();
            right();
            ahead();
        }
        else if(pos == -1 || pos == 3){
            right();
            ahead();
            right();
            ahead();
            left();
            ahead(); 
        }
        else{
            //pos == 0;
            right();
            ahead();
            right();
            ahead();
            ahead();
            right();
            ahead();
            left();
            ahead();
        }
    }else{
        //peor caso
        //pos == 1 -> true
        // or pos == 0
        int it = 0;
        while (lineaNegra == false && foundSalida == false && it<2){
            left();
            ahead();
            if(lineaNegra == false){
                left();
                ahead();
                if(lineaNegra == false){
                    right();
                }else{
                    lineaNegra = true;
                    left();
                    ahead();
                    left();
                }
                if(paredAdelante() == true && lineaNegra == false){
                    it++;   
                }else{
                    ahead();
                    foundSalida= true;
                }
                right();
            }else{
                lineaNegra = true;
                right();
            }
           
        }
        if (foundSalida == false){
            //tanto 0 como 1 pueden hacer esto
            right();
            ahead();
            right();
            ahead();
            ahead();
            right();
            ahead();
            left();
            if(paredAdelante() == true){
                right();
                ahead();
                right();
                ahead();
                left();
            }
            if(paredAdelante() == false){
                ahead();
            }
        }
    }
}

// ZONA B !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 0 detectar negro, 1 detectar color
//cambiar color en el caso de que sea diferente número para blanco y negro
// 5 igual a blanco, 4 a negro
void path(){
    int infra1=digitalRead(infrared1);
    int infra2=digitalRead(infrared2);
    int color = getcolor();
    Serial.print(infra2);
    Serial.print(",");
    Serial.print(color);
    Serial.print(",");
    Serial.println(infra1);
     //básico
    if (infra1 == 0 && color == 4 && infra2 == 0){
        ahead();
    }//avanza V
    else if(infra1 == 1 && color != 4 && infra2 == 1){
        bool found = false;
        setleft();
        while (!found){ //sacar un rango de pasos para llegar al final
            setleft();
            infra1=digitalRead(infrared1);
            color = getcolor();
            if(infra1 == 0 && color == 4){
                break;
            }
        }
    }// Línea en la derecha
    else if(infra1 == 0 && color == 4 && infra2 == 1){
        while(infra2 == 1){
            setleft();
            infra2=digitalRead(infrared2);
        }
    }// Línea en la izquierda PID
    else if(infra1 == 1 && color == 4 && infra2 == 0){
        while(infra1 == 1){
            setright();
            infra1=digitalRead(infrared1);
        }
    } // Línea Punteada, Blanco los tres
    else if(infra1 == 0 && color != 4 && infra2 == 0){
        int count = 0;
        bool linea = false; 
        while (count < 100 && linea == false){
            setahead();
            color = getcolor();
            if(color == 4){
                linea = true; 
            }
            count++; 
        }
    }// peor de los casos
    else if (infra1 == 1 && color == 4 && infra2 == 1){
        // tipos de patrones
        // tipo 1: linea perpendicular con línea consecuente en el frente.
        for(int i = 0; i < 100; i++){
          setahead();
        }
        color = getcolor();
        if(color != 4){

        }
        
        // tipo 2: circulo o cuadrado con el centro despejado sin línea, recorrer el contorno
        
    }// Error, 001 and 100 
    else if (infra1 == 0 && color != 4 && infra2 == 1){
        setright();
    }
    else if (infra1 == 1 && color != 4 && infra2 == 0){
        setleft();
    }
    // else{
    //     cout << "error";
    // }
}
void zonaB(){
    SPEED_MT = 90;
    SPEED_MT2 = SPEED_MT;
    set_speed();
    while(paredAdelante() == false){
        path();
    }
}
