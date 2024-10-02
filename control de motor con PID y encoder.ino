#include <PID_v1.h>//libreria PID
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
  codigo prueba
  ahead(1);
  stop(300);
  back(1);
  stop(300);  
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
}

void back(int x){//retroceso
  digitalWrite(IN1_MT,0);
  digitalWrite(IN2_MT,1);
  digitalWrite(IN3_MT,0);
  digitalWrite(IN4_MT,1);
  units(x);
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
