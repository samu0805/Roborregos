//sensores y motores conectados al arduino 
//declarar los 2 motores (Señales PWM)
int ENA_PIN = 0;
int IN1_PIN = 1;
int IN2_PIN = 2;
int IN3_PIN = 3;
int IN4_PIN = 4;

int infra1 = 5;  
int infra2 = 6;




//otras variables
int countBot = 0;



void adelante(){
    //primer motor (izq)
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    //segundo motor (der)
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH); 
}
void girarDer(){
    //primer motor (der)
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);

    //segundo motor (izq)
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
}
void girarIzq(){
    //primer motor (der)
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    //segundo motor (izq))
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);   
}
void setup(){
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
}



void loop(){
    




}