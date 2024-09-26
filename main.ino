//sensores y motores conectados al arduino 
//declarar los 2 motores (Señales PWM)
const int ENA_PIN = 0;
const int IN1_PIN = 1;
const int IN2_PIN = 2;
const int IN3_PIN = 3;
const int IN4_PIN = 4;
//infrarrojo (digital - input)
const int infra1 = 5;  
const int infra2 = 6;
//ultrasonico (digital - input/output)
const int trigger = 7; 
const int echo = 8;
//led RGB
const int ledRGB = 9; 
//sensor de color(digital/analoga, input/output) 
const int s0 = 10;  
const int s1 = 11;  
const int s2 = 12;  
const int s3 = 13;  
const int out = 14;    


//botones (digital - input)
int botonA = 15;
int botonB = 16; 
int botonC = 17;


//otras variables
int countBot = 0;
bool end = false;


//funciones de movimiento
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

//funciones de detección y lógica 
bool paredAdelante(){
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    t = pulseIn(echo,HIGH);
    distancia = t/59;

    if(distancia < 10){
        return true;
    }else{
        return false;
    }

}
//detección de color
int redM(){
    digitalWrite(s2,LOW);
    digitalWrite(s3, LOW);

    int red = pulseIn(out, LOW);

    return red; 
}
int blueM(){
    digitalWrite(s2,LOW);
    digitalWrite(s3, HIGH);

    int blue = pulseIn(out, LOW);

    return blue; 
}
int greenM(){
    digitalWrite(s2,HIGH);
    digitalWrite(s3, HIGH);

    int green = pulseIn(out, LOW);

    return green; 
}
int lecturaColor(){

    int red = redM();
    delay(200);
    int blue = blueM();
    delay(200);
    int green = greenM();
    delay(200);

    // amarillo = 100.0% de rojo, 100.0% de verde y 0.0% de azul
    // rosa = 91.76% de rojo, 53.73% de verde y 60.39% de azul
    // morado = 34.12% de rojo, 13.73% de verde y 39.22% de azul

    //0 -> rojo, 1-> amrillo, 2 -> rosa, 3 -> morado, 4-> negro, 5-> error
    if (red > 250 && 10> blue && 10> green){
        return 0;
    }else if(red> 240 && green> 240 && 10> blue){
        return 1;    
    }else if (red > 250 && 140> green && green >120 && 150> blue && blue >130){
        return 2;
    }else if(90 > red && red > 60 && 30> green && green>10 && 90 > blue && blue> 60){
        return 3;
    }else if(10> red && 10> blue && 10> green){
        return 4;
    }else{
        return 5;
    }
    
}
void setup(){
    Serial.begin(9600);
    //motores
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    //infrarrojo
    pinMode(infra1, INPUT);
    pinMode(infra1, INPUT);


    // ultrasonico
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);

    //ledRGB
    pinMode()

    //sensor de color 
    pinMode(s0,OUTPUT);  
    pinMode(s1,OUTPUT);  
    pinMode(s2,OUTPUT);  
    pinMode(s3,OUTPUT);  
    pinMode(out,INPUT);   
    digitalWrite(s0,HIGH);  
    digitalWrite(s1,HIGH);  

    //botones
    pinMode(botonA, INPUT);
    pinMode(botonB, INPUT);
    pinMode(botonC, INPUT);

}
void loop(){
    int estadoA = digitalRead(botonA);
    int estadoB = digitalRead(botonB);
    int estadoC = digitalRead(botonC);

    if (estadoA == HIGH){
        while (end){

        }

    }else if(estadoB == HIGH){
        while (end){

        }
    }else if(estadoC == HIGH){
        while (end){

        }
    }else{
        //regresar un color con el rgb
    }

    




}