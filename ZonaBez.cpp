#include <iostream>
using namespace std; 



int getLinea(int infraR){
    // if linea == true 
    return 1;
}
int getColor(int color){
    // if color == negro 
    return 1; 
}
//movimiento
void avanza(){}
//giros en porcentajes con PID giros bien chicos
void girarDer(){}
void girarIzq(){}
// 0 detectar negro, 1 detectar color
//cambiar color en el caso de que sea diferente número para blanco y negro
int infraI = 0;
int infraD = 0; 
// 5 igual a blanco, 4 a negro
int color  = 4;
void corrector(){
    // lectura de blacno y negro en infrrario y color 
    infraI = getLinea(infraI);
    infraD = getLinea(infraD);
    color = getColor(color);

    //básico
    if (infraI == 0 && color == 4 && infraD == 0){
        avanza();
    }//avanza V
    else if(infraI == 1 && color == 5 && infraD == 1){
        bool found = false;
        girarIzq();
        while (!found){ //sacar un rango de pasos para llegar al final
            girarIzq();
            infraI = getLinea(infraI);
            color = getColor(color);
            if(infraI == 0 && color == 4){
                break;
            }
        }
    }// Línea en la derecha
    else if(infraI == 0 && color == 4 && infraD == 1){
        while(infraD == 1){
            girarIzq();
            infraD = getLinea(infraD);
        }
    }// Línea en la izquierda PID
    else if(infraI == 1 && color == 4 && infraD == 0){
        while(infraI == 1){
            girarDer();
            infraI = getLinea(infraI);
        }
    } // Línea Punteada, Blanco los tres
    else if(infraI == 0 && color == 5 && infraD == 0){
        int count = 0;
        bool linea = false; 
        while (count < 100 && !linea){
            avanza();
            color = getColor(color);
            if(color == 5){
                linea = true; 
            }
            count++; 
        }
    }// peor de los casos
    else if (infraI == 1 && color == 4 && infraD == 1){
        // tipos de patrones
        // tipo 1: linea perpendicular con línea consecuente en el frente.
        // tipo 2: circulo o cuadrado con el centro despejado sin línea, recorrer el contorno
        
    }// Error, 001 and 100 
    else{
        cout << "error" ; 
    }
}


void search(int x, int y){ 

}

void loop(){
    corrector();
}


