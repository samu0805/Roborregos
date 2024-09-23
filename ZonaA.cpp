#include <iostream>

//int matriz[]  = ;
bool foundPelota = false; 
bool foundSalida = false;
int posX = 0;
int posY = 0;

// 1-> recorrido, 0-> no recorrido. Camino de regreso. Valor inicial == lugar[1][0]
int lugares[3][3] = {{0,0,0}, {0,0,0}, {0,0,0}};
int paredes[7][7] = {{0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}, {0,0,0,0,0,0,0}}; 

//método de referencia para determinar encontrada
void pelotaEncontrada(){
    //vuelta 180 y todo para atras
    // sensor ultrasonico, hasta que d = 2cm, while d > 2cm

}
//métodos de referencia para el movimiento
void adelante(){
    //inicializar motores
    /* 
    int t = 0; //- > segundos
    while (t < 30){ //(estimado, puede variar, medir práctica)
        if (lineaAbajo() == true){
            //poner motores en nulo
            t= 0;
            while( t< 10){ //(tiempo estimado en regresar al centro)
                //activar motoroes en sentido negativo (retroceder) <-
                t++; 
            }
            t= 30;
        }else{
            //activar motores en sentido positivo (avanzar) ->
            t++; 
        }
        
    }
    */
    
}
void girarIzq(){

}
void girarDer(){

}
bool lineaAbajo(){
    //avanzar exactamente la mita de la distancia y regresa y leer si hay línea
    //leer con el detector de color 
    
    return false; 

}
bool paredAdelante(){
    //ver si hay una pared para entrar al centro
    return true;
}



void busquedaPelota() {
    //{{0,0,0}, {0,0,0}, {0,0,0}}

    //algoritmo
    while (foundPelota == false){

        if (paredAdelante() == true){
            //revisar la dereha
            girarDer();
            adelante();
        }else{ 
            pelotaEncontrada();
            foundPelota = true;
        }
    }





}


int main()
{
    //primer paso
    adelante();
    posX = 1;
    lugares[posX][posY] = 1;
    //{{0,0,0}, {1,0,0}, {0,0,0}}
    
    busquedaPelota();
}
