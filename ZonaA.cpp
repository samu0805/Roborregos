#include <iostream>

//int matriz[]  = ;
bool foundPelota = false; 
bool foundSalida = false;
bool lineaEncontrada = true; 
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
    //avanzar exactamente la mitad de la distancia y regresa y leer si hay línea
    //leer con el detector de color 
    
    return false; 

}
bool paredAdelante(){
    //ver si hay una pared para entrar al centro
    return true;
}

void busquedaPelota() {
    //{{0,0,0}, {0,0,0}, {0,0,0}}


    // camino derecha
    while (foundPelota == false && lineaEncontrada == false){
        if(paredAdelante() == true){
            girarDer();

            if(lineaAbajo() == false && lineaEncontrada == false){
                adelante();
                girarIzq();
            }else{
                lineaEncontrada = true;
                girarIzq();
            }
            if(lineaAbajo() == false && lineaEncontrada == false){
                adelante();
                girarIzq();
            }else{
                lineaEncontrada = true;
                girarIzq();
                adelante();
                girarDer();
            }
        }else{
            pelotaEncontrada();
        }
    }
    while (foundPelota == false){
        if(paredAdelante() == true){
            girarIzq();
            adelante();
            girarDer();
            adelante();
            girarDer();
        }else{
            pelotaEncontrada();
        }
    }
}
void fuga(){
    girarIzq();

    while (paredAdelante() ){ // Detectar punto final
        girarDer();
        if (paredAdelante() == false){
            adelante();
            while (paredAdelante() == false){
                girarIzq();
                adelante();
                
            }
            girarIzq();

        }else{

        }
    
    }
}

int main()
{
    //primer paso
    adelante();
    busquedaPelota();
    fuga();
    /*
    posX = 1;
    lugares[posX][posY] = 1;
    //{{0,0,0}, {1,0,0}, {0,0,0}}
    */
    
}



//códigos karel prueba
/*
NUMERO 1: VARIABLES Y LETRAS MAL 
void search()
{
	//comentario
    if(!nextToABeeper)
    {
        putbeeper();
        iterate(4)
        {
            if(frontIsClear)
            {
                move();
                nfl();
                turnleft();
                turnleft();
                move();
                turnleft();
                turnleft();
            }
                turnleft();
       }
    }
    //
    int foundBall = 0;
    int linea = 0;
    while (foundBall == 0 && linea == 0){
    	if(!fronIsClear){
            girarDer();

            if(frontIsClear){
                adelante();
                turnleft();
            }else{
                linea = 1;
                girarIzq();
            }
            if(frontIsClear && linea == 0){
                adelante();
                girarIzq();
            }else if(linea == 0){
                lineaEncontrada = 1;
                girarIzq();
                adelante();
                girarDer();
            }
        }else{
        	foundBall = 1; 
            move();
            pickbeeper();
        }
    
    }
    
}

NUMERO 2: FUNCIONAL

class program {

void girarDer(){
	turnleft();
    turnleft();
    turnleft();
}
void search()
{
    while (!frontIsClear && noBeepersInBeeperBag){
            girarDer();

            if(frontIsClear){
                move();
                turnleft();
                if(frontIsClear){
                    move();
                    turnleft();
                }else{
                    pickbeeper();
                    turnleft();
                    move();
                    girarDer();
                }
            }else{
                pickbeeper();
                turnleft();
            }
            
    }
    while (!frontIsClear){
            turnleft();
            move();
            girarDer();
            move();
            girarDer();
    }
    if (frontIsClear){
        move();
        pickbeeper();
    } 
}
void escape(){
    while (leftIsBlocked && notNextToABeeper){
        if (frontIsClear){
            move();
            while (leftIsClear){
                turnleft();
                move();
            }
        }else{
            girarDer();
        }
    
    }
    
    
    
}
program() {
	move();
    search();
    escape();
    turnoff();
}

}
*/