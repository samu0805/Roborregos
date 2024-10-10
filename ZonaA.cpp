#include <iostream>

void left(){}
void right(){}
void ahead(int x){}
void back(int x){}
bool paredAdelante(){ return true;}
int getcolor(){return 1;}
//Zona A
void pelotaEncontrada(){
    //vuelta 180 y todo para atras
    // sensor ultrasonico, hasta que d = 2cm, while d > 2cm
    left();
    left();
    back(1);
    /*
    while(distancia() > 1){
        back();
    }
    //cerrar servo
    */
}
bool lineaAbajo(){
    //avanzar exactamente la mitad de la distancia y regresa y leer si hay línea
    //leer con el detector de color 
    // NECESITO INFRARROJO 
    return false; 
}

//Zona A !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//int matriz[]  = ;
void ZonaA(){
  bool foundPelota = false; 
  bool foundSalida = false;
  bool lineaEncontrada = true; 
  int countD= 0;
  int countL=0;
  //busquda pelota
    while (foundPelota == false && lineaEncontrada == false){
        if(paredAdelante() == true){
            right();
            if(lineaAbajo() == false && lineaEncontrada == false){
                ahead(1);
                left();
            }else{
                lineaEncontrada = true;
                left();
            }
            if(lineaAbajo() == false && lineaEncontrada == false){
                ahead(1);
                left();
            }else{
                lineaEncontrada = true;
                left();
                ahead(1);
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
            ahead(1);
            right();
            ahead(1);
            right();
            countL++;
        }else{
            pelotaEncontrada();
        } 
    }
    //fuga 
    int pos = countD-countL;
    ahead(1);
    if(lineaEncontrada){
        if(abs(pos)== 2){
            ahead(1);
        }
        else if(pos == -3){
            left();
            ahead(1);
            left();
            ahead(1);
            right();
            ahead(1);
        }
        else if(pos == -1 || pos == 3){
            right();
            ahead(1);
            right();
            ahead(1);
            left();
            ahead(1); 
        }
        else{
            //pos == 0;
            right();
            ahead(1);
            right();
            ahead(1);
            ahead(1);
            right();
            ahead(1);
            left();
            ahead(1);
        }
    }else{
        //peor caso
        //pos == 1 -> true
        // or pos == 0
        int it = 0;
        while (lineaEncontrada == false && foundSalida == false && it<2){
            left();
            if(lineaAbajo() == false && lineaEncontrada == false){
                ahead(1);
                left();
            }else{
                lineaEncontrada = true;
                right();
            }
            if(lineaAbajo() == false && lineaEncontrada == false){
                ahead(1);
                left();
            }else{
                lineaEncontrada = true;
                left();
                ahead(1);
                left();
            }
            if(paredAdelante() == true){
                it++;   
            }else{
                ahead(1);
                if(getcolor() == 5){ // "rojo
                    foundSalida = true;
                }
            }
            right();
        }
        if (foundSalida == false){
          //tanto 0 como 1 pueden hacer esto
            right();
            ahead(1);
            right();
            ahead(1);
            ahead(1);
            right();
            ahead(1);
            left();
            if(paredAdelante() == true){
                right();
                ahead(1);
                right();
                ahead(1);
                left();
            }
            if(paredAdelante() == false){
                ahead(1);
            }
        }
    }
}

/*

//int matriz[]  = ;
bool foundPelota = false; 
bool foundSalida = false;
bool lineaEncontrada = true; 
int countD= 0;
int countL=0;
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
            countD++;
        }else{
            pelotaEncontrada();
            foundPelota = true;
        }
       
    }
    while (foundPelota == false){
        if(paredAdelante() == true){
            girarIzq();
            adelante();
            girarDer();
            adelante();
            girarDer();
            countL++;
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

        }
    }
}


void fuga(){
    int pos = countD-countL;
    adelante();
    if(lineaEncontrada){
        if(abs(pos)== 2){
            adelante();
        }
        else if(pos == -3){
            girarIzq();
            adelante();
            girarIzq();
            adelante();
            girarDer();
            adelante();
        }
        else if(pos == -1 || pos == 3){
            girarDer();
            adelante();
            girarDer();
            adelante();
            girarIzq();
            adelante();
        }
        else{
            //pos == 0;
            girarDer();
            adelante();
            girarDer();
            adelante();
            adelante();
            girarDer();
            adelante();
            girarIzq();
            adelante();
        }
    }else{
        //peor caso
        //pos == 1 -> true
        // or pos == 0
        int it = 0;
        while (lineaEncontrada == false && foundSalida == false && it<2){
            girarIzq();
            if(lineaAbajo() == false && lineaEncontrada == false){
                adelante();
                girarIzq();
            }else{
                lineaEncontrada = true;
                girarDer();
            }
            if(lineaAbajo() == false && lineaEncontrada == false){
                adelante();
                girarIzq();
            }else{
                lineaEncontrada = true;
                girarIzq();
                adelante();
                girarIzq();
            }
            
            if(paredAdelante() == true){
                i++;   
            }else{
                adelante();
                if(getcolor() == "rojo"){
                    foundSalida = true;
                }
            }
            girarDer();
       
        }
        if (foundSalida == false){
            //tanto 0 como 1 pueden hacer esto
            girarDer();
            adelante();
            girarDer();
            adelante();
            adelante();
            girarDer();
            adelante();
            girarIzq();

            if(paredAdelante() == true){
                girarDer();
                adelante();
                girarDer();
                adelante();
                girarIzq();
            }
            if(paredAdelante() == false){
                adelante();
            }
        
        }
    }
}

int main()
{
    //primer paso
    adelante();
    busquedaPelota();
    fuga();
    
    posX = 1;
    lugares[posX][posY] = 1;
    //{{0,0,0}, {1,0,0}, {0,0,0}}
    
    
}



//códigos karel prueba

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