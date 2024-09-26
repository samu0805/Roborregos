#include <iostream>

int places[3][5] = {{0,0,0,0,0}
                ,   {0,0,0,0,0}
                ,   {0,0,0,0,0}};
int posY = 1;
int posX = 0;

void retroceder(){
    girarIzq();
    girarIzq();
    adelante();
    girarIzq();
    girarIzq();

}
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

bool paredAdelante(){
    //ver si hay una pared para entrar al centro
    return true;
}
bool cuadroNegro(){

}


//método para buscar y mapear todos 
void search(){
    if(places[posY][posX] == 0){
        places[posY][posX] == 1;
        
        for(int i = 0; i< 4; i++){
            if(paredAdelante() == false){
                adelante();
                search();
                retroceder();
            }
            girarIzq();
        }
    }

 }

void recorrido(){


}

int main(){

}