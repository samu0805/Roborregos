void avanzar(){
    //avanzar una unidad
    //de primer punto rojo, al último
}
// algoritmo facil 

//Presentar junta:
/*
Approach 1: si ambos son negros continuar y si ambos son blancos continuar, linea recta.  y tarde o temprano regresará al otro lado con el nodo correspondiente
Approach 2: Si detecta dos negros, irse por la izquierda, y si no se mantiene la línea dar vuelta y buscar po el otro lado.
*/
void girarIzq(){}
void girarDer(){}
bool lineaDer(){}
bool lineaIzq(){}

int cantLin(){
    int cant = 0;
    //deteccion 1 infrarrojo
    // deteccion 2 infrarojo
    bool der = lineaDer();
    bool izq = lineaIzq();

    if (der == true && izq == false){
        //corregir izq
    }else if(izq == true && der == false){
        //corregir der
    }else if(izq == true && der == true){
        // girar derecha hasta emparejarse con la linea de la derecha
        search();
        
    }else{
        avanzar();
    }
}
void search(){

}

void zonaB(){
    bool fin = false; 
    while (fin == false){

    }

}

