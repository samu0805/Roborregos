#include <iostream>
using namespace std; 

int directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};


//solo adelante y girar izq
void adelante(){}
void girarIzq(){
    int tempx = directions[0][0];
    int tempy = directions[0][1];
    
    for (int i = 0; i< 3; i++){
        directions[i][0] = directions[i+1][0];
        directions[i][1] = directions[i+1][1];
    }
    directions[3][0] = tempx;
    directions[3][1] = tempy;
}
void retroceso(){}

bool paredAdelante(){
    //ver si hay una pared para entrar al centro
    return true;
}
bool cuadroNegro(){
    return true; 
}
int colorDet(){

    return 0;
}

// arriba, izquierda, abajo, derecha

void dfs(bool visited[5][3], int x, int y){

    colorDet();
    visited[x][y] = true;

    for (int i = 0; i<4; i++){
        int newX = x + directions[0][0];
        int newY = y + directions[0][1];
        if(paredAdelante() == false){
            bool negro = cuadroNegro();
            if(negro == false){
                if(visited[newX][newY] == false){
                    adelante();
                    dfs(visited, newX, newY);
                    retroceso();
                }
            }else{
                visited[newX][newY] = true; 
            }  
        }
        girarIzq();
    }
}

int main() {
    int count = 0;
    // arreglo para llevar el control de las celdas visitadas
    bool visited[5][3] = {{false}}; 
    // punto inicial
    int start_x = 0, start_y = 0;

    
    // iniciar DFS
    dfs(visited, start_x, start_y);
    
    // imprimir matriz con los valores de distancia
    return 0;
}

