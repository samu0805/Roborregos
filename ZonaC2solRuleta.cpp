//Zona C !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void left(){}
void right(){}
void ahead(int x){}
void back(int x){}
bool paredAdelante(){return true;}
int getcolor(){return 1;}
//solo adelante y girar izq
void girar(int directions[4][2]){
    int tempx = directions[0][0];
    int tempy = directions[0][1];
    for (int i = 0; i< 3; i++){
        directions[i][0] = directions[i+1][0];
        directions[i][1] = directions[i+1][1];
    }
    directions[3][0] = tempx;
    directions[3][1] = tempy;
    left();
}
bool cuadroNegro(){
  //hacerlo con infrarrojos
  //SE PUEDE?? 
  ahead(0.3);
  int col= getcolor();
  back(0.3);
  if(col == 4){
    return true; 
  }else{
    return false;
  }
}
int colorDet(int colors[3]){
  int col = getcolor() -1;
  colors[col]++; 
  return 0;
}

// arriba, izquierda, abajo, derecha
void search(bool visited[5][3], int x, int y, int directions[4][2], int colors[3], int backstep[5][3], int& count, bool& pathFound){
    colorDet(colors);
    visited[x][y] = true;
    if(!pathFound){
        count++;
        backstep[x][y] = count;
    }
    if(x== 4 && y == 2){
       pathFound = true; 
    }
    for (int i = 0; i<4; i++){
        int newX = x + directions[0][0];
        int newY = y + directions[0][1];
        if((newX >= 0 || newY >=0 || newX<=5 || newY <= 3) && (paredAdelante() == false)){
            bool negro = cuadroNegro();
            if(negro == false){
                if(visited[newX][newY] == false){
                    ahead(1);
                    search(visited, newX, newY, directions, colors, backstep, count, pathFound);
                    back(1);
                }
            }else{
                visited[newX][newY] = true; 
            } 
        }             
        girar(directions);
    }
    if(!pathFound){
        backstep[x][y] = 30;
        count--;
    }
}
void fuga(int count, int x, int y, int directions[4][2], int backstep[5][3]){
    for(int i = 0; i < count-1; i++){
        for (int j = 0; j < 4; j++){
            int newX = x + directions[0][0];
            int newY = y + directions[0][1];
            if (backstep[newX][newY] == i+1){
                ahead(1);
                j=0;
            }else{
                girar(directions);
            }
        }
    }
    ahead(1);
}
void zonaB() {
    //adelante, izquierda, atras, derecha
    int directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
    
    bool visited[5][3] = {{false}}; 
    int backstep[5][3] = {{30}}; 
    int count = 0;
    // punto inicial
    int start_x = 0, start_y = 0;
    int colors[3] = {0};

    bool pathFound = false;
    search(visited, start_x, start_y, directions, colors, backstep, count, pathFound);

    fuga(count, start_x, start_y, directions, backstep);
}
/*#include <iostream>
using namespace std; 
//solo adelante y girar izq
void adelante(){}
void girarIzq(int directions[4][2]){
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

void search(bool visited[5][3], int x, int y, int directions[4][2]){

    colorDet();
    visited[x][y] = true;

    for (int i = 0; i<4; i++){
        int newX = x + directions[0][0];
        int newY = y + directions[0][1];
        if((newX >= 0 || newY >=0 || newX<=5 || newY <= 3) && (paredAdelante() == false)){
            bool negro = cuadroNegro();
            if(negro == false){
                if(visited[newX][newY] == false){
                    adelante();
                    search(visited, newX, newY, directions);
                    retroceso();
                }
            }else{
                visited[newX][newY] = true; 
            } 
        }             
        girarIzq(directions);
    }
}

int main() {
    //adelante, izquierda, atras, derecha
    int directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};

    int count = 0;
    // arreglo para llevar el control de las celdas visitadas
    bool visited[5][3] = {{false}}; 

    // punto inicial
    int start_x = 0, start_y = 0;

    
    // iniciar DFS
    search(visited, start_x, start_y, directions);
    
    // imprimir matriz con los valores de distancia
    return 0;
}
*/
