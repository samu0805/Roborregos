#include <iostream>
#include <vector>
using namespace std; 
/*
void DFS (vector<vector<int>> &adj){

    vector<bool> visited (adj.size())



}
void adelante(){

}
void girarIzq(){}
void girarDer(){}
void paredAdelante(){}

int lugares[3][5];
int posX = 0;
int posY = 0;

int count = 1;
int explored[15];
int frontier[15];

void DFS(int count){
    if (lugares[posX][posY] == 0){
        lugares[posX][posY] = count;
        int i = 0;
        while (4>i){
            if (paredAdelante == false){
                count+= 1;
                adelante();
                DFS(count);
                //retroceder
            }else{
                girarIzq();
            }
            i++;
        }
    } 
}
*/
//prueba -FUNCIONAL
#include <iostream>

using namespace std;

int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

void dfs(int maze[5][4], int x, int y, bool visited[5][4]) {
    // Verifica si está fuera de los límites, si es una pared (1) o si ya fue visitado
    if (x < 0 || y < 0 || x >= 5 || y >= 4 || maze[x][y] == 1 || visited[x][y]) {
        return;
    }
    
    // marcar como visitado
    visited[x][y] = true;
    
    // imprimir posición actual
    cout << "Visitando posición: (" << x << ", " << y << ")" << endl;

    // explorar las 4 direcciones
    for (int i = 0; i < 4; ++i) {
        int newX = x + directions[i][0];
        int newY = y + directions[i][1];
        dfs(maze, newX, newY, visited);
    }
}

int main() {
    // inicialización del laberinto usando un arreglo multidimensional
    int maze[5][4] = {
        {0, 0, 1, 0},
        {1, 0, 1, 0},
        {0, 0, 0, 0},
        {0, 1, 1, 1},
        {0, 0, 0, 0}
    };

    // arreglo para llevar el control de las celdas visitadas
    bool visited[5][4] = {{false}}; 

    int start_x = 0, start_y = 0;

    // iniciar DFS
    dfs(maze, start_x, start_y, visited);

    return 0;
}


