#include <iostream>
using namespace std; 

void adelante(){
    
}
void girarIzq(){
}
void girarDer(){

}
void retroceder(){
    girarIzq();
    girarIzq();
    adelante();
    girarIzq();
    girarIzq();

}

bool paredAdelante(){
    //ver si hay una pared para entrar al centro
    return true;
}
bool cuadroNegro(){
    return true; 
}


/*



//método para buscar y mapear todos 
//vectores de direccion: izquierda, derecha, abajo, arriba
int directions[4][2] = {{-2, 0}, {2, 0}, {0, -2}, {0, 2}};

// Parámetros: laberinto, posición x e y, arreglo de valores visitados
void dfs(int maze[11][7], int x, int y, bool visited[11][7]) {
    // Verifica si está fuera de los límites y si ya fue visitado
    if (x < 0 || y < 0 || x >= 11 || y >= 7 || visited[x][y]) {
        return;
    }
    
    // Marcar como visitado
    visited[x][y] = true;
    
    // Imprimir la posición actual
    cout << "Visitando posición: (" << x << ", " << y << ")" << endl;

    // Explorar las 4 direcciones
    for (int i = 0; i < 4; i++) {
        int newX = x + directions[i][0];
        int newY = y + directions[i][1];
        int wall = maze[x+ (directions[i][0])/2][y +(directions[i][1])/2];
        cout << wall;
        if (wall == 1){ 
            dfs(maze, newX, newY, visited);
        }
    }
}

int main() {
    // Inicialización del laberinto usando un arreglo multidimensional(0 -> casillas, 1-> puerta abierta, 2-> pared ahí)
    int maze[11][7] = {
        {1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 2, 1, 2, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 2, 1, 1, 1, 2, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 2, 1, 2, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 2, 1, 2, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1}
    };

    // Arreglo para llevar el control de las celdas visitadas
    bool visited[11][7] = {{false}}; 

    int start_x = 1, start_y = 1;

    // Iniciar DFS
    dfs(maze, start_x, start_y, visited);

    return 0;
}

*/

int directions[4][2] = {{-2, 0}, {2, 0}, {0, -2}, {0, 2}};
int path[4][2] = {{-1, 0}, {1,0}, {0, -1}, {0, 1}}; 

int maze[11][7] = {
        {1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1},
        {1, 0, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 1, 1}
    };
int backstep[5][3]={
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
    };

void dfs(int maze[11][7], int x, int y, bool visited[11][7]) {
    // Verifica si está fuera de los límites o si ya fue visitado
    if (x < 0 || y < 0 || x >= 11 || y >= 7 || visited[x][y]) {
        return;
    }
    
    // Marcar como visitado
    visited[x][y] = true;
    
    // Imprimir la posición actual
    cout << "Visitando posición: (" << x << ", " << y << ")" << endl;

    // Explorar las 4 direcciones
    for (int i = 0; i < 4; i++) {
        int newX = x + directions[i][0];
        int newY = y + directions[i][1];
        int newPx = x + path[i][0];
        int newPy = y + path[i][0];
        int wall = maze[x+ (directions[i][0])/2][y +(directions[i][1])/2];
        cout << wall;
        if (wall == 1){ 
            dfs(maze, newX, newY, visited);
        }
    }
}

int main() {

    // Arreglo para llevar el control de las celdas visitadas
    bool visited[11][7] = {{false}}; 

    int start_x = 1, start_y = 1;

    // Iniciar DFS
    dfs(maze, start_x, start_y, visited);

    return 0;
}

