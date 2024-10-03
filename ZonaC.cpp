#include <iostream>
using namespace std; 

void adelante(){
    
}
void girarIzq(){
    //cambiar los valores de directions
    int tempx = directions[0][0];
    int tempy = directions[0][1];

    for (int i = 0; i< 3; i++){
        directions[i][0] = directions[i+1][0];
        directions[i][1] = directions[i+1][1];
    }
    directions[3][0] = tempx;
    directions[3][1] = tempy;
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
int colorDet(){
    return 0;
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
// arriba, izquierda, abajo, derecha
int directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0} };

int backstep[5][3]={{0}};

void dfs(bool visited[5][3], int x, int y, int count){
    count = count +1;
    if (visited[x][y]){
        retroceder();
        return;
    }
    visited[x][y] = true;
    backstep[x][y] = count;

    for (int i = 0; i<4; i++){
        if(paredAdelante() == false){
            int newX = x + directions[0][0];
            int newY = y + directions[0][1];
            adelante();
            dfs(visited,newX, newY, count);
            retroceder();
            girarIzq();
        }else{
            girarIzq();
        }
    }
}


int main() {
    int count = 0;
    // Arreglo para llevar el control de las celdas visitadas
    bool visited[5][3] = {{false}}; 

    int start_x = 1, start_y = 1;

    
    // Iniciar DFS
    dfs(visited, start_x, start_y, count);
    
    for (int i = 0; i< 5; i++){
        for (int j= 0; j< 3; j++){
            cout << backstep[i][j];
        }
    }
    return 0;
}

