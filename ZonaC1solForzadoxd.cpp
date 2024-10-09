#include <iostream>
#include <stack>
using namespace std; 

void adelante(){}
void girarIzq(){}
void girarDer(){}

void move(int op){
    if (op == 0){
        adelante();
        //poner motores para adelante
    }else if(op == 1){
        girarIzq();
        girarIzq();
        adelante();
        girarIzq();
        girarIzq();
        //irse hacia atrás
    }
}
void girar(int op){
    //cambiar los valores de directions
    if (op == 0){
        girarIzq();
        adelante();
        girarDer();
        //poicionarse viendo hacia el norte
    }else if(op == 1){
        girarDer();
        adelante();
        girarIzq();
        //poicionarse viendo hacia el norte       
    }
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

/* EJEMPLO CLASE:
void recursiveFunction(int n){
    if (n == 0){
        return; 
    }

    cout << "funcion regresiva" << n;
    recursiveFunction(n-1);
    cout << "regresando" << n;
}
int fibRecursivo(int n){
    if(n == 0){
        return 0;
    }
    if(n== 1){
        return 1;
    }
    return fibRecursivo(n-1) + fibRecursiv(n-2)
}
*/


// arriba, izquierda, abajo, derecha
int directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
// 1 adelante, 2 girar izquierda, 3 atras, 4 girar derecha 
stack<int> st; 

int backstep[5][3]={{20}};

void dfs(bool visited[5][3], int x, int y){

    colorDet();
    visited[x][y] = true;

    for (int i = 0; i<4; i++){
        bool negro = cuadroNegro();
        if(paredAdelante() == false &&  negro == false){
            int newX = x + directions[i][0];
            int newY = y + directions[i][1];
            if (cuadroNegro() == true){
                visited[newX][newY] = true;
            }else{
                if(visited[newX][newY] == false){
                    //regresar al norte
                    for(int j = i; j>0; j--){
                        girarDer();
                    }
                    //dependiendo de cual ventana este abierta. 
                    switch(i) {
                        case 0:
                            move(0); // adelante
                            break;
                        case 1:
                            girar(0); // girar izquierda
                            break;
                        case 2:
                            move(1); // atras
                            break;
                        case 3: 
                            girar(1); // girar der
                            break;
                        default:
                            // xd
                        }
                    dfs(visited,newX, newY);
                    //regreso (movimiento contrario)
                    switch(i) {
                        case 0:
                            move(1); // adelante
                            break;
                        case 1:
                            girar(1); // girar izquierda
                            break;
                        case 2:
                            move(0); // atras
                            break;
                        case 3: 
                            girar(0); // girar der
                            break;
                        default:
                            // xd
                        }
                        for(int j = i; j>0; j--){
                            girarIzq();
                        }
                        
                        //e
                }
            }
        }else{
            girarIzq();
        }
    }
    return; 
}

int main() {
    int count = 0;
    // arreglo para llevar el control de las celdas visitadas
    bool visited[5][3] = {{false}}; 
    // punto inicial
    int start_x = 1, start_y = 1;

    
    // iniciar DFS
    dfs(visited, start_x, start_y);
    
    // imprimir matriz con los valores de distancia
    for (int i = 0; i< 5; i++){
        for (int j= 0; j< 3; j++){
            cout << backstep[i][j];
        }
    }
    return 0;
}

