//Zona C !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#include <iostream>
using namespace std;

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
}

// arriba, izquierda, abajo, derecha
void search(bool visited[5][3], int x, int y, int directions[4][2], int backstep[5][3], int& count, bool& pathFound){
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
        if(newX >= 0 && newY >=0 && newX<=4 && newY <= 2){
            if(visited[newX][newY] == false){
                cout << "Visitando " << newX << ", " << newY << endl;
                search(visited, newX, newY, directions, backstep, count, pathFound);
                cout << "Retrocediendo a: " << x << ", " << y << endl; 
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
    for(int i = 0; i < count; i++){
        for (int j = 0; j < 4; j++){
            int newX = x + directions[0][0];
            int newY = y + directions[0][1];
            if (newX >= 0 && newY >= 0 && newX < 5 && newY < 3 && backstep[newX][newY] == i+1){
                x = newX;
                y = newY;
                cout << "Visitando " << x << ", " << y << endl;
                j=4;
            }else{
                girar(directions);
            }
        }
    }
}
int main() {
    //adelante, izquierda, atras, derecha
    int directions[4][2] = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
    
    int maze[5][3] = {{0}};

    bool visited[5][3] = {{false}}; 
    int backstep[5][3] = {{30}}; 
    int count = 0;
    // punto inicial
    int start_x = 0, start_y = 0;
    int colors[3] = {0};

    bool pathFound = false;
    search(visited, start_x, start_y, directions, backstep, count, pathFound);
    cout << "Count: "<< count << ", " << "Path Found: " << pathFound << endl;
    fuga(count, start_x, start_y, directions, backstep);
}