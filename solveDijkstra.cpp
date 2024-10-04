#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <limits>

using namespace std;

const vector<pair<int, int> > directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
const int INF = numeric_limits<int>::max();

struct Node {
    int x, y, cost;
    bool operator>(const Node& other) const {
        return cost > other.cost; 
    }
};

void dijkstra() {

}

int main() {
    int maze[5][5]= {
        {0, 1, 0, 0, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {1, 1, 0, 0, 0},
        {0, 0, 1, 1, 0}
    };

    dijkstra(); // De (0,0) a (4,4)

    return 0;
}
