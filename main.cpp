
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <utility>
#include <chrono>
using namespace std;
using namespace std::chrono;

struct Node {
    int x, y;
    float cost, priority;
    bool operator>(const Node& other) const {
        return priority > other.priority;
    }
};

float heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

vector<pair<int, int>> directions = {{0,1}, {1,0}, {0,-1}, {-1,0}};

float getAnomalyCost(int x, int y, const vector<vector<float>>& heatmap) {
    return 1.0 + heatmap[x][y];
}

vector<pair<int, int>> AAAStar(const vector<vector<float>>& heatmap, pair<int,int> start, pair<int,int> goal) {
    int n = heatmap.size(), m = heatmap[0].size();
    vector<vector<float>> gScore(n, vector<float>(m, INFINITY));
    vector<vector<pair<int, int>>> parent(n, vector<pair<int, int>>(m, make_pair(-1, -1)));
    priority_queue<Node, vector<Node>, greater<Node>> open;
    gScore[start.first][start.second] = 0;
    open.push({start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second)});

    while (!open.empty()) {
        Node current = open.top();
        open.pop();
        if (current.x == goal.first && current.y == goal.second) break;

        for (size_t d = 0; d < directions.size(); ++d) {
            int dx = directions[d].first;
            int dy = directions[d].second;
            int nx = current.x + dx, ny = current.y + dy;
            if (nx >= 0 && ny >= 0 && nx < n && ny < m) {
                float cost = gScore[current.x][current.y] + getAnomalyCost(nx, ny, heatmap);
                if (cost < gScore[nx][ny]) {
                    gScore[nx][ny] = cost;
                    parent[nx][ny] = make_pair(current.x, current.y);
                    float priority = cost + heuristic(nx, ny, goal.first, goal.second);
                    open.push({nx, ny, cost, priority});
                }
            }
        }
    }

    vector<pair<int, int>> path;
    for (pair<int, int> at = goal; at != make_pair(-1, -1); at = parent[at.first][at.second])
        path.push_back(at);
    reverse(path.begin(), path.end());
    return path;
}

void printGrid(const vector<vector<float>>& heatmap, const vector<pair<int,int>>& path) {
    int n = heatmap.size(), m = heatmap[0].size();
    vector<vector<char>> grid(n, vector<char>(m, '.'));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < m; ++j)
            if (heatmap[i][j] > 1.0) grid[i][j] = '#';

    for (size_t i = 0; i < path.size(); ++i) {
        int x = path[i].first, y = path[i].second;
        grid[x][y] = '*';
    }

    cout << "Grid with path ('*') and anomalies ('#'):" << endl;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j)
            cout << grid[i][j] << " ";
        cout << endl;
    }
}

int main() {
    vector<vector<float>> heatmap = {
        {0, 0, 0.5, 0},
        {0, 1.5, 1.5, 0},
        {0, 0, 0, 0}
    };
    pair<int, int> start = make_pair(0, 0), goal = make_pair(2, 3);
    auto start_time = high_resolution_clock::now();
    vector<pair<int, int>> path = AAAStar(heatmap, start, goal);
    auto end_time = high_resolution_clock::now();

    cout << "Path: ";
    for (size_t i = 0; i < path.size(); ++i)
        cout << "(" << path[i].first << "," << path[i].second << ") ";
    cout << endl;

    printGrid(heatmap, path);
    auto duration = duration_cast<milliseconds>(end_time - start_time).count();
    cout << "Execution Time: " << duration << " ms" << endl;
    return 0;
}
