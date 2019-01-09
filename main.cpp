#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <fstream>
#include <ctime>
#include<limits>
#include <chrono>
using namespace std;

struct Vertex {
    int x;
    int y;
};

void generate() {
    srand(time(NULL));
    int n = rand() % 1001;
    ofstream plik1;
    plik1.open("7.txt");
    if (!plik1.good()) {
        cout << plik1.good() << endl;
        cout << "something went wrong" << endl;
    }
    plik1 << n << endl;
    int i = 1;
    int x, y;
    while (i <= n) {
        x = rand() % 1001;
        y = rand() % 1001;
        plik1 << i << " " << x << " " << y << endl;
        i++;
    }
    plik1.close();
}

void load_coordinates(vector<Vertex> &vertexes, int &n, string file_name) {
    fstream file;
    file.open(file_name);
    file >> n;
    Vertex vertex{};
    int j;
    for (int i = 1; i <= n; i++) {
        file >> j >> vertex.x >> vertex.y;
        vertexes.push_back(vertex);
    }
    file.close();
}

double count_distance(Vertex a, Vertex b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void create_matrix(vector<Vertex> vertexes, double **distances) {
    for (int i = 0; i < vertexes.size(); i++) {
        for (int j = 0; j < vertexes.size(); j++) {
            distances[i][j] = count_distance(vertexes[i], vertexes[j]);
        }
    }
}

void print_matrix(double **distances, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cout << distances[i][j] << " ";
        }
        cout << endl;
    }
}

void showPath(int *path, int n) {
    for (int i = 0; i < n; i++) cout << path[i] + 1 << " ";
    cout << endl;
}

void greedy(int current_vertex, double **A, bool *visited, int *path, int n, double &cost, int pathPosition) {
    double min_cost = 9999999;
    int next_vertex = -1;
    visited[current_vertex] = true;
    bool every_vertex_visited = true;
    for (int i = 0; i < n; i++) {
        if (i != current_vertex) {
            if (!visited[i]) {
                every_vertex_visited = false;
                if (A[current_vertex][i] < min_cost) {
                    next_vertex = i;
                    min_cost = A[current_vertex][i];
                }
            }
        }
    }
    if (!every_vertex_visited) {
        path[pathPosition++] = current_vertex;
        cost += min_cost;
        greedy(next_vertex, A, visited, path, n, cost, pathPosition);
    } else {
        path[pathPosition++] = current_vertex;
        cost += A[path[pathPosition - 1]][path[0]];
    }
}

double pathCost(int *path, double **distances, int n) {
    double cost = 0;
    for (int i = 0; i < n - 1; i++) {
        cost += distances[path[i]][path[i + 1]];
    }
    return cost + distances[path[n - 1]][path[0]];
}

double count2OptPathCost(vector<int> path, double **A) {
    double cost = 0;
    for (int i = 1; i < path.size(); i++) {
        cost += A[path[i - 1] - 1][path[i] - 1];
    }
    cost += A[path[0] - 1][path[path.size() - 1] - 1];
    return cost;
}

void twoOptSwap(vector<int> &newRoute, vector<int> currentRoute, int i, int k) {
    for (int j = 0; j < i; j++) newRoute.push_back(currentRoute[j]);
    for (int j = k; j >= i; j--) newRoute.push_back(currentRoute[j]);
    for (int j = k + 1; j < currentRoute.size(); j++) newRoute.push_back(currentRoute[j]);
}

void twoOptLocalSearch(vector<int> &path, double &cost, double **A) {
    bool improvementMade = false;
    do {
        improvementMade = false;
        for (int i = 0; i < path.size(); i++) {
            for (int k = i + 1; k < path.size(); k++) {
                vector<int> newPath;
                twoOptSwap(newPath, path, i, k);
                double newCost = count2OptPathCost(newPath, A);
                if (newCost < cost) {
                    path = newPath;
                    //show2OptPath(path);
                    cost = newCost;
                    improvementMade = true;
                }
            }
        }
    } while (improvementMade);
    path.push_back(path[0]);
}

void show2OptPath(vector<int> path) {
    cout << "path: ";
    for (int i = 0; i < path.size(); i++)cout << path[i] << ' ';
    cout << endl;
}

void getBestNearbySolution(int it, double **distances, int n, int *path, int maxTabuValue, int **tabuList,
                           int **tabuFrequencyList, int penaltyValue, double globalBestCost) {
    double localBestCost = std::numeric_limits<double>::max();
    int firstNodeBeingSwapped = 0;
    int secondNodeBeingSwapped = 1;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            swap(path[i], path[j]);
            double currentCost = pathCost(path, distances, n);
            double penaltyCost = currentCost + penaltyValue * tabuFrequencyList[i][j];
            if ((localBestCost > penaltyCost && tabuList[i][j] <= it) || currentCost < globalBestCost) {
                firstNodeBeingSwapped = i;
                secondNodeBeingSwapped = j;
                localBestCost = penaltyCost;
                tabuList[i][j] = it + maxTabuValue;
                tabuList[j][i] = it + maxTabuValue;
            }
            swap(path[j], path[i]);
        }
    }
    tabuFrequencyList[firstNodeBeingSwapped][secondNodeBeingSwapped] += 1;
    swap(path[firstNodeBeingSwapped], path[secondNodeBeingSwapped]);
}

void tabuSearch(double **distances, int *path, int n, int numberOfIterations, int maxIterationsWithoutImprovement,
                int penaltyValue, int maxTabuValue, double &globalBestCost) {
    int *bestPath = new int[n];
    int **tabuList = new int *[n];
    int **tabuFrequencyList = new int *[n];
    for (int i = 0; i < n; i++) {
        bestPath[i] = path[i];
        tabuFrequencyList[i] = new int[n];
        tabuList[i] = new int[n];
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            tabuList[i][j] = 0;
            tabuFrequencyList[i][j] = 0;
        }
    }
    int iterationsWithoutImprovements = 0;
    for (int i = 0; i < numberOfIterations; i++) {
        getBestNearbySolution(i, distances, n, path, maxTabuValue, tabuList, tabuFrequencyList, penaltyValue,
                              globalBestCost);
        double cost = pathCost(path, distances, n);
        if (cost < globalBestCost) {
            globalBestCost = cost;
            iterationsWithoutImprovements = 0;
            for (int p = 0; p < n; p++) bestPath[p] = path[p];
        } else {
            iterationsWithoutImprovements++;
            if (iterationsWithoutImprovements > maxIterationsWithoutImprovement) {
                iterationsWithoutImprovements = 0;
                int minFrequency = tabuFrequencyList[0][0];
                int minFrequencyRow = 0;
                int minFrequencyColumn = 0;
                for (int r = 0; r < n; r++) {
                    for (int c = 0; c < n; c++) {
                        if (tabuFrequencyList[r][c] < minFrequency) {
                            minFrequency = tabuFrequencyList[r][c];
                            minFrequencyRow = r;
                            minFrequencyColumn = c;
                        }
                    }
                }
                swap(path[minFrequencyRow], path[minFrequencyColumn]);
                cost = pathCost(path, distances, n);
                if (cost < globalBestCost) {
                    globalBestCost = cost;
                    iterationsWithoutImprovements = 0;
                    for (int p = 0; p < n; p++) bestPath[p] = path[p];
                }
                for (int r = 0; r < n; r++) {
                    for (int c = 0; c < n; c++) {
                        tabuList[r][c] = 0;
                        tabuFrequencyList[r][c] = 0;
                    }
                }
            }
        }
        if (i % 100 == 0)cout << i << endl;
    }
    for (int p = 0; p < n; p++) path[p] = bestPath[p];
}

//void getRandomPath(int n,int *path){
//    for(int i =0;i<n;i++) path[i]=i;
//    srand(time(NULL));
//    for(int i = n-1; i >= 0; i--){
//        int j = rand() % n;
//        swap(path[i],path[j]);
//    }
//}

int main() {
    //generate();
    int n;
    double cost = 0;
    int starting_vertex = 0;
    string file_name2 = "berlin52.txt";
    vector<Vertex> vertexes;
    load_coordinates(vertexes, n, file_name2);
    double **distances = new double *[n];
    bool *visited = new bool[n];
    for (int i = 0; i < n; i++) {
        visited[i] = false;
        distances[i] = new double[n];
    }
    create_matrix(vertexes, distances);
    //print_matrix(distances, n);
    int *path = new int[n];
    int pathPosition = 0;
    greedy(starting_vertex, distances, visited, path, n, cost, pathPosition);
    cout << "greedy" << endl;
    cout << "cost: " << cost << endl;
    cout << "path: ";
    showPath(path, n);
//    vector<int> pathFor2Opt;
//    double twoOptCost = cost;
//    for (int i = 0; i < n; i++)pathFor2Opt.push_back(path[i] + 1);
//    twoOptLocalSearch(pathFor2Opt, twoOptCost, distances);
//    cout << endl << "2opt" << endl;
//    cout << "cost: " << twoOptCost << endl;
//    show2OptPath(pathFor2Opt);
    int numberOfIterations = 15000;
    int maxIterationsWithoutImprovement = 500;
    int penaltyValue = 10;
    int maxTabuValue = static_cast<int>(sqrt(n));
//    for (int i = 0; i < n; i++)path[i] = pathFor2Opt[i] - 1;
//    cost = twoOptCost;
//    showPath(path, n);
    tabuSearch(distances, path, n, numberOfIterations, maxIterationsWithoutImprovement, penaltyValue, maxTabuValue,
              cost);
    cout << endl << "Tabu Search" << endl;
    cout << "cost: " << pathCost(path, distances, n) << endl;
    cout << " final path: ";
    showPath(path, n);



    //int path3[52] = {1, 49, 32, 45, 19, 41, 8, 9, 10, 43, 33, 51, 11, 52, 14, 13, 47, 26, 27, 28, 12, 25, 4, 6, 15, 5, 24, 48, 38, 37, 40, 39, 36, 35, 34, 44, 46, 16, 29, 50, 20, 23, 30, 2, 7, 42, 21, 17, 3, 18, 31, 22}; //optimum






    return 0;
}


