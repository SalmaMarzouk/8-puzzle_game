#include <iostream>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <queue>
#include <chrono>
#include <stack>
#include <set>

using namespace std;
//9!*4 + 9!
//structure to return the output
struct resultSet {
    resultSet(vector<string> &p, int d, int c, int exp, long long t) {
        path = p;
        depth = d;
        cost = c;
        expanded = exp;
        runningTime = t;
    }

    vector<string> path;
    int depth;
    int cost;
    int expanded;
    long long runningTime;
};

//remove commas and spaces from the input
string parseInput(string &state) {
    string parsed;
    for (char c : state) {
        if (c != ',' && c != ' ') {
            parsed.push_back(c);
        }
    }
    return parsed;
}

//find the index of the blank cell
int getBlank(string &state) {
    return (int) state.find('0');
}

//find the neighbours of the current state
vector<string> getNeighbours(string &currState, unordered_map<string, string> &parent) {
    vector<string> neighbours;
    int blankPos = getBlank(currState);
    //indices of the blank cell
    int row = blankPos / 3, col = blankPos % 3;

    //add to neighbours if available (no borders, not visited)

    if (row + 1 <= 2) {
        int newBlankPos = (row + 1) * 3 + col;
        //down move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!parent.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }

    if (col + 1 <= 2) {
        int newBlankPos = row * 3 + (col + 1);
        //right move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!parent.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }
    if (col - 1 >= 0) {
        int newBlankPos = row * 3 + (col - 1);
        //left move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!parent.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }
    if (row - 1 >= 0) {
        int newBlankPos = (row - 1) * 3 + col;
        //up move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!parent.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }
    return neighbours;
}

//get neighbours of the current state for A*
vector<string> getNeighbours(string &currState, unordered_set<string> &visited) {
    vector<string> neighbours;
    int blankPos = getBlank(currState);
    //indices of the blank cell
    int row = blankPos / 3, col = blankPos % 3;

    //add to neighbours if available (no borders, not visited)
    if (row + 1 <= 2) {
        int newBlankPos = (row + 1) * 3 + col;
        //down move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!visited.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }

    if (col + 1 <= 2) {
        int newBlankPos = row * 3 + (col + 1);
        //right move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!visited.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }
    if (col - 1 >= 0) {
        int newBlankPos = row * 3 + (col - 1);
        //left move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!visited.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }
    if (row - 1 >= 0) {
        int newBlankPos = (row - 1) * 3 + col;
        //up move
        swap(currState[blankPos], currState[newBlankPos]);
        if (!visited.contains(currState)) {
            neighbours.push_back(currState);
        }
        swap(currState[blankPos], currState[newBlankPos]);
    }
    return neighbours;
}


//push neighbours in a queue
void pushManyQ(queue<string> &q, vector<string> &elements, unordered_map<string, string> &parent, string &currState) {
    for (auto &element : elements) {
        q.push(element);
        parent[element] = currState;
    }
}

//push neighbours in a stack
void pushManySt(stack<pair<string, int>> &st, vector<string> &elements, unordered_map<string, string> &parent,
                string &currState, int currDepth) {
    for (auto &element : elements) {
        st.push({element, currDepth + 1});
        parent[element] = currState;
    }
}

//BFS search
resultSet BFS(string &initial) {
    auto start = std::chrono::high_resolution_clock::now();     //set the start time
    string goalState = "012345678";
    int depth = 0;
    queue<string> bfsQ;
    int nodesExpanded = 0;
    unordered_map<string, string> parent;
    string currState;
    bfsQ.push(initial);
    parent[initial] = initial;      // set the parent of the initial state as the initial itself
    while (!bfsQ.empty()) {
        int size = (int) bfsQ.size();
        //while loop to expand the nodes of each level
        while (size--) {
            currState = bfsQ.front();
            bfsQ.pop();
            if (currState == goalState) {
                break;
            }
            vector<string> neighbours = getNeighbours(currState, parent);
            nodesExpanded++;
            pushManyQ(bfsQ, neighbours, parent, currState); //add neighbours of the current state to the bfs queue
        }
        //break if goal is found
        if (size != -1) {
            break;
        }
        depth++;
    }
    vector<string> path;
    auto stopSearch = std::chrono::high_resolution_clock::now();   //set the end time if the goal is not found
    auto searchDur = duration_cast<std::chrono::microseconds>(stopSearch - start);
    if (currState != goalState) {
        return resultSet(path, depth, INT_MAX, nodesExpanded, searchDur.count());
    }
    while (parent[currState] != currState) {
        path.push_back(currState);
        currState = parent[currState];
    }
    path.push_back(currState);
    reverse(path.begin(), path.end());
    auto stop = std::chrono::high_resolution_clock::now();          //set the end time if goal is found
    auto duration = duration_cast<std::chrono::microseconds>(stop - start);
    return resultSet(path, depth, depth, nodesExpanded, duration.count());
}

//DFS search
resultSet DFS(string &initial) {
    auto start = std::chrono::high_resolution_clock::now();     //set the start time
    string goalState = "012345678";
    int maxDepth = 0;
    stack<pair<string, int>> dfsSt;      //associate the node level
    int nodesExpanded = 0;
    unordered_map<string, string> parent;
    string currState;
    dfsSt.push({initial, 0});
    parent[initial] = initial;      // set the parent of the initial state as the initial itself
    while (!dfsSt.empty()) {
        currState = dfsSt.top().first;
        int currDepth = dfsSt.top().second;
        maxDepth = max(maxDepth, currDepth);
        dfsSt.pop();
        if (currState == goalState) {
            break;
        }
        vector<string> neighbours = getNeighbours(currState, parent);
        nodesExpanded++;
        pushManySt(dfsSt, neighbours, parent, currState,
                   currDepth); //add neighbours of the current state to the dfs stack
    }
    vector<string> path;
    auto stopSearch = std::chrono::high_resolution_clock::now();   //set the end time if goal is not found
    auto searchDur = duration_cast<std::chrono::microseconds>(stopSearch - start);
    if (currState != goalState) {
        return resultSet(path, maxDepth, INT_MAX, nodesExpanded, searchDur.count());
    }
    while (parent[currState] != currState) {
        path.push_back(currState);
        currState = parent[currState];
    }
    path.push_back(currState);
    reverse(path.begin(), path.end());
    auto stop = std::chrono::high_resolution_clock::now();          //set the end time if goal is found
    auto duration = duration_cast<std::chrono::microseconds>(stop - start);
    return resultSet(path, maxDepth, (int) path.size() - 1, nodesExpanded, duration.count());

}

//compute h value using Manhattan Distance heuristic
int hMan(string &state) {
    int h = 0;
    for (int i = 0; i < state.size(); i++) {
        int n = state[i] - '0';
        h += abs(i / 3 - n / 3);
        h += abs(i % 3 - n % 3);
    }
    return h;
}

//compute h value using Euclidean Distance heuristic
double hEuc(string &state) {
    double h = 0;
    for (int i = 0; i < state.size(); i++) {
        int n = state[i] - '0';
        int r1 = i / 3, r2 = n / 3, c1 = i % 3, c2 = n % 3;
        h += sqrt((r1 - r2) * (r1 - r2) + (c1 - c2) * (c1 - c2));
    }
    return h;
}

//compute f value using Manhattan Distance heuristic
vector<int> getManFs(vector<string> &neighbours, int g) {
    vector<int> Fs(neighbours.size());
    for (int i = 0; i < neighbours.size(); i++) {
        Fs[i] = g + hMan(neighbours[i]);
    }
    return Fs;
}

//compute f value using Euclidean Distance heuristic
vector<double> getEucFs(vector<string> &neighbours, int g) {
    vector<double> Fs(neighbours.size());
    for (int i = 0; i < neighbours.size(); i++) {
        Fs[i] = g + hEuc(neighbours[i]);
    }
    return Fs;
}

//A* search using Manhattan Distance heuristic
resultSet AStarMan(string &initial) {
    auto start = std::chrono::high_resolution_clock::now();     //set the start time
    string goalState = "012345678";
    set<pair<int, string>> minPQ;
    unordered_map<string, int> depth;
    unordered_set<string> visited;
    unordered_map<string, string> parent;
    unordered_map<string, int> f;
    int maxDepth = 0;
    int expanded = 0;
    f[initial] = hMan(initial);
    minPQ.insert({f[initial], initial});     //associate the cost of the initial state
    depth[initial] = 0;
    parent[initial] = initial;      //set the parent of the initial state as itself
    pair<int, string> currState;
    while (!minPQ.empty()) {
        currState = *minPQ.begin();
        minPQ.erase(minPQ.begin());
        visited.insert(currState.second);
        if (currState.second == goalState) {
            break;      //stop when the goal is reached
        }
        vector<string> neighbours = getNeighbours(currState.second, visited);
        vector<int> neighboursFs = getManFs(neighbours, depth[currState.second] + 1);
        for (int i = 0; i < neighbours.size(); i++) {
            //if the neighbour has f value, compare with the new value and associate it with the minimum f
            if (f.contains(neighbours[i])) {
                if (neighboursFs[i] < f[neighbours[i]]) {
                    minPQ.erase({f[neighbours[i]], neighbours[i]});
                    minPQ.insert({neighboursFs[i], neighbours[i]});
                    f[neighbours[i]] = neighboursFs[i];
                    depth[neighbours[i]] = depth[currState.second] + 1;
                    parent[neighbours[i]] = currState.second;
                }
            }
            //if not, associate the neighbour with its f value
            else {
                minPQ.insert({neighboursFs[i], neighbours[i]});
                f[neighbours[i]] = neighboursFs[i];
                depth[neighbours[i]] = depth[currState.second] + 1;
                parent[neighbours[i]] = currState.second;
            }
            maxDepth = max(maxDepth, depth[neighbours[i]]);
        }
        expanded++;
    }
    vector<string> path;
    auto stopSearch = std::chrono::high_resolution_clock::now();   //set the end time if goal not found
    auto searchDur = duration_cast<std::chrono::microseconds>(stopSearch - start);
    if (currState.second != goalState) {
        return resultSet(path, maxDepth, INT_MAX, expanded, searchDur.count());
    }
    //compute path if the goal is found
    while (parent[currState.second] != currState.second) {
        path.push_back(currState.second);
        currState.second = parent[currState.second];
    }
    path.push_back(currState.second);
    reverse(path.begin(), path.end());
    auto stop = std::chrono::high_resolution_clock::now();          //set the end time if goal is found
    auto duration = duration_cast<std::chrono::microseconds>(stop - start);
    return resultSet(path, maxDepth, (int) path.size() - 1, expanded, duration.count());

}

//A* search using Euclidean Distance heuristic
resultSet AStarEuc(string &initial) {
    auto start = std::chrono::high_resolution_clock::now();     //set the start time
    string goalState = "012345678";
    set<pair<double, string>> minPQ;
    unordered_map<string, int> depth;
    unordered_set<string> visited;
    unordered_map<string, string> parent;
    unordered_map<string, double> f;
    int maxDepth = 0;
    int expanded = 0;
    f[initial] = hEuc(initial);
    minPQ.insert({f[initial], initial});      //associate the cost of the initial state
    depth[initial] = 0;
    parent[initial] = initial;          //set the parent of the initial state as itself
    pair<int, string> currState;
    while (!minPQ.empty()) {
        currState = *minPQ.begin();
        minPQ.erase(minPQ.begin());
        visited.insert(currState.second);
        if (currState.second == goalState) {
            break;      //stop when the goal is reached
        }
        vector<string> neighbours = getNeighbours(currState.second, visited);
        vector<double> neighboursFs = getEucFs(neighbours, depth[currState.second] + 1);
        for (int i = 0; i < neighbours.size(); i++) {
            //if the neighbour has f value, compare with the new value and associate it with the minimum f
            if (f.contains(neighbours[i])) {
                if (neighboursFs[i] < f[neighbours[i]]) {
                    minPQ.erase({f[neighbours[i]], neighbours[i]});
                    minPQ.insert({neighboursFs[i], neighbours[i]});
                    f[neighbours[i]] = neighboursFs[i];
                    depth[neighbours[i]] = depth[currState.second] + 1;
                    parent[neighbours[i]] = currState.second;
                }
            }
            //if not, associate the neighbour with its f value
            else {
                minPQ.insert({neighboursFs[i], neighbours[i]});
                f[neighbours[i]] = neighboursFs[i];
                depth[neighbours[i]] = depth[currState.second] + 1;
                parent[neighbours[i]] = currState.second;
            }
            maxDepth = max(maxDepth, depth[neighbours[i]]);
        }
        expanded++;
    }
    vector<string> path;
    auto stopSearch = std::chrono::high_resolution_clock::now();   //set the end time if goal not found
    auto searchDur = duration_cast<std::chrono::microseconds>(stopSearch - start);
    if (currState.second != goalState) {
        return resultSet(path, maxDepth, INT_MAX, expanded, searchDur.count());
    }
    //compute path if the goal is found
    while (parent[currState.second] != currState.second) {
        path.push_back(currState.second);
        currState.second = parent[currState.second];
    }
    path.push_back(currState.second);
    reverse(path.begin(), path.end());
    auto stop = std::chrono::high_resolution_clock::now();          //set the end time if goal is found
    auto duration = duration_cast<std::chrono::microseconds>(stop - start);
    return resultSet(path, maxDepth, (int) path.size() - 1, expanded, duration.count());

}

void printState(string &state){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            cout << state[i*3+j] << '\t';
        }
        cout << endl;
    }
    cout << endl;
}

void printResultSet(string algorithmName, resultSet &r) {
    cout << algorithmName << endl;
    cout << "Path:\n";
    for (auto & i : r.path) {
        printState(i);
    }
    cout << "Cost of path: " << r.cost << endl;
    cout << "Nodes expanded: " << r.expanded << endl;
    cout << "Search depth: " << r.depth << endl;
    cout << "Running time: " << r.runningTime << " microseconds" << endl;
    cout << endl;
}

int main() {
    string inputState;
    cout << "Enter the initial state: " << endl;
    getline(cin,inputState);
    string parsedInputState = parseInput(inputState);
    resultSet BFSResult = BFS(parsedInputState);
    resultSet DFSResult = DFS(parsedInputState);
    resultSet AStarManResult = AStarMan(parsedInputState);
    resultSet AStarEucResult = AStarEuc(parsedInputState);
    printResultSet("BFS",BFSResult);
    //printResultSet("DFS",DFSResult);
    //printResultSet("A* Manhattan",AStarManResult);
    //printResultSet("A* Euclidean",AStarEucResult);
    return 0;
}
