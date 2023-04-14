#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <memory>

using namespace std;

const int ROWS = 12;
const int COLS = 12;
const int OBSTACLE = 1;
const int FREE = 0;
const int STEP_SIZE = 1;
const double MAX_ITERATION = 100;

struct Point
{
    int x, y;
    double goalbias;
};

struct Node
{
    Point position;
    Node *parent;
};

class InformedRRT
{
private:
    vector<vector<int>> Map;
    vector<Node *> Nodes;
    Node *start;
    Node *goal;
    bool PathComplete = false, Reached = false;

    bool isFree(int x, int y)
    {
        if (x < 0 || y < 0 || x >= ROWS || y >= COLS || Map[x][y] == OBSTACLE)
            return false;

        return Map[x][y] == FREE;
    }

    float distance(Node *n1, Node *n2)
    {
        return sqrt(pow(n1->position.x - n2->position.x, 2) + pow(n1->position.y - n2->position.y, 2));
    }

    float Bias(Node *node)
    {
        return sqrt(pow(goal->position.x - node->position.x, 2) + pow(goal->position.y - node->position.y, 2));
    }

    Node *RandNode()
    {
        Node *node = new Node();
        node->position.x = rand() % ROWS;
        node->position.y = rand() % COLS;
        node->parent = nullptr;

        return node;
    }

    Node *nearestNode(Node *node)
    {
        Node *nearest = Nodes[0];
        // cout<<nodes[0]->x<<"|"<<nodes[0]->y<<endl;
        float minDistance = distance(node, nearest);

        for (int i = 0; i < Nodes.size(); i++)
        {
            float d = distance(node, Nodes[i]);
            if (d < minDistance)
            {
                nearest = Nodes[i];
                minDistance = d;
            }
        }

        return nearest;
    }
};