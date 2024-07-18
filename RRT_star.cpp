#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>

using namespace std;

const int ROWS = 10;
const int COLS = 12;
const int OBSTACLE = 1;
const int FREE = 0;
const int STEP_SIZE = 1;
const double MAX_ITERATION = 10100;

struct Node
{
    int x, y;
    double costfromneighbor, costfromroot;
    Node* parent;
};

class RRTstar
{
    private:
        vector<vector<int>> matrix;
        vector<Node*> nodes;
        Node* start;
        Node* goal;
        int iterations;
        bool pathFound = false, goalFound = false;

        bool isFree(int x, int y)
        {
            if(x<0 || y<0 || y>=COLS || x>=ROWS || matrix[x][y] == OBSTACLE)
            {
                return false;
            }

            return matrix[x][y] == FREE;
        }

        float distance(Node* node1, Node* node2)
        {
            int dx = node1->x - node2->x;
            int dy = node1->y - node2->y;
            return sqrt(pow(dx,2) + pow(dy,2));
        }

        void CostNode(Node* node1)
        {
            Node* parent = node1->parent;

            double cost = sqrt(pow(node1->x - parent->x,2) + pow(node1->y - parent->y, 2));
            double cost2root = sqrt(pow(node1->x - start->x,2) + pow(node1->y - start->y, 2));

            node1->costfromneighbor = cost;
            node1->costfromroot = cost2root;
        }

        Node* randomNode()
        {
            Node* node = new Node();

            node->x = rand() % ROWS;
            node->y = rand() % COLS;
            node->parent = nullptr;
            node->costfromneighbor = 0;
            node->costfromroot = 0;

            return node;
        }

        Node* nearestNode(Node* node)
        {
            Node* nearest = nodes[0];
            float minDistance = distance(node, nearest);

            for(int i=0; i<nodes.size(); i++){
                float d = distance(node, nodes[i]);
                if(d<minDistance)
                {
                    nearest = nodes[i];
                    minDistance = d;
                }
            }
            return nearest;
        }

        Node* newNode(Node* nearest, Node* random)
        {
            Node* node = new Node();
            float d = distance(nearest, random);
            if(d<=STEP_SIZE)
            {
                node->x = random->x;
                node->y = random->y;
            }
            else
            {
                float theta = atan2(random->y - nearest->y, random->x - nearest->x);
                node->x = nearest->x + STEP_SIZE * cos(theta);
                node->y = nearest->y + STEP_SIZE * sin(theta);
            }
            node->parent = nearest;
            CostNode(node);
            return node;
        }

        bool hasReachedGoal(Node* node)
        {
            return distance(node, goal) <= STEP_SIZE;
        }


        bool isPathCollisionFree(Node* n1, Node* n2)
        {
            if (n1 == nullptr || n2 == nullptr) {
                return false;
            }

            int dx = abs(n2->x - n1->x);
            int dy = abs(n2->y - n1->y);

            int step_x = n1->x < n2->x ? 1 : -1;
            int step_y = n1->y < n2->y ? 1 : -1;

            int delta = dx - dy;

            int x = n1->x;
            int y = n1->y;

            while (true)
            {
                int error = 2 * delta;

                if (error > -dy)
                {
                    delta -= dy;
                    x += step_x;
                }

                if (error < dx)
                {
                    delta += dx;
                    y += step_y;
                }

                if (x == n2->x && y == n2->y) // Added break condition
                {
                    break;
                }

                if (!isFree(x, y))
                {
                    return false;
                }
            }

            return true;
        }

        void findPath()
        {
            Node* current = goal;
            while(current != NULL && current != start)
            {
                if(current->x <= 0 || current->y < 0 || current->x >= ROWS || current->y >= COLS)
                {
                    break;
                }
                current = current->parent;
                if(current != NULL)
                {
                    matrix[current->x][current->y] = 2;
                }
            }
                if(current == start)
                {
                    matrix[current->x][current->y] = 2;
                }
        }

    public:
        RRTstar() : matrix(ROWS, vector<int>(COLS, 0))
        {
            srand(time(NULL));

            matrix = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},//  0
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},//  1
                      {1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1},//  2
                      {1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1},//  3
                      {1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1},//  4
                      {1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1},//  5
                      {0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0},//  6
                      {1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1},//  7
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},//  8
                      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};// 9
                    /* 0  1  2  3  4  5  6  7  8  9  10 11 */
            
            start = new Node();
            start->x = 1;
            start->y = 1;
            start->parent = nullptr;
            start->costfromneighbor = 0;
            start->costfromroot = 0;

            goal = new Node();
            goal->x = 8;
            goal->y = 10;
            goal->parent = nullptr;
            goal->costfromneighbor = 0;
            goal->costfromroot = 0;

            nodes.push_back(start);

            iterations = 0;
            pathFound = false;
        }

        bool GoalReach(Node* cell)
        {
            if(cell == goal)
                return true;
            return false;
        }

        void generateRRTstar(int max_iterations)
        {
            int iteration_count = 0;
            bool goalFound = false;
            do
            {
                Node* random = randomNode();
                Node* nearest = nearestNode(random);
                Node* node = newNode(nearest, random);

                if(isFree(node->x, node->y) && isPathCollisionFree(nearest, node))
                {
                    nodes.push_back(node);
                    
                    if(hasReachedGoal(node))
                    {
                        goalFound = true;
                        pathFound = true;
                        findPath();
                        break;
                    }
                    
                    matrix[node->x][node->y] = 3;

                    vector<Node*> path;
                    Node* current = node;
                    while (current != start)
                    {
                        path.push_back(current);
                        current = current->parent;
                    }
                    path.push_back(start);

                    for(int i = path.size()-1; i>=1; i--
                    )
                    {
                        Node* node1 = path[i];
                        Node* node2 = path[i-1];
                        if(isPathCollisionFree(node1,node2))
                        {
                            matrix[node1->x][node1->y] = 2;
                            matrix[node2->x][node2->y] = 2;

                            // cout<<"("<<node1->x<<","<<node1->y<<")"<<endl;
                            // cout<<"("<<node2->x<<","<<node2->y<<")"<<endl;
                        }
                        else
                            continue;
                    }
                    iteration_count++;

                    // cout<<iteration_count<<endl; //Print the Iteration
                    if(iteration_count>= max_iterations)
                        break;
                }
            }while (!goalFound);            
        }

        int PrintMatrix()
        {
            vector<vector<int>> Matrix = matrix;
            cout<<"\t-- Binary Matrix --"<<endl;
            for(int i=0; i<ROWS; i++)
            {
                cout<<"\t";
                for(int j=0; j<COLS; j++)
                    cout<<Matrix[i][j]<<" ";
                cout<<endl;
            }
            cout<<endl;
            return 0;
        }

        int PrintPath(bool visualize = true)
        {
            reverse(nodes.begin(), nodes.end());
            Node* current;
            current = nodes[0];
            vector<Node*> path;
            vector<vector<int>> Path = matrix;
            do{
                Path[current->x][current->y] = 8;
                path.push_back(current);
                current = current->parent;
            }while(current->parent != NULL);

            if(!visualize)
            {
                cout<<"\t-- Path Found -- "<<endl;
                for(int i=0; i<ROWS; i++)
                {
                    cout<<"\t";
                    for(int j=0; j<COLS; j++)
                    {
                        cout<<Path[i][j]<<" ";
                    }
                    cout<<endl;
                }
                cout<<endl;
            }
            else
            {
                cout<<"\t-- Path Visulization --"<<endl;
                for(int i=0; i<ROWS; i++)
                {
                    cout<<"\t";
                    for(int j=0; j<COLS; j++)
                    {
                        if(i == start->x && j == start->y)
                            cout<<"->";
                        else if(i == goal->x && j == goal->y)
                            cout<<"#";
                        else if(Path[i][j] == 8)
                            cout<<"* ";
                        else if(Path[i][j] == 1)
                            cout<<"| ";
                        else
                            cout<<"  ";
                    }
                    cout<<endl;
                }
                cout<<endl;
            }
            return 0;
        }

        void PrintNode()
        {
            cout<<nodes.size()<<endl;
            for(int i=0; i<nodes.size(); i++)              
                cout<<"Node "<<i<<": ("<<nodes[i]->x<<","<<nodes[i]->y<<") \t Cost: "<<nodes[i]->costfromneighbor<<"\t Root: "<<nodes[i]->costfromroot<<"\t Parent : ("<<nodes[i]->parent->x<<","<<nodes[i]->parent->x<<")|"<<endl;
        }

};

int main()
{
    RRTstar rrt_star = RRTstar();

    // rrt_star.PrintMatrix();
    // cout<<endl;

    rrt_star.generateRRTstar(MAX_ITERATION);

    // cout<<"\t-- Node Explored -- "<<endl;
    // rrt_star.PrintMatrix();
    
    rrt_star.PrintPath(true);

    rrt_star.PrintNode();
    
    return 0;
}