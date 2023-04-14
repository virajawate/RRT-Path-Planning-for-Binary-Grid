#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>

using namespace std;

const int ROWS = 10;
const int COLS = 12;
const int OBSTACLE =1;
const int FREE = 0;
const int STEP_SIZE  = 1;
const double MAX_ITERATIONS = 10100;

struct Node
{
    int x,y;
    Node* parent;
};

class BiRRT
{
    private:
        vector<vector<int>> matrix;
        vector<Node*> forwards;
        vector<Node*> backwards;
        vector<Node*> path;
        Node* start;
        Node* goal;
        int iteration_count;
        bool pathFound;
        bool TreeComplete = false;

        bool isFree(int x, int y){
            if(x<0 || x >= ROWS || y<0 || y >= COLS || matrix[x][y] == OBSTACLE){
                return false;
            }
            return matrix[x][y] == FREE;
        }

        float distance(Node* n1, Node* n2){ 
            int dx = abs(n1->x - n2->x);
            int dy = abs(n1->y - n2->y);
            return sqrt(dx*dx + dy*dy);
        }

        Node* randomNode(){
            Node* node = new Node();
            node->x = rand() % ROWS;
            node->y = rand() % COLS;
            node->parent = nullptr;
            // cout<<node->x<<"|"<<node->y<<endl;
            return node;
        }

        Node* nearest2start(Node* node)
        {
            Node* nearest = forwards[0];
            float minDistance = distance(node, nearest);
            for(int i=1; i<forwards.size(); i++){
                if(forwards[i]!=NULL){
                    float d = distance(node, forwards[i]);
                    if(d<minDistance){
                        nearest = forwards[i];
                        minDistance = d;
                    }
                }
            }
            return nearest;
        }

        Node* nearest2goal(Node* node)
        {
            Node* nearest = backwards[0];
            float minDistance = distance(node, nearest);

            for(int i=1; i<backwards.size(); i++)
            {
                if(backwards[i]!=NULL)
                {
                    float d = distance(node, backwards[i]);
                    if(d<minDistance)
                    {
                        nearest = backwards[i];
                        minDistance = d;
                    }
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
            return node;
        }

        bool isTreeComplete(vector<Node*> forward, vector<Node*> backward)
        {
            for(int i=0; i<forward.size(); i++)
            {
                for(int j=0; j<backward.size(); j++)
                {
                    if(distance(forward[i], backward[j]) <= STEP_SIZE)
                        return true;
                    else
                        continue;
                }
            }
            return false;
        }

        bool isPathCollisionFree(Node* n1, Node* n2)
        {
            // Calculate the distance between the two nodes
            int dx = abs(n2->x - n1->x);
            int dy = abs(n2->y - n1->y);

            // Determine the direction to move along the x and y axes
            int sx = n1->x < n2->x ? 1 : -1;
            int sy = n1->y < n2->y ? 1 : -1;

            // Initialize the error term to zero
            int err = dx - dy;
            
            // Initialize the current cell to the start node
            int x = n1->x;
            int y = n1->y;

            // Iterate through each cell along the line segment using Bresenham's algorithm
            while(x != n2->x || y != n2->y)
            {
                // Calculate the error term for the next step
                int e2 = 2*err;

                // If the error term is greater than -dy, move along the x axis
                if(e2 > -dy)
                {
                    err -= dy;
                    x += sx;
                }

                // If the error term is less than dx, move along the y axis
                if(e2 < dx)
                {
                    err += dx;
                    y += sy;
                }

                // Check if the current cell is occupied
                if(matrix[x][y] == OBSTACLE)
                {
                    // The path is not collision-free, so return false
                    return false;
                }
            }

            // All cells along the path are unoccupied, so return true
            return true;
        }


        void findPath()
        {
            Node* current = goal;
            while (current != NULL && current != start)
            {
                if (current->x <= 0 || current->x > ROWS || current->y <= 0 || current->y > COLS)
                {
                    // current is outside the bounds of the matrix
                    break;
                }
                
                if (current != NULL) { // add a check here to make sure current is not NULL
                    matrix[current->x][current->y] = 2;
                    path.push_back(current);
                }
                current = current->parent;
            }
            if (current == start)
            {
                matrix[start->x][start->y] = 2;
            }
        }
    
    public:
        BiRRT() : matrix(ROWS, vector<int>(COLS,0))
        {
            srand(time(NULL));

            matrix = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},//0
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},//1
                      {1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1},//2
                      {1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1},//3
                      {1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1},//4
                      {1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1},//5
                      {0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0},//6
                      {1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1},//7
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},//8
                      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};//9
                    /* 0  1  2  3  4  5  6  7  8  9  10 11*/
            start = new Node();
            start->x = 1;
            start->y = 1;
            start->parent = nullptr;

            goal = new Node();
            goal->x = 8;
            goal->y = 9;
            goal->parent = nullptr;

            forwards.push_back(start);
            backwards.push_back(goal);

            iteration_count = 0;
            pathFound = false;
        }

        void Connect(vector<Node*> forwards, vector<Node*> backwards)
        {
            for(int i=0; i<forwards.size(); i++)
            {
                for(int j=0; j<backwards.size(); j++)
                {
                    if(distance(forwards[i],backwards[j]) <=STEP_SIZE)
                    {
                        backwards[j]->parent = forwards[i];
                        TreeComplete = true;
                    }
                }
            }
            TreeComplete = false;
        }

        void generateRRT(int max_iterations)
        {
            bool TreeComplete = false;
            iteration_count = 0;
            path.push_back(start);
            do
            {
                Node* random = randomNode();
                Node* leaf = nearest2start(random);
                Node* root = nearest2goal(random);
                Node* forward_node = newNode(leaf, random);
                Node* backward_node = newNode(root,random);

                if(isFree(forward_node->x, forward_node->y) && isPathCollisionFree(leaf, forward_node))
                {
                    forwards.push_back(forward_node);
                }
                if(isFree(backward_node->x, backward_node->y) && isPathCollisionFree(root, backward_node))
                {
                    backwards.push_back(backward_node);
                }
                Connect(forwards, backwards);                
                iteration_count++;  // increment the iteration count
                if (iteration_count >= max_iterations)  // exit the loop if maximum iterations reached
                {
                    break;
                }
            } while (!TreeComplete);
            if(TreeComplete)
            {
                pathFound = true;
                findPath();
                path.push_back(goal);
            }
        }

        int printMatrix()
        {
            vector<vector<int>> Matrix = matrix;
            for(int i=0; i<ROWS; i++)
            {
                cout<<"     ";
                for(int j=0; j<COLS; j++)
                {
                    cout<<Matrix[i][j]<<" ";
                }
                cout<<endl;
            }
            cout<<endl;
            // cout<<matrix[9][1]<<endl;
            return 0;
        }

        void printNodes()
        {
            cout<<"Forward:"<<endl;
            for (int i = 0; i < forwards.size(); i++)
            {
                cout << "Node " << i << ": (" << forwards[i]->x << ", " << forwards[i]->y << ")" <<"Parent is : " << forwards[i]->parent <<","<< &forwards[i]->parent<< endl;
            }
            cout<<endl;
            cout<<"Backward:"<<endl;
            for (int i = 0; i < backwards.size(); i++)
            {
                cout << "Node " << i << ": (" << backwards[i]->x << ", " << backwards[i]->y << ")" <<"Parent is : " << backwards[i]->parent <<","<< &backwards[i]->parent<< endl;
            }
        }

        void printPath()
        {
            reverse(path.begin(), path.end());
            Node* current;
            current = path[0];
            vector<vector<int>> Path = matrix;
            do
            {
                Path[current->x][current->y]=8;
                current = current->parent;
            }while (current->parent != NULL);
            
            for(int i=0; i<ROWS; i++)
            {
                cout<<"     ";
                for(int j=0; j<COLS; j++)
                {
                    cout<<Path[i][j]<<" ";
                }
                cout<<endl;
            }
            cout<<endl;

            cout<<"\t-- Visualizing the Path --"<<endl;
            for(int i=0; i<ROWS; i++)
            {
                cout<<"     ";
                for(int j=0; j<COLS; j++)
                {
                    if(i == start->x && j == start->y)
                        cout<<"->";
                    else if (i == goal->x && j == goal->y)
                        cout<<"-|";
                    else if(Path[i][j] ==0)
                    {
                        cout<<"  ";
                    }
                    else if(Path[i][j] ==1)
                    {
                        cout<<"# ";
                    }
                    else if(Path[i][j] ==8)
                    {
                        cout<<"* ";
                    }
                    else
                    {
                        cout<<"  ";
                    }
                    
                }
                cout<<endl;
            }
        }
        
        


};

int main()
{
    BiRRT birrt = BiRRT();
    cout<<"\t-- Binary Matrix --"<<endl;
    birrt.printMatrix();
    cout<<endl;
    birrt.generateRRT(MAX_ITERATIONS);
    cout<<"\t-- Nodes Explored --"<<endl;
    birrt.printMatrix();
    // rrt.printNodes();
    cout<<"\t-- Path Achived --"<<endl;
    birrt.printPath();
    return 0;
}