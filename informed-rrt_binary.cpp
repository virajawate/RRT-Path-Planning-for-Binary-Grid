#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <cstdio>
#include <memory>

using namespace std;

const int ROWS = 12;
const int COLS = 10;
const int OBSTACLE = 1;
const int FREE = 0;
const int STEP_SIZE = 1;
const double MAX_ITERATION = 50000;
const int REWIRE_RADIUS = 1;

struct Node
{
    int x, y;
    double costFromNeighbor, costFromRoot;
    Node* parent;
};

class InformedRRT
{
    private:
        vector<vector<int>> Matrix;
        vector<Node*> nodes;
        vector<Node*> path;
        vector<Node*> AOI;
        Node* start;
        Node* goal;
        int iteration;
        bool pathFound, goalFound;

        vector<Node*> AreaofInterest(Node* start, Node* goal)
        {
            
            int c = sqrt(pow(start->x - goal->x, 2) + pow(start->y - goal->y, 2)); //Distance between start and goal
            int a = c-1; //width of the area of interest
            int b = sqrt(pow(c,2) - pow(a,2)); // height of the area of interest
            // Centre of area of interest
            int x0 = 0.5*(start->x + goal->x); 
            int y0 = 0.5*(start->y + goal->y);
            double theta = atan2(abs(start->x - goal->x),abs(start->y - goal->y));
            cout<<start->x<<"|"<<start->y<<endl;
            cout<<goal->x<<"|"<<goal->y<<endl;
            cout<<"C = "<<c<<"|A = "<<a<<"|B = "<<b<<"|Theta = "<<theta<<endl;
            cout<<"Centre is "<<x0<<","<<y0<<endl;

            for(int i=0; i<ROWS; i++)
            {
                for(int j=0; j<COLS; j++)
                {
                    double a0 = ((i-x0)*cos(theta)) + ((j-y0)*sin(theta));
                    double b0 = -((i-x0)*sin(theta)) + ((j-y0)*cos(theta));                    
                    double check = pow(b0/b, 2) + pow(a0/a, 2);
                    cout<<check<<endl;
                    if(check<=1&&Matrix[i][j]==0)
                    {
                        Node* node = new Node();
                        node->x = (int)i;
                        node->y = (int)j;
                        node->parent = nullptr;
                        node->costFromNeighbor = 0;
                        node->costFromRoot = 0;
                        AOI.push_back(node);
                        // cout<<i<<"|"<<j<<"|"<<a0<<"|"<<b0<<"|"<<check<<endl;
                        // cout<<"FOund"<<endl;
                    }
                    // else
                    //     cout<<"GOingforNxt"<<endl;
                    
                }
            } 
            
            return AOI;  
        }

        Node* randomNode()
        {
            vector<Node*> EllipseNode = AreaofInterest(start,goal);

            int index = rand() % EllipseNode.size();

            return EllipseNode[index];
        }

        Node* nearestNode(Node* node)
        {
            Node* nearest = nodes[0];
            // cout<<nodes[0]->x<<"|"<<nodes[0]->y<<endl;
            float minDistance = distance(node, nearest);

            for(int i=0; i<nodes.size(); i++)
            {
                float d = distance(node, nodes[i]);
                if(d<minDistance){
                    nearest = nodes[i];
                    minDistance = d;
                }
            }
            // cout<<nearest->x<<"|"<<nearest->y<<endl;

            return nearest;
        }

        bool isFree(int x, int y){
            if (x >= 0 && x < COLS && y >= 0 && y < ROWS && Matrix[x][y] == OBSTACLE)
                return false;
            
            return Matrix[x][y] == FREE;
        }

        float distance(Node* n1, Node* n2){
            return sqrt(pow(n1->x-n2->x, 2) + pow(n1->y-n2->y, 2));
        }

        bool isPathCollisionFree(Node* n1, Node* n2)
        {
            if(n1==nullptr || n2==nullptr) return false;

            int dx = abs(n1->x - n2->x);
            int dy = abs(n1->y - n2->y);

            int step_x = n1->x < n2->x ? 1:-1;
            int step_y = n1->y < n2->y ? 1:-1;

            int delta = dx-dy;

            while(true)
            {
                int error = 2 * delta;

                if(error > -dy)
                {
                    delta -= dy;
                    n1->x += step_x;
                }

                if(error<dx){
                    delta += dx;
                    n1->y += step_y;
                }

                if(n1->x == n2->x && n1->y == n2->y)
                    break;
                
                if(!isFree(n1->x, n1->y))
                    return false;

            }
            return true;
        }

        Node* newNode(Node* nearest, Node* random)
        {
            Node* node = new Node();
            float d = distance(nearest, random);
            
            if(d<=STEP_SIZE)
            {
                node->x=random->x;
                node->y=random->y;
            }
            else{
                float theta = atan2(random->y - nearest->y, random->x - nearest->x);
                node->x = nearest->x + STEP_SIZE * cos(theta);
                node->y = nearest->y + STEP_SIZE * sin(theta);
            }
            
            node->parent = nearest;
            node->costFromNeighbor =  sqrt(pow(node->x - node->parent->x, 2) + pow(node->y-node->parent->y,2));
            node->costFromRoot = node->parent->costFromRoot+node->costFromNeighbor;
            
            return node;
        }

        void rewire_tree(Node* newNode, vector<Node*>& nodes)
        {
            for(Node* nearNode : nodes)
            {
                if(nearNode == newNode || nearNode->parent == nullptr){ continue;}

                double cost = newNode->costFromNeighbor;
                if(nearNode->costFromRoot + cost < newNode->costFromRoot)
                {
                    if(isPathCollisionFree(nearNode, newNode))
                    {
                        newNode->parent = nearNode;
                        newNode->costFromNeighbor =  sqrt(pow(newNode->x - newNode->parent->x, 2) + pow(newNode->y-newNode->parent->y,2));
                        newNode->costFromRoot = newNode->parent->costFromRoot+newNode->costFromNeighbor;
            
                    }
                }
            }
        }

        void findPath()
        {
            Node* current = goal;

            while(current != NULL && current != start)
            {
                if(current->x <=0 || current->y <=0 || current->x >= ROWS || current->y >= COLS)
                    break;

                current= current->parent;

                if(current != NULL)
                    Matrix[current->x][current->y] = 2; 
            }

            if(current == start)
                Matrix[current->x][current->y] = 2;
                
        }

        bool hasReachedGoal(Node* node){
            return distance(node, goal) <= STEP_SIZE;
        }


    public:
        InformedRRT(): Matrix(ROWS, vector<int>(COLS, FREE))
        {
            srand(time(NULL));

            Matrix = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},//0
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},//1
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},//2
                      {1, 0, 0, 0, 1, 1, 0, 0, 0, 1},//3
                      {1, 0, 0, 0, 1, 1, 0, 0, 0, 1},//4
                      {1, 0, 0, 0, 1, 1, 0, 0, 0, 1},//5
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},//6
                      {1, 0, 0, 0, 1, 1, 0, 0, 0, 1},//7
                      {1, 0, 0, 0, 1, 1, 0, 0, 0, 1},//8
                      {1, 0, 0, 0, 1, 1, 0, 0, 0, 1},//9
                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},//10
                      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};//11
                    /* 0  1  2  3  4  5  6  7  8  9 */
            
            start = new Node();
            start->x =1;
            start->y =1;
            start->parent =nullptr;
            start->costFromNeighbor =0;
            start->costFromRoot =0;

            goal = new Node();
            goal->x =7;
            goal->y =7;
            goal->parent =nullptr;
            goal->costFromRoot =0;
            goal->costFromNeighbor =0;

            nodes.push_back(start);
            iteration = 0;

            pathFound = false;
        }

        void generatePath(double max_iterations)
        {
            iteration =0;
            bool goalFound = false;
            // cout<<"Started Gen Path"<<endl;
            
            do
            {
                                
                /* code */
                // cout<<"Going for Random node"<<endl;
                Node* random = randomNode();
                // cout<<"Going for nearest node"<<endl;
                Node* nearest = nearestNode(random);
                // cout<<"Going for new node"<<endl;
                Node* node = newNode(nearest, random);
                // cout<<"All nodes Genrated"<<endl;
                if(isFree(node->x, node->y) && isPathCollisionFree(nearest, node))
                {
                    nodes.push_back(node);

                    // rewire_tree(node, nodes);

                    if(hasReachedGoal(node))
                    {
                        goalFound = true;
                        pathFound = true;
                        cout<<goalFound<<endl;
                        findPath();
                        break;
                    }

                    Matrix[node->x][node->y] = 3;

                    Node* current = node;

                    while(current !=start)
                    {
                        path.push_back(current);
                        current = current->parent;
                    }
                    path.push_back(start);

                    for(int i=path.size()-1; i>=1; i--)
                    {
                        Node* node1 = path[i];
                        Node* node2 = path[i-1];
                        if(isPathCollisionFree(node1, node2))
                        {
                            Matrix[node1->x][node1->y] = 2;
                            Matrix[node2->x][node2->y] = 2;
                        }
                        else continue;
                    }
                    iteration++;
                    // cout<<iteration<<"th Iteration complete."<<endl;

                    if(iteration >= max_iterations){
                        cout<<"ITERATOIN lIMIT"<<endl;
                        break;
                    }
                }
                // cout<<"I am done"<<endl;
            }while(!goalFound);
            cout<<"I am out man"<<endl;
        }

        int PrintMatrix()
        {
            cout<<"\t-- Binary Matrix --"<<endl;
            for(int i=0; i<ROWS ; i++)
            {
                cout<<"\t";
                for(int j=0; j<COLS; j++)
                    cout<<Matrix[i][j]<<" ";
                cout<<endl;
            }
            cout<<endl;
            return 0;
        }

        void PrintPath(bool visualize = true)
        {
            reverse(nodes.begin(), nodes.end());
            Node* current;
            current = nodes[0];
            
            vector<vector<int>> Path = Matrix;
            do{
                Path[current->x][current->y] = 8;
                current = current->parent;
            }while(current->parent != NULL);

            if(!visualize){
                cout<<"\t-- Path Found --"<<endl;
                for(int i=0; i<ROWS ; i++)
                {
                    cout<<"\t";
                    for(int j=0; j<COLS; j++)
                        cout<<Matrix[i][j]<<" ";
                    cout<<endl;
                }
                cout<<endl;                
            }
            else{
                cout<<"\t-- Path Visualizatoin --"<<endl;
                for(int i=0; i<ROWS ; i++)
                {
                    cout<<"\t";
                    for(int j=0; j<COLS; j++){
                        if(i == start->x && j == start->y)
                            cout<<"->";
                        else if(i == goal->x && j == goal->y)
                            cout<<"-|";
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
        }

        void PrintNode()
        {
            cout<<nodes.size()<<endl;
            cout<<path.size()<<endl;
            cout<<"\t-- Node Explored -- "<<endl;            
            for(int i=0; i<nodes.size(); i++)              
                cout<<"Node "<<i<<": ("<<nodes[i]->x<<","<<nodes[i]->y<<") \t Cost: "<<nodes[i]->costFromNeighbor<<"\t Root: "<<nodes[i]->costFromRoot<<"\t Parent : ("<<nodes[i]->parent->x<<","<<nodes[i]->parent->x<<")|"<<endl;   
        }

        void Ellipse()
        {
            vector<Node*> insideNode = AreaofInterest(start,goal);
            cout<<insideNode.size()<<endl;
            vector<vector<int>> Map = Matrix;
            for(auto nodes:insideNode)
            {
                for(int i=0; i<ROWS; i++)
                {
                    for(int j=0; j<COLS; j++)
                    {
                        if(i == start->x && j == start->y)                        
                            Map[i][j] = 7;
                        if(i == goal->x && j == goal->y)                        
                            Map[i][j] = 9;
                        if(nodes->x == i && nodes->y == j)
                            Map[i][j] = 2;
                    }
                }
            }

            for(int i=0; i<ROWS; i++)
            {
                cout<<"\t";
                for(int j=0; j<COLS; j++)
                    cout<<Map[i][j]<<" ";
                cout<<endl;
            }           
        }

        ~InformedRRT(){

            cout<<"\nEnding the Program"<<endl;
        }
};

int main()
{
    
    InformedRRT InRRTstar = InformedRRT();
    cout<<"Program start"<<endl;
    
    InRRTstar.Ellipse();

    // InRRTstar.PrintMatrix();
    // cout<<"Program Printed"<<endl;

    // InRRTstar.generatePath(MAX_ITERATION);
    // cout<<"PathFOund"<<endl;

    // InRRTstar.PrintMatrix();

    // InRRTstar.PrintPath();
    // InRRTstar.PrintNode();

    // InRRTstar.PrintPath(false);
    

    return 0;
}