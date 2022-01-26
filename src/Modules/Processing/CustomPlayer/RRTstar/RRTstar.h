#ifndef PROJECT_UNIFICATION_RRTSTAR_H
#define PROJECT_UNIFICATION_RRTSTAR_H

#include "obstacles.h"
#include "soccer-common/Extends/QPoint/ExtendsQPoint.h"
#include <vector>
#include <cmath>
using namespace std;

struct Node {
  vector<Node*> children;
  Node* parent;
  Point position;
  float orientation;
  double cost;
  // DubinsPath path;
};

class RRTSTAR {
 public:
  RRTSTAR();
  void initialize();
  Node* getRandomNode();
  Node* nearest(Point point);
  void near(Point point, float radius, vector<Node*>& out_nodes);
  double distance(Point& p, Point& q);
  double Cost(Node* q);
  double PathCost(Node* qFrom, Node* qTo);
  Point newConfig(Node* q, Node* qNearest);
  void add(Node* qNearest, Node* qNew);
  bool reached();
  void setStepSize(int step);
  void setMaxIterations(int iter);
  void setInitialPos(Point initialPos);
  void setFinalPos(Point finalPos);
  void deleteNodes(Node* root);
  Obstacles* obstacles;
  vector<Node*> nodes;
  vector<Node*> path;
  Node *root, *lastNode;
  Point startPos, endPos;
  int max_iter;
  int step_size;
};

#endif // PROJECT_UNIFICATION_RRTSTAR_H
