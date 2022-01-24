#include "RRTstar.h"
#include <random>

const double EPS = 1e-6;
bool pathFound = false;

RRTSTAR::RRTSTAR() {
  obstacles = new Obstacles;
  startPos.setX(START_POS_X);
  startPos.setY(START_POS_Y);
  endPos.setX(END_POS_X);
  endPos.setY(END_POS_Y);
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->orientation = START_ORIENT;
  root->cost = 0.0;
  lastNode = root;
  nodes.push_back(root);
  step_size = 18;
  max_iter = 3000;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTSTAR::initialize() {
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->orientation = START_ORIENT;
  root->cost = 0.0;
  lastNode = root;
  nodes.push_back(root);
}

template <typename T> // Returns a random number in [low, high]
T randomCoordinate(T low, T high) {
  random_device random_device;
  mt19937 engine{random_device()};
  uniform_real_distribution<double> dist(low, high);
  return dist(engine);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTSTAR::getRandomNode() {
  Node* ret;
  double random_sample =
      randomCoordinate(0.0,
                       1.0); // Comentar daqui até a linha 58 para testar sem misturar os algoritmos
  Point point;
  if ((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) {
    point = Point(END_POS_X, END_POS_Y) + Point(RADIUS, RADIUS);
  } else {
    point = Point(randomCoordinate((float) 0.0, WORLD_WIDTH),
                  randomCoordinate((float) 0.0, WORLD_HEIGHT));
  }
  // Point point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT); // Descomentar essa parte para
  // testar sem misturar os algoritmos
  float orient = drand48() * 2 * 3.142;
  if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT &&
      orient > 0 && orient < 2 * 3.142) {
    ret = new Node;
    ret->position = point;
    ret->orientation = orient;
    return ret;
  }
  return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTSTAR::distance(Point& p, Point& q) {
  Point v = p - q;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTSTAR::nearest(Point point) {
  float minDist = 1e9;
  Node* closest = NULL;
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTSTAR::near(Point point, float radius, vector<Node*>& out_nodes) {
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < radius) {
      out_nodes.push_back(nodes[i]);
    }
  }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Point RRTSTAR::newConfig(Node* q, Node* qNearest) {
  Point to = q->position;
  Point from = qNearest->position;
  Point intermediate = to - from;
  intermediate = intermediate / intermediate.norm();
  Point pos = from + step_size * intermediate;
  Point ret(pos.x(), pos.y());
  return ret;
}

/**
 * @brief Return trajectory cost.
 * @param q
 * @return
 */
double RRTSTAR::Cost(Node* q) {
  return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTSTAR::PathCost(Node* qFrom, Node* qTo) {
  return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node* qNearest, Node* qNew) {
  qNew->parent = qNearest;
  qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
  qNearest->children.push_back(qNew);
  nodes.push_back(qNew);
  lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRTSTAR::reached() {
  if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD) {
    pathFound = true;
    return true;
  }
  return false;
}

void RRTSTAR::setStepSize(int step) {
  step_size = step;
}

void RRTSTAR::setMaxIterations(int iter) {
  max_iter = iter;
}

// void RRTSTAR::setStartPos(Point startPosition) {
//   START_POS_X = (float) startPosition.x();
//   START_POS_Y = (float) startPosition.y();
// }

// void RRTSTAR::setEndPos(Point endPosition) {
//   END_POS_X = (float) endPosition.x();
//   END_POS_Y = (float) endPosition.y();
// }

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRTSTAR::deleteNodes(Node* root0) {
  for (int i = 0; i < (int) root0->children.size(); i++) {
    deleteNodes(root0->children[i]);
  }
  delete root0;
}

// #include <cmath>
// #include <math.h>
// #include <random>
// #include <vector>
// #include "../CustomPlayer.h"
// #include "RRTstar.h"

// const double FIELD_LENGTH = 9000; // WIDTH
// const double FIELD_WIDTH = 6000;  // HEIGHT
// // const double GOAL_DEPTH = 180;
// // const double GOAL_WIDTH = 1000;
// // const double PENALTY_AREA_DEPTH = 1000;
// // const double PENALTY_AREA_WIDTH = 2000;
// // const double BOUNDARY_WIDTH = 300;
// // const double GOAL_CENTER_TO_PENALTY_MARK = 6000;

// const double RADIUS = 50;
// const double GOAL_SAMPLING_PROB = 0.05;
// const double INF = 1e18;

// const double JUMP_SIZE = (FIELD_LENGTH / 100.0 * FIELD_WIDTH / 100.0) / 1.5;
// const double DISK_SIZE = JUMP_SIZE; // Ball radius around which nearby points are found

// QList<Robot> obstaclesRobot;
// vector<QVector<Point>> obstacles;
// Point start, stop;
// int obstacle_cnt = 1;
// double robotRadius = 90;

// QList<Point> nodes;
// QList<int> parent, nearby;
// QList<double> cost, jumps;
// int nodeCnt = 0, goalIndex = -1;

// vector<QVector<Point>> polygons;
// Point startingPoint, endingPoint;
// bool pathFound = 0;

// void getInput(Point robotPosition,
//               Point endPoint,
//               int obstacle_count,
//               const Robots<Robot>& allies,
//               const Robots<Robot>& enemies) {
//   start = robotPosition;
//   stop = endPoint;
//   obstacle_cnt = obstacle_count;

//   // obstaclesRobot.resize(obstacle_cnt);
//   // obstacles.resize(obstacle_cnt);
//   for (const Robot& allyRobot : allies) {
//     obstaclesRobot.append(allyRobot);
//   }
//   for (const Robot& enemyRobot : enemies) {
//     obstaclesRobot.append(enemyRobot);
//   }
//   int pnts = 0;
//   Point pntA, pntB, pntC, pntD;
//   QVector<Point> poly;

//   for (int i = 0; i < obstacle_cnt; i++) {
//     // poly.clear();
//     // pnts = 4;
//     // poly.resize(pnts);

//     pntA.setX(obstaclesRobot.at(i).position().x() + robotRadius); // Ponto superior direito
//     pntA.setY(obstaclesRobot.at(i).position().y() + robotRadius);
//     obstacles[i].append(pntA);

//     pntB.setX(obstaclesRobot.at(i).position().x() - robotRadius); // Ponto superior esquerdo
//     pntB.setY(obstaclesRobot.at(i).position().y() + robotRadius);
//     obstacles[i].append(pntB);

//     pntC.setX(obstaclesRobot.at(i).position().x() + robotRadius); // Ponto inferior direito
//     pntC.setY(obstaclesRobot.at(i).position().y() - robotRadius);
//     obstacles[i].append(pntC);

//     pntD.setX(obstaclesRobot.at(i).position().x() - robotRadius); // Ponto inferior esquerdo
//     pntD.setY(obstaclesRobot.at(i).position().y() - robotRadius);
//     obstacles[i].append(pntD);
//     // Os polígonos dos obstáculos vão ser os QVectors de obstacles

//     // for (int j = 0; j < pnts; j++) {
//     //   pnt.setX(obstaclesRobot.at(i).position().x() + 90);
//     //   pnt.setY(obstaclesRobot.at(i).position().y() + 90);
//     //   obstacles[i].append(pnt);
//     // }
//   }
// }

// void prepareInput() {
//   startingPoint.setX(start.x());
//   startingPoint.setY(start.y());
//   endingPoint.setX(stop.x());
//   endingPoint.setY(stop.y());

//   polygons.resize(obstacle_cnt);
//   polygons.assign(obstacles.begin(), obstacles.end());
//   // for (int i = 0; i < obstacle_cnt; i++) {
//   // }
// }

// template <class T> // Retorna um número aleatório dentro do intervalo [low, high]
// T randomCoordinate(T low, T high) {
//   random_device random_device;
//   mt19937 engine{random_device()};
//   uniform_real_distribution<double> dist(low, high);
//   return dist(engine);
// }

// bool isEdgeObstacleFree(Point a, Point b) {
//   for (auto& poly : obstacles) {
//     if (lineSegmentIntersectPolygon(a, b, poly)) {
//       return false;
//     }
//   }
//   return true;
// }

// Point pickRandomPoint() {
//   double random_sample = randomCoordinate(0.0, 1.0);
//   if ((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) {
//     return stop + Point(RADIUS, RADIUS);
//   }
//   return Point(randomCoordinate<double>(0.0, FIELD_LENGTH),
//                randomCoordinate<double>(0.0, FIELD_WIDTH));
// }

// void checkDestinationReached() {
//   Point position;
//   position.setX(endingPoint.x());
//   position.setY(endingPoint.y());
//   if (checkCollision(nodes.at(parent.at(nodeCnt - 1)),
//                      nodes.back(),
//                      Point(position.x(), position.y()),
//                      RADIUS)) {
//     pathFound = 1;
//     goalIndex = nodeCnt - 1;
//   }
// }

// Point steer(const Point& thisPoint, const Point& t, double DELTA) {
//   if (abs(thisPoint.distTo(t)) < DELTA) {
//     return t;
//   } else {
//     double theta = atan2(t.y() - thisPoint.y(), t.x() - thisPoint.x());
//     return Point(thisPoint.x() + DELTA * cos(theta), thisPoint.y() + DELTA * sin(theta));
//   }
// }

// void insertNodesInPath(int rootIndex, Point& q) {
//   Point p = nodes.at(rootIndex);
//   if (!isEdgeObstacleFree(p, q)) {
//     return;
//   }
//   while (!(p == q)) {
//     Point nxt = steer(p, q, JUMP_SIZE);
//     nodes.push_back(nxt);
//     parent.push_back(rootIndex);
//     cost.push_back(cost.at(rootIndex) + abs(p.distTo(nxt)));
//     rootIndex = nodeCnt++;
//     p = nxt;
//   }
// }

// void rewire() {
//   int lastInserted = nodeCnt - 1;
//   for (auto nodeIndex : nearby) {
//     int par = lastInserted, cur = nodeIndex;

//     while (((cost.at(par) + abs(nodes.at(par).distTo(nodes.at(cur)))) - cost.at(cur)) <= EPS) {
//       int oldParent = parent.at(cur);
//       parent[cur] = par;
//       cost[cur] = cost[par] + abs(nodes[par].distTo(nodes[cur]));
//       par = cur;
//       cur = oldParent;
//     }
//   }
// }

// void RRT() {
//   Point newPoint, nearestPoint, nextPoint;
//   bool updated = false;
//   int cnt = 0;
//   int nearestIndex = 0;
//   double minCost = INF;
//   nearby.clear();
//   jumps.resize(nodeCnt);

//   while (!updated) {
//     newPoint = pickRandomPoint();

//     nearestPoint = *nodes.begin();
//     nearestIndex = 0;
//     for (int i = 0; i < nodeCnt; i++) {
//       if (pathFound and randomCoordinate(0.0, 1.0) < 0.25) {
//         cost[i] = cost[parent[i]] + abs(nodes[parent[i]].distTo(nodes[i]));
//       }

//       jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE;
//       auto pnt = nodes[i];
//       if ((abs(pnt.distTo(newPoint)) - abs(nearestPoint.distTo(newPoint))) <= EPS and
//           isEdgeObstacleFree(pnt, steer(pnt, newPoint, jumps[i]))) {
//         nearestPoint = pnt;
//         nearestIndex = i;
//       }
//     }
//     nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
//     if (!isEdgeObstacleFree(nearestPoint, nextPoint)) {
//       continue;
//     }

//     for (int i = 0; i < nodeCnt; i++) {
//       if ((abs(nodes[i].distTo(nextPoint)) - DISK_SIZE) <= EPS and
//           isEdgeObstacleFree(nodes[i], nextPoint)) {
//         nearby.push_back(i);
//       }
//     }

//     int par = nearestIndex;
//     minCost = cost[par] + abs(nodes[par].distTo(nextPoint));
//     for (auto nodeIndex : nearby) {
//       if (((cost[nodeIndex] + abs(nodes[nodeIndex].distTo(nextPoint))) - minCost) <= EPS) {
//         minCost = cost[nodeIndex] + abs(nodes[nodeIndex].distTo(nextPoint));
//         par = nodeIndex;
//       }
//     }

//     parent.push_back(par);
//     cost.push_back(minCost);
//     nodes.push_back(nextPoint);
//     nodeCnt++;
//     updated = true;
//     if (!pathFound) {
//       checkDestinationReached();
//     }
//     rewire();
//   }
// }