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
  // Point newDubinConfig(Node *q, Node *qNearest, DubinsPath &path);
  void add(Node* qNearest, Node* qNew);
  bool reached();
  void setStepSize(int step);
  void setMaxIterations(int iter);
  // void setStartPos(Point startPosition);
  // void setEndPos(Point endPosition);
  void deleteNodes(Node* root);
  Obstacles* obstacles;
  vector<Node*> nodes;
  vector<Node*> path;
  Node *root, *lastNode;
  Point startPos, endPos;
  int max_iter;
  int step_size;
};

// #include <cmath>
// // #include "../CustomPlayer.h"
// #include "Entities/Entities.h"
// #include "soccer-common/Extends/QPoint/ExtendsQPoint.h"
// #include <iostream>
// #include <qvectornd.h>
// #include <random>
// #include <type_traits>
// #include <vector>
// using namespace std;

// #define ftype double

// const ftype EPS = 1e-5;

// struct structPoint {
//   ftype x, y;
//   structPoint() {
//   }
//   structPoint(ftype x, ftype y) : x(x), y(y) {
//   }
//   structPoint& operator+=(const structPoint& t) {
//     x += t.x;
//     y += t.y;
//     return *this;
//   }
//   structPoint& operator-=(const structPoint& t) {
//     x -= t.x;
//     y -= t.y;
//     return *this;
//   }
//   structPoint& operator*=(ftype t) {
//     x *= t;
//     y *= t;
//     return *this;
//   }
//   structPoint& operator/=(ftype t) {
//     x /= t;
//     y /= t;
//     return *this;
//   }
//   structPoint operator+(const structPoint& t) const {
//     return structPoint(*this) += t;
//   }
//   structPoint operator-(const structPoint& t) const {
//     return structPoint(*this) -= t;
//   }
//   structPoint operator*(ftype t) const {
//     return structPoint(*this) *= t;
//   }
//   structPoint operator/(ftype t) const {
//     return structPoint(*this) /= t;
//   }
//   ftype dot(const structPoint& t) const {
//     return (x * t.x + y * t.y);
//   }
//   ftype cross(const structPoint& t) const {
//     return x * t.y - y * t.x;
//   }
//   ftype cross(const structPoint& a, const structPoint& b) const {
//     return (a - *this).cross(b - *this);
//   }
//   ftype distance(const structPoint& t) const {
//     const double x_diff = x - t.x, y_diff = y - t.y;
//     return sqrt(x_diff * x_diff + y_diff * y_diff);
//   }
//   structPoint steer(const structPoint& t, ftype DELTA) {
//     if (this->distance(t) < DELTA) {
//       return t;
//     } else {
//       double theta = atan2(t.y - y, t.x - x);
//       return structPoint(x + DELTA * cos(theta), y + DELTA * sin(theta));
//     }
//   }
//   bool operator==(const structPoint& rhs) const {
//     return fabs(x - rhs.x) < EPS and fabs(y - rhs.y) < EPS; // or another approach as above
//   }
// };

// structPoint operator*(ftype a, structPoint b) {
//   return b * a;
// }

// ftype distance(structPoint& a, structPoint& b) {
//   const ftype x_diff = a.x - b.x, y_diff = a.y - b.y;
//   return sqrt(x_diff * x_diff + y_diff * y_diff);
// }

// ftype dot(structPoint a, structPoint b) {
//   return (a.x * b.x + a.y * b.y);
// }

// ftype cross(structPoint a, structPoint b) {
//   return (a.x * b.y - b.x * a.y);
// }

// /*  Returns a point in the direction of (p2 - p1) vector such that
//     the new point is within a DELTA distance of point1  */
// structPoint stepNear(structPoint& p1, structPoint& p2, ftype DELTA) {
//   if ((distance(p1, p2) - DELTA) <= EPS)
//     return p2;
//   else {
//     ftype theta = atan2(p2.y - p1.y, p2.x - p1.x);
//     return structPoint(p1.x + DELTA * cos(theta), p1.y + DELTA * sin(theta));
//   }
// }

// // Return minimum distance between line segment vw and point p
// ftype minimum_distance(structPoint v, structPoint w, structPoint p) {
//   ftype l2 = distance(v, w);
//   l2 *= l2; // i.e. |w-v|^2 -  avoid a sqrt
//   if (l2 < EPS)
//     return distance(p, v); // v == w case

//   // Consider the line extending the segment, parameterized as v + t (w - v).
//   // We find projection of point p onto the line.
//   // It falls where t = [(p-v) . (w-v)] / |w-v|^2
//   // We clamp t from [0,1] to handle points outside the segment vw.
//   const ftype t = max(0.0, min(1.0, dot(p - v, w - v) / l2));

//   structPoint projection = v + t * (w - v); // Projection falls on the segment
//   return distance(p, projection);
// }
// double robotRadius = 90;
// const float BOT_CLEARANCE = 1.5 * robotRadius;
// struct structPolygon {
//   pair<structPoint, structPoint> points; // Assumes clockwise/anti-clockwise points input
//   int pointCnt = 0;

//   void addPoint(structPoint firstPoint, structPoint secondPoint) {
//     // points.push_back(pnt);
//     Point firstPnt = Point(firstPoint.x, firstPoint.y);
//     Point secondPnt = Point(secondPoint.x, secondPoint.y);
//     pointCnt++;
//     // Get topLeft and bottomRight points from the given points.
//     Point tmp;
//     if (firstPnt.x() > secondPnt.x() && firstPnt.y() > secondPnt.y()) {
//       tmp = firstPnt;
//       firstPnt = secondPnt;
//       secondPnt = tmp;
//     } else if (firstPnt.x() < secondPnt.x() && firstPnt.y() > secondPnt.y()) {
//       int height = firstPnt.y() - secondPnt.y();
//       // firstPnt.y() -= height;
//       firstPnt.setY(firstPnt.y() - height);
//       // secondPnt.y() += height;
//       secondPnt.setY(secondPnt.y() + height);
//     } else if (firstPnt.x() > secondPnt.x() && firstPnt.y() < secondPnt.y()) {
//       int length = firstPnt.x() - secondPnt.x();
//       // firstPnt.x() -= length;
//       firstPnt.setX(firstPnt.x() - length);
//       // secondPnt.x() += length;
//       secondPnt.setX(secondPnt.x() + length);
//     }
//     firstPnt.setX(firstPnt.x() - BOT_CLEARANCE);
//     firstPnt.setY(firstPnt.y() - BOT_CLEARANCE);
//     secondPnt.setX(secondPnt.x() + BOT_CLEARANCE);
//     secondPnt.setY(secondPnt.y() + BOT_CLEARANCE);
//     // firstPoint.x() -= BOT_CLEARANCE;
//     // firstPoint.y() -= BOT_CLEARANCE;
//     // secondPoint.x() += BOT_CLEARANCE;
//     // secondPoint.y() += BOT_CLEARANCE;

//     points = make_pair(firstPoint, secondPoint);
//     // obstacles.push_back(make_pair(firstPoint, secondPoint));
//   }
//   pair<structPoint, structPoint> getPoints() {
//     return points;
//   }
//   // bool pointInside(const structPoint point) { // Can be done in log(N)
//   //   int i, j, nvert = points.size();
//   //   bool c = false;
//   //   for (i = 0, j = nvert - 1; i < nvert; j = i++) {
//   //     if (((points[i].y >= point.y) != (points[j].y >= point.y)) &&
//   //         (point.x <=
//   //          (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y)
//   +
//   //              points[i].x))
//   //       c = !c;
//   //   }
//   //   return c;
//   // }
// };

// /*  Return true if the given line segment intersects the circle whose center
//     is at location */
// bool checkCollision(structPoint lineFrom, structPoint lineTo, structPoint location, ftype radius)
// {
//   location +=
//       structPoint(radius, radius); // Adjust location from top-left corner to center coordinates
//   ftype ab2, acab, h2;
//   structPoint ac = location - lineFrom;
//   structPoint ab = lineTo - lineFrom;
//   ab2 = dot(ab, ab);
//   acab = dot(ac, ab);
//   ftype t = acab / ab2;

//   if (t < 0)
//     t = 0;
//   else if (t > 1)
//     t = 1;

//   structPoint h = ((ab * t) + lineFrom) - location;
//   h2 = dot(h, h);
//   return (h2 <= (radius * radius));
// }

// // taken from stackoverflow:
// // https://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
// // this can be done in log(N) though
// bool PointInPolygon(structPoint point, structPolygon polygon) {
//   pair<structPoint, structPoint> points = polygon.getPoints();
//   structPoint pnt = point;
//   // int i, j, nvert = points.size();
//   bool c = false;

//   // for (i = 0, j = nvert - 1; i < nvert; j = i++) {
//   //   if (((points[i].y >= point.y) != (points[j].y >= point.y)) &&
//   //       (point.x <=
//   //        (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) +
//   //            points[i].x))
//   //     c = !c;
//   // }
//   return c;
// }

// // helper function
// int sign(const ftype x) {
//   return x >= 0 ? x ? 1 : 0 : -1;
// }

// /*  Returns true if two line segments on the same line intersect.
//     (a, b) denote the endpoints of first line segment and
//     (c, d) denotes the endpoints of the second lint segment */
// bool intersectOnLine(ftype a, ftype b, ftype c, ftype d) {
//   if ((a - b) > EPS)
//     swap(a, b);
//   if ((c - d) > EPS)
//     swap(c, d);
//   return max(a, c) <= min(b, d);
// }

// // Returns true if the two line segments (a, b) and (c, d) intersect
// bool check_intersection(const structPoint a,
//                         const structPoint b,
//                         const structPoint c,
//                         const structPoint d) {
//   // Check if both line segments lie on the same line
//   if (c.cross(a, d) == 0 && c.cross(b, d) == 0)
//     return intersectOnLine(a.x, b.x, c.x, d.x) && intersectOnLine(a.y, b.y, c.y, d.y);

//   // Check if a and b both lie on different side of line segment CD
//   // Similarly check for c and d to lie on different side of line segment AC
//   return sign(a.cross(b, c)) != sign(a.cross(b, d)) && sign(c.cross(d, a)) != sign(c.cross(d,
//   b));
// }

// /*  Returns true if the given line segment represented by ba intersects with any
//     side of the polygon */
// bool lineSegmentIntersectsPolygon(structPoint a, structPoint b, structPolygon& polygon) {
//   Point p1 = Point(a.x, a.y);
//   Point p2 = Point(b.x, b.y);
//   QLineF lineSegment(p1.x(), p1.y(), p2.x(), p2.y());
//   QPointF* intersectPt = new QPointF;

//   float length = polygon.points.second.x - polygon.points.first.x;
//   float breadth = polygon.points.second.y - polygon.points.first.y;
//   QLineF lseg1(polygon.points.first.x,
//                polygon.points.first.y,
//                polygon.points.first.x + length,
//                polygon.points.first.y);
//   QLineF lseg2(polygon.points.first.x,
//                polygon.points.first.y,
//                polygon.points.first.x,
//                polygon.points.first.y + breadth);
//   QLineF lseg3(polygon.points.second.x,
//                polygon.points.second.y,
//                polygon.points.second.x,
//                polygon.points.second.y - breadth);
//   QLineF lseg4(polygon.points.second.x,
//                polygon.points.second.y,
//                polygon.points.second.x - length,
//                polygon.points.second.y);
//   QLineF::IntersectType x1 = lineSegment.intersects(lseg1, intersectPt);
//   QLineF::IntersectType x2 = lineSegment.intersects(lseg2, intersectPt);
//   QLineF::IntersectType x3 = lineSegment.intersects(lseg3, intersectPt);
//   QLineF::IntersectType x4 = lineSegment.intersects(lseg4, intersectPt);
//   // check for bounded intersection. IntersectType for bounded intersection is 1.
//   if (x1 == 1 || x2 == 1 || x3 == 1 || x4 == 1) {
//     return true;
//   }

//   return false;
//   // // PQ is merely a point not enough distance for it be line segment
//   // if (a.distance(b) < EPS)
//   //   return PointInPolygon((a + b) / 2.0, polygon);

//   // int num = polygon.pointCnt;
//   // vector<structPoint> points = polygon.getPoints();
//   // for (int i = 0; i < num; i++) {
//   //   int nxt = i + 1;
//   //   if (nxt == num)
//   //     nxt = 0;
//   //   if (check_intersection(a, b, points[i], points[nxt]))
//   //     return true;
//   // }
//   // return false;
// }

// // ftype minimumDistance(structPoint v, structPoint w, structPoint p) {
// //   ftype l2 = v.distTo(w);
// //   l2 *= l2;
// //   if (l2 < EPS) {
// //     return p.distTo(v);
// //   }
// //   const ftype t = max(0.0, min(1.0, Geometry2D::dot(p - v, w - v)));

// //   structPoint projection = v + t * (w - v);
// //   return p.distTo(projection);
// // }

// // bool checkCollision(structPoint lineFrom, structPoint lineTo, structPoint location, ftype
// // radius)
// // {
// //   location += structPoint(radius, radius);
// //   ftype ab2, acab, h2;
// //   structPoint ac = location - lineFrom;
// //   structPoint ab = lineTo - lineFrom;
// //   ab2 = Geometry2D::dot(ab, ab);
// //   acab = Geometry2D::dot(ac, ab);
// //   ftype t = acab / ab2;

// //   if (t < 0) {
// //     t = 0;
// //   } else if (t > 1) {
// //     t = 1;
// //   }

// //   structPoint h = ((ab * t) + lineFrom) - location;
// //   h2 = Geometry2D::dot(h, h);
// //   return (h2 <= (radius * radius));
// // }

// // bool pointInPolygon(Point point, const QVector<Point>& polygon) {
// //   return Geometry2D::pointInPolygon(polygon, point);
// // }

// // int sign(const ftype x) {
// //   return x >= 0 ? x ? 1 : 0 : -1;
// // }

// // bool intersectOnLine(ftype a, ftype b, ftype c, ftype d) {
// //   if ((a - b) > EPS) {
// //     swap(a, b);
// //   }
// //   if ((c - d) > EPS) {
// //     swap(c, d);
// //   }
// //   return max(a, c) <= min(b, d);
// // }

// // bool checkIntersection(const Point a, const Point b, const Point c, const Point d) {
// //   if ((a - c).cross(d - c) == 0 && (b - c).cross(d - c) == 0) {
// //     return intersectOnLine(a.x(), b.x(), c.x(), d.x()) &&
// //            intersectOnLine(a.y(), b.y(), c.y(), d.y());
// //   }
// //   cout << "checkIntersection return sign: "
// //        << (sign((b - a).cross(c - a)) != sign((b - a).cross(d - a)) &&
// //            sign((d - c).cross(a - c) != sign((d - c).cross(b - c))))
// //        << endl;
// //   return sign((b - a).cross(c - a)) != sign((b - a).cross(d - a)) &&
// //          sign((d - c).cross(a - c) != sign((d - c).cross(b - c)));
// // }

// // template <class PT>
// // bool lineSegmentIntersectPolygon(Point a, Point b, const QVector<PT>& polygon) {
// //   if (a.distTo(b) < EPS) {
// //     return pointInPolygon((a + b) / 2.0, polygon);
// //   }

// //   int num = polygon.size();
// //   QList<Point> points = polygon.toList();
// //   for (int i = 0; i < num; i++) {
// //     int next = i + 1;
// //     if (next == num) {
// //       next = 0;
// //     }
// //     Point e = points.at(i);
// //     Point f = points.at(next);
// //     if (checkIntersection(a, b, e, f)) {
// //       return true;
// //     }
// //   }
// //   return false;
// // }

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

// vector<structPolygon> obstacles;
// structPoint start, stop;
// int obstacle_cnt = 1;

// QList<Point> obstaclesRobot = QList<Point>();
// // vector<QVector<Point>> obstacles = vector<QVector<Point>>();
// // Point start = Point(0.0, 0.0);
// // Point stop = Point(0.0, 0.0);
// // int obstacle_cnt = 1;
// // double robotRadius = 90;

// vector<structPoint> nodes;
// vector<int> parents, nearby;
// vector<double> cost, jumps;
// int nodeCnt = 0, goalIndex = -1;

// // QList<Point> nodes;
// // QList<int> parents, nearby;
// // QList<double> cost, jumps;
// // int nodeCnt = 0, goalIndex = -1;

// vector<structPolygon> polygons;
// structPoint startingPoint, endingPoint;
// bool pathFound = false;

// // vector<QVector<Point>> polygons;
// // Point startingPoint, endingPoint;
// // bool pathFound = false;

// void getInput(Point robotPosition,
//               Point endPoint,
//               int obstacle_count,
//               const Robots<Robot>& allies,
//               const Robots<Robot>& enemies) {
//   start.x = robotPosition.x();
//   start.y = robotPosition.y();
//   stop.x = endPoint.x();
//   stop.y = endPoint.y();
//   obstacle_cnt = obstacle_count;

//   obstaclesRobot.clear();
//   // obstacles.clear();
//   // // obstaclesRobot.resize(obstacle_cnt);
//   obstacles.resize(obstacle_cnt);
//   int pnts = 0;
//   structPoint pntA, pntB, pntC, pntD;
//   vector<structPoint> poly;
//   for (int i = 0; i < allies.size();
//        i++) { // Aqui aparece o erro "Falha de segmentação (imagem do núcleo gravada)"
//     obstaclesRobot.append(allies.at(i).position());
//   }
//   for (int j = 0; j < enemies.size(); j++) {
//     obstaclesRobot.append(enemies.at(j).position());
//   }
//   // int pnts = 0;
//   // Point pntA = Point(0.0, 0.0);
//   // Point pntB = Point(0.0, 0.0);
//   // Point pntC = Point(0.0, 0.0);
//   // Point pntD = Point(0.0, 0.0);

//   for (int m = 0; m < obstacle_cnt; m++) {
//     // poly.clear();
//     // pnts = 4;
//     // poly.resize(pnts);

//     // pntA.x = obstaclesRobot.at(m).x() + robotRadius;
//     // pntA.y = obstaclesRobot.at(m).y() + robotRadius;
//     // obstacles[m].addPoint(pntA);

//     pntB.x = obstaclesRobot.at(m).x() - robotRadius;
//     pntB.y = obstaclesRobot.at(m).y() + robotRadius;
//     // obstacles[m].addPoint(pntB);

//     pntC.x = obstaclesRobot.at(m).x() + robotRadius;
//     pntC.y = obstaclesRobot.at(m).y() - robotRadius;
//     obstacles[m].addPoint(pntB, pntC);

//     // pntD.x = obstaclesRobot.at(m).x() - robotRadius;
//     // pntD.y = obstaclesRobot.at(m).y() - robotRadius;
//     // obstacles[m].addPoint(pntD);

//     // QVector<Point> pntABCD = QVector<Point>();

//     // pntA.setX(obstaclesRobot.at(m).x() + robotRadius); // Ponto superior direito
//     // pntA.setY(obstaclesRobot.at(m).y() + robotRadius);
//     // pntABCD.append(pntA);

//     // pntB.setX(obstaclesRobot.at(m).x() - robotRadius); // Ponto superior esquerdo
//     // pntB.setY(obstaclesRobot.at(m).y() + robotRadius);
//     // pntABCD.append(pntB);

//     // pntC.setX(obstaclesRobot.at(m).x() + robotRadius); // Ponto inferior direito
//     // pntC.setY(obstaclesRobot.at(m).y() - robotRadius);
//     // pntABCD.append(pntC);

//     // pntD.setX(obstaclesRobot.at(m).x() - robotRadius); // Ponto inferior esquerdo
//     // pntD.setY(obstaclesRobot.at(m).y() - robotRadius);
//     // pntABCD.append(pntD);
//     // obstacles.push_back(pntABCD);
//     // Os polígonos dos obstáculos vão ser os QVectors de obstacles

//     // for (int j = 0; j < pnts; j++) {
//     //   pnt.setX(obstaclesRobot.at(i).position().x() + 90);
//     //   pnt.setY(obstaclesRobot.at(i).position().y() + 90);
//     //   obstacles[i].append(pnt);
//     // }
//   }
// }

// void prepareInput() {
//   // cout << "prepareInput()\n";
//   startingPoint.x = start.x;
//   startingPoint.y = start.y;
//   endingPoint.x = stop.x;
//   endingPoint.y = stop.y;
//   // startingPoint.setX(start.x());
//   // startingPoint.setY(start.y());
//   // endingPoint.setX(stop.x());
//   // endingPoint.setY(stop.y());

//   polygons.resize(obstacle_cnt);
//   polygons.assign(obstacles.begin(), obstacles.end());
//   // for (int i = 0; i < obstacle_cnt; i++) {
//   // }
// }

// template <typename T> // Returns a random number in [low, high]
// T randomCoordinate(T low, T high) {
//   random_device random_device;
//   mt19937 engine{random_device()};
//   uniform_real_distribution<double> dist(low, high);
//   return dist(engine);
// }

// // template <class T> // Retorna um número aleatório dentro do intervalo [low, high]
// // T randomCoordinate(T low, T high) {
// //   random_device random_device;
// //   mt19937 engine{random_device()};
// //   uniform_real_distribution<double> dist(low, high);
// //   return dist(engine);
// // }

// // Returns true if the line segment ab is obstacle free
// bool isEdgeObstacleFree(structPoint a, structPoint b) {
//   for (auto& poly : obstacles)
//     if (lineSegmentIntersectsPolygon(a, b, poly))
//       return false;
//   return true;
// }

// // bool isEdgeObstacleFree(Point a, Point b) {
// //   for (auto& poly : obstacles) {
// //     if (lineSegmentIntersectPolygon(a, b, poly)) {
// //       return false;
// //     }
// //   }
// //   return true;
// // }

// // Returns a random point with some bias towards goal
// structPoint pickRandomPoint() {
//   double random_sample = randomCoordinate(0.0, 1.0);
//   if ((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound)
//     return stop + structPoint(RADIUS, RADIUS);
//   return structPoint(randomCoordinate(0.0, FIELD_LENGTH), randomCoordinate(0.0, FIELD_WIDTH));
// }

// // Point pickRandomPoint() {
// //   double random_sample = randomCoordinate(0.0, 1.0);
// //   if ((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) {
// //     return stop + Point(RADIUS, RADIUS);
// //   }
// //   return Point(randomCoordinate<double>(0.0, FIELD_LENGTH),
// //                randomCoordinate<double>(0.0, FIELD_WIDTH));
// // }

// void checkDestinationReached() {
//   structPoint position = structPoint(endingPoint.x, endingPoint.y);
//   // sf::Vector2f position = endingPoint.getPosition();
//   if (checkCollision(nodes[parents[nodeCnt - 1]],
//                      nodes.back(),
//                      structPoint(position.x, position.y),
//                      RADIUS)) {
//     pathFound = 1;
//     goalIndex = nodeCnt - 1;
//     cout << "Reached!! With a distance of " << cost.back() << " units. " << endl << endl;
//   }
// }

// // void checkDestinationReached() {
// //   Point position;
// //   position.setX(endingPoint.x());
// //   position.setY(endingPoint.y());
// //   if (checkCollision(nodes.at(parents.at(nodeCnt - 1)),
// //                      nodes.back(),
// //                      Point(position.x(), position.y()),
// //                      RADIUS)) {
// //     pathFound = true;
// //     goalIndex = nodeCnt - 1;
// //   }
// // }

// // Point steer(const Point& thisPoint, const Point& t, double DELTA) {
// //   if (abs(thisPoint.distTo(t)) < DELTA) {
// //     return t;
// //   } else {
// //     double theta = atan2(t.y() - thisPoint.y(), t.x() - thisPoint.x());
// //     return Point(thisPoint.x() + DELTA * cos(theta), thisPoint.y() + DELTA * sin(theta));
// //   }
// // }

// /* Inserts nodes on the path from rootIndex till Point q such
//    that successive nodes on the path are not more than
//    JUMP_SIZE distance away */
// void insertNodesInPath(int rootIndex, structPoint& q) {
//   structPoint p = nodes[rootIndex];
//   if (!isEdgeObstacleFree(p, q))
//     return;
//   while (!(p == q)) {
//     structPoint nxt = p.steer(q, JUMP_SIZE);
//     nodes.push_back(nxt);
//     parents.push_back(rootIndex);
//     cost.push_back(cost[rootIndex] + distance(p, nxt));
//     rootIndex = nodeCnt++;
//     p = nxt;
//   }
// }

// // void insertNodesInPath(int rootIndex, Point& q) {
// //   Point p = nodes.at(rootIndex);
// //   if (!isEdgeObstacleFree(p, q)) {
// //     return;
// //   }
// //   while (!(p == q)) {
// //     Point nxt = steer(p, q, JUMP_SIZE);
// //     nodes.push_back(nxt);
// //     parents.push_back(rootIndex);
// //     cost.push_back(cost.at(rootIndex) + abs(p.distTo(nxt)));
// //     rootIndex = nodeCnt++;
// //     p = nxt;
// //   }
// // }

// /*  Rewires the parents of the tree greedily starting from
//         the new node found in this iterationsation as the parent */
// void rewire() {
//   int lastInserted = nodeCnt - 1;
//   for (auto nodeIndex : nearby) {
//     int par = lastInserted, cur = nodeIndex;

//     // Rewire parents as much as possible (greedily)
//     while (((cost[par] + distance(nodes[par], nodes[cur])) - cost[cur]) <= EPS) {
//       int oldParent = parents[cur];
//       parents[cur] = par;
//       cost[cur] = cost[par] + distance(nodes[par], nodes[cur]);
//       par = cur, cur = oldParent;
//     }
//   }
// }

// // void rewire() {
// //   int lastInserted = nodeCnt - 1;
// //   for (auto nodeIndex : nearby) {
// //     int par = lastInserted, cur = nodeIndex;

// //     while (((cost.at(par) + abs(nodes.at(par).distTo(nodes.at(cur)))) - cost.at(cur)) <= EPS)
// // {
// //       int oldParent = parents.at(cur);
// //       parents.replace(cur, par);
// //       cost.replace(cur, cost.at(par) + abs(nodes.at(par).distTo(nodes.at(cur))));
// //       par = cur;
// //       cur = oldParent;
// //     }
// //   }
// // }

// /*	Runs one iteration of RRT depending on user choice
//         At least one new node is added on the screen each iteration. */
// void RRT() {
//   structPoint newPoint, nearestPoint, nextPoint;
//   bool updated = false;
//   int cnt = 0;
//   int nearestIndex = 0;
//   double minCost = INF;
//   nearby.clear();
//   jumps.resize(nodeCnt);

//   while (!updated) {
//     newPoint = pickRandomPoint();

//     // Find nearest point to the newPoint such that the next node
//     // be added in graph in the (nearestPoint, newPoint) while being obstacle free
//     nearestPoint = *nodes.begin();
//     nearestIndex = 0;
//     for (int i = 0; i < nodeCnt; i++) {
//       if (pathFound and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while
//         cost[i] = cost[parents[i]] + distance(nodes[parents[i]], nodes[i]);

//       // Make smaller jumps sometimes to facilitate passing through narrow passages
//       jumps[i] = randomCoordinate(0.3, 1.0) * JUMP_SIZE;
//       auto pnt = nodes[i];
//       if ((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and
//           isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps[i])))
//         nearestPoint = pnt, nearestIndex = i;
//     }
//     nextPoint = stepNear(nearestPoint, newPoint, jumps[nearestIndex]);
//     if (!isEdgeObstacleFree(nearestPoint, nextPoint))
//       continue;

//     // if( (whichRRT == 1) or (!pathFound and whichRRT == 3)) {
//     // 	// This is where we don't do any RRT* optimization part
//     // 	updated = true ;
//     // 	nodes.push_back(nextPoint); nodeCnt++;
//     // 	parent.push_back(nearestIndex);
//     // 	cost.push_back(cost[nearestIndex] + distance(nearestPoint, nextPoint));
//     // 	if(!pathFound) checkDestinationReached();
//     // 	continue ;
//     // }

//     // Find nearby nodes to the new node as center in ball of radius DISK_SIZE
//     for (int i = 0; i < nodeCnt; i++)
//       if ((nodes[i].distance(nextPoint) - DISK_SIZE) <= EPS and
//           isEdgeObstacleFree(nodes[i], nextPoint))
//         nearby.push_back(i);

//     // Find minimum cost path to the new node
//     int par = nearestIndex;
//     minCost = cost[par] + distance(nodes[par], nextPoint);
//     for (auto nodeIndex : nearby) {
//       if (((cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint)) - minCost) <= EPS)
//         minCost = cost[nodeIndex] + distance(nodes[nodeIndex], nextPoint), par = nodeIndex;
//     }

//     parents.push_back(par);
//     cost.push_back(minCost);
//     nodes.push_back(nextPoint);
//     nodeCnt++;
//     updated = true;
//     if (!pathFound)
//       checkDestinationReached();
//     rewire();
//   }
// }

// // void RRT() {
// //   Point newPoint, nearestPoint, nextPoint;
// //   bool updated = false;
// //   int cnt = 0;
// //   int nearestIndex = 0;
// //   double minCost = INF;
// //   nearby.clear();
// //   // jumps.resize(nodeCnt);

// //   while (!updated) {
// //     newPoint = pickRandomPoint();

// //     nearestPoint = *nodes.begin();
// //     nearestIndex = 0;
// //     for (int i = 0; i < nodeCnt; i++) {
// //       if (pathFound and randomCoordinate(0.0, 1.0) < 0.25) {
// //         cost.replace(
// //             i,
// //             cost.at(parents.at(i)) +
// //                 abs(nodes.at(parents.at(i))
// //                         .distTo(nodes.at(
// //                             i)))); // consertar porque eu estou indexando um elemento que não
// //                             existe
// //       }
// //       jumps.replace(i, randomCoordinate(0.3, 1.0) * JUMP_SIZE);
// //       auto pnt = nodes.at(i);
// //       if ((abs(pnt.distTo(newPoint)) - abs(nearestPoint.distTo(newPoint))) <= EPS and
// //           isEdgeObstacleFree(pnt, steer(pnt, newPoint, jumps.at(i)))) {
// //         nearestPoint = pnt;
// //         nearestIndex = i;
// //       }
// //     }
// //     nextPoint = stepNear(nearestPoint, newPoint, jumps.at(nearestIndex));
// //     if (!isEdgeObstacleFree(nearestPoint, nextPoint)) {
// //       cout << "continue" << endl;
// //       continue;
// //     }
// //     cout << "Depois continue" << endl;
// //     for (int i = 0; i < nodeCnt; i++) {
// //       if ((abs(nodes.at(i).distTo(nextPoint)) - DISK_SIZE) <= EPS and
// //           isEdgeObstacleFree(nodes.at(i), nextPoint)) {
// //         nearby.push_back(i);
// //       }
// //     }

// //     int par = nearestIndex;
// //     minCost = cost.at(par) + abs(nodes.at(par).distTo(nextPoint));
// //     for (auto nodeIndex : nearby) {
// //       if (((cost.at(nodeIndex) + abs(nodes.at(nodeIndex).distTo(nextPoint))) - minCost) <=
// // EPS) {
// //         minCost = cost.at(nodeIndex) + abs(nodes.at(nodeIndex).distTo(nextPoint));
// //         par = nodeIndex;
// //       }
// //     }

// //     parents.push_back(par);
// //     cost.push_back(minCost);
// //     nodes.push_back(nextPoint);
// //     nodeCnt++;
// //     std::cout << "Updated antes: " << updated << endl;
// //     updated = true;
// //     std::cout << "Updated depois: " << updated << endl;
// //     if (!pathFound) {
// //       checkDestinationReached();
// //     }
// //     rewire();
// //   }
// // // }

#endif // PROJECT_UNIFICATION_RRTSTAR_H
