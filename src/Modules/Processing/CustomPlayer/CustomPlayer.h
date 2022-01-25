#ifndef PROJECT_UNIFICATION_CUSTOMPLAYER_H
#define PROJECT_UNIFICATION_CUSTOMPLAYER_H

#include "./RRTstar/RRTstar.h"
#include "Modules/Modules.h"
#include "Modules/Processing/ProcessingUtils/ProcessingUtils.h"

class CustomPlayer : public Processing {
 public:
  CustomPlayer(int index, QThreadPool* threadPool);

 protected:
  void buildParameters(Parameters::Handler& parameters) override;
  void connectModules(const Modules* modules) override;
  void init(const Modules* modules) override;
  void update() override;
  void exec() override;

 private:
  struct Args {};
  Args args;

  struct Shared {
    SharedOptional<Frame> frame;
    SharedOptional<Robot> robot;
    SharedOptional<Field> field;
    // SharedOptional<Point> target = Point(-4500, -3000); // field->bottomLeft()
    // SharedOptional<vector<Point>> pathNodesList;
    // SharedOptional<Point> objective = Point(3500, 0.0);
    // SharedOptional<int> currentNode = -1;
    SharedValue<QSet<Qt::Key>> keys;
  };
  SharedWrapper<Shared, std::mutex> shared;

  std::optional<Field> field;
  std::optional<Frame> frame;
  std::optional<Robot> robot;
  Point target = Point(-4500, -3000); // field->bottomLeft()
  vector<Point> pathNodesList;
  Point objective;
  int currentNode;
  GameVisualizer::Key pathKey;

  SSLNavigation sslNavigation;
  VSSNavigation vssNavigation;

 private slots:
  void receiveField(const Field& field);
  void receiveFrame(const Frame& frame);
  void receiveTarget(const Point& target);
  void receivePathNodesList(const vector<Point>& pathNodesList);
  void receiveObjective(const Point& objective);
  void receiveCurrentNode(const int& currentNode);
};

#endif // PROJECT_UNIFICATION_CUSTOMPLAYER_H
