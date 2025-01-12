#ifndef PROJECT_UNIFICATION_CUSTOMPLAYER5_H
#define PROJECT_UNIFICATION_CUSTOMPLAYER5_H

#include "./RRTstar/RRTstar.h"
#include "Modules/Modules.h"
#include "Modules/Processing/ProcessingUtils/ProcessingUtils.h"

class CustomPlayer5 : public Processing {
 public:
  CustomPlayer5(int index, QThreadPool* threadPool);

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
  int currentState;
  GameVisualizer::Key pathKey;

  SSLNavigation sslNavigation;
  VSSNavigation vssNavigation;

 private slots:
  void receiveField(const Field& field);
  void receiveFrame(const Frame& frame);
  void receiveTarget(const Point& target);
};

#endif // PROJECT_UNIFICATION_CUSTOMPLAYER5_H
