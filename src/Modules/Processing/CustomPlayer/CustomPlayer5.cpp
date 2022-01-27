#include "CustomPlayer5.h"
#include "./RRTstar/RRTstar.h"
#include "Packages/SSLRobotCommand/SSLRobotCommand.h"
#include <iostream>
#include <cmath>

CustomPlayer5::CustomPlayer5(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer5::buildParameters(Parameters::Handler& parameters) {
}

void CustomPlayer5::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &CustomPlayer5::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &CustomPlayer5::receiveField,
          Qt::DirectConnection);
}

void CustomPlayer5::init(const Modules* modules) {
  pathKey.setup(modules->gui()->gameVisualizer());
}

void CustomPlayer5::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
}

void CustomPlayer5::exec() {
  if (!field || !frame || !robot) {
    return;
  }
  if (!frame->has_ball()) {
    return;
  }
  if (!frame.has_value()) {
    return;
  }
  double distRobotBall = robot->distSquaredTo(frame->ball().position());
  bool ballWithRobot = distRobotBall <= 1.49712e+04;
  // Robot robotAlly0 = *frame->allies().findById(0);
  SSLMotion::GoToPoint motion(Point(4200.0, 440.0),
                              (field->enemyPenaltyAreaCenter() - robot->position()).angle(),
                              true);
  SSLRobotCommand command(motion);
  command.set_dribbler(true);
  emit sendCommand(sslNavigation.run(robot.value(), command));
  // if (ballWithRobot) { // Verifica se o robô está com a bola
  //   if (robot->distTo(field->enemyPenaltyAreaCenter()) <= 150) {
  //     currentState = 2;
  //   } else {
  //     currentState = 1;
  //   }
  // } else {
  //   if (field->enemyGoalContains(frame->ball().position())) {
  //     currentState = 3;
  //   } else {
  //     currentState = 0;
  //   }
  // }
  // switch (currentState) {
  //   case 0: {
  //     SSLMotion::GoToPoint motion(frame->ball().position(),
  //                                 (frame->ball().position() - robot->position()).angle(),
  //                                 true);
  //     SSLRobotCommand command(motion);
  //     command.set_dribbler(true);
  //     emit sendCommand(sslNavigation.run(robot.value(), command));
  //     break;
  //   }
  //   case 1: {
  //     if (!target.isNull()) {
  //       vector<Point> pathNodes = vector<Point>();
  //       Point targ = field->enemyPenaltyAreaCenter();
  //       if (targ.distTo(target) >= 20.0) {
  //         RRTSTAR* rrtstar = new RRTSTAR;
  //         receiveTarget(targ);
  //         rrtstar->setInitialPos(robot->position());
  //         rrtstar->nodes.clear();
  //         rrtstar->initialize();
  //         for (int o = 1; o < frame->allies().size(); o++) {
  //           Point topRight = Point(frame->allies().at(o).position().x() + BOT_RADIUS,
  //                                  frame->allies().at(o).position().y() + BOT_RADIUS);
  //           Point bottomLeft = Point(frame->allies().at(o).position().x() - BOT_RADIUS,
  //                                    frame->allies().at(o).position().y() - BOT_RADIUS);
  //           rrtstar->obstacles->addObstacle(topRight, bottomLeft);
  //         }
  //         for (int o = 0; o < frame->enemies().size(); o++) {
  //           Point topRight = Point(frame->enemies().at(o).position().x() + BOT_RADIUS,
  //                                  frame->enemies().at(o).position().y() + BOT_RADIUS);
  //           Point bottomLeft = Point(frame->enemies().at(o).position().x() - BOT_RADIUS,
  //                                    frame->enemies().at(o).position().y() - BOT_RADIUS);
  //           rrtstar->obstacles->addObstacle(topRight, bottomLeft);
  //         }
  //         rrtstar->setMaxIterations(2300);
  //         rrtstar->setStepSize(100);
  //         // RRTSTAR Algorithm
  //         rrtstar->RRTstarAlgorithm();
  //         pathNodesList = rrtstar->generatePath(pathNodes);
  //         pathKey.draw([path = pathNodesList](GameVisualizerPainter2D* f) {
  //           if (!path.empty()) {
  //             for (int i = 0; i < (int) path.size() - 1; ++i) {
  //               f->drawFilledCircle(path[i], 30, Color::Magenta);
  //               f->drawLine(path[i], path[i + 1], Color::Red, 10);
  //             }
  //             f->drawFilledCircle(path.back(), 30, Color::Magenta);
  //           }
  //         });
  //         currentNode = (int) pathNodesList.size() - 1;
  //         objective = pathNodesList.at(currentNode);
  //         delete rrtstar;
  //       }
  //     }
  //     if (robot->position().distTo(objective) <= 150 && currentNode - 1 >= 0) {
  //       currentNode = currentNode - 1;
  //       objective = pathNodesList.at(currentNode);
  //     }
  //     bool lastNode = false;
  //     if (currentNode == 0) {
  //       lastNode = true;
  //     }
  //     SSLMotion::GoToPoint motion(objective,
  //                                 (field->enemyGoalInsideBottom() - robot->position()).angle() -
  //                                     3.14,
  //                                 true);
  //     SSLRobotCommand command(motion);
  //     command.set_dribbler(true);
  //     emit sendCommand(sslNavigation.run(robot.value(), command));
  //     break;
  //   }
  //   case 2: {
  //     Robot closestAlly = *frame->allies().closestTo(field->enemyGoalOutsideTop());
  //     SSLMotion::RotateOnSelf m((closestAlly.position() - robot->position()).angle());
  //     SSLRobotCommand c(m);
  //     c.set_dribbler(true);
  //     cout << "closestAlly: " << closestAlly.id() << endl;
  //     // if (robot->angleTo(closestAlly) <= 1) {
  //     //   c.set_dribbler(false);
  //     //   c.set_front(true);
  //     //   c.set_kickSpeed(3);
  //     //   emit sendCommand(sslNavigation.run(robot.value(), c));
  //     // }
  //     // c.set_dribbler(false);
  //     // c.set_front(true);
  //     // c.set_kickSpeed(3);
  //     emit sendCommand(sslNavigation.run(robot.value(), c));
  //     break;
  //   }
  //   case 3: {
  //     vector<Point> pathNodes = vector<Point>();
  //     Point targ = Point(1.0, 1.0);
  //     if (!target.isNull()) {
  //       if (targ.distTo(target) >= 20.0) {
  //         SSLMotion::GoToPoint motion(field->enemyPenaltyAreaCenter(),
  //                                     (field->center() - robot->position()).angle(),
  //                                     true);
  //         SSLRobotCommand command(motion);
  //         emit sendCommand(sslNavigation.run(robot.value(), command));
  //         RRTSTAR* rrtstar = new RRTSTAR;
  //         receiveTarget(targ);
  //         rrtstar->setInitialPos(robot->position());
  //         rrtstar->setFinalPos(Point(0.0, 0.0));
  //         rrtstar->nodes.clear();
  //         rrtstar->initialize();
  //         for (int o = 1; o < frame->allies().size(); o++) {
  //           Point topRight = Point(frame->allies().at(o).position().x() + BOT_RADIUS,
  //                                  frame->allies().at(o).position().y() + BOT_RADIUS);
  //           Point bottomLeft = Point(frame->allies().at(o).position().x() - BOT_RADIUS,
  //                                    frame->allies().at(o).position().y() - BOT_RADIUS);
  //           rrtstar->obstacles->addObstacle(topRight, bottomLeft);
  //         }
  //         for (int o = 0; o < frame->enemies().size(); o++) {
  //           Point topRight = Point(frame->enemies().at(o).position().x() + BOT_RADIUS,
  //                                  frame->enemies().at(o).position().y() + BOT_RADIUS);
  //           Point bottomLeft = Point(frame->enemies().at(o).position().x() - BOT_RADIUS,
  //                                    frame->enemies().at(o).position().y() - BOT_RADIUS);
  //           rrtstar->obstacles->addObstacle(topRight, bottomLeft);
  //         }
  //         rrtstar->setMaxIterations(2300);
  //         rrtstar->setStepSize(100);
  //         // RRTSTAR Algorithm
  //         rrtstar->RRTstarAlgorithm();
  //         pathNodesList = rrtstar->generatePath(pathNodes);
  //         pathKey.draw([path = pathNodesList](GameVisualizerPainter2D* f) {
  //           if (!path.empty()) {
  //             for (int i = 0; i < (int) path.size() - 1; ++i) {
  //               f->drawFilledCircle(path[i], 30, Color::Magenta);
  //               f->drawLine(path[i], path[i + 1], Color::Red, 10);
  //             }
  //             f->drawFilledCircle(path.back(), 30, Color::Magenta);
  //           }
  //         });
  //         currentNode = (int) pathNodesList.size() - 1;
  //         objective = pathNodesList.at(currentNode);
  //         delete rrtstar;
  //       }
  //     }

  //     if (robot->position().distTo(objective) <= 100 && currentNode - 1 >= 0) {
  //       currentNode = currentNode - 1;
  //       objective = pathNodesList.at(currentNode);
  //     }
  //     bool lastNode = false;
  //     if (currentNode == 0) {
  //       lastNode = true;
  //     }

  //     SSLMotion::GoToPoint motion(objective, (field->center() - robot->position()).angle(),
  //     true); SSLRobotCommand command(motion); emit sendCommand(sslNavigation.run(robot.value(),
  //     command)); break;
  //   }
  //   default: cout << "currentState default: " << currentState << endl;
  // }
  // TODO: here...
  // emit sendCommand(sslNavigation.run(robot.value(), command));
}

void CustomPlayer5::receiveField(const Field& field) {
  shared->field = field;
}

void CustomPlayer5::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

void CustomPlayer5::receiveTarget(const Point& targetReceived) {
  target = targetReceived;
}

void CustomPlayer5::receivePathNodesList(const vector<Point>& pathNodesListReceived) {
  pathNodesList = pathNodesListReceived;
}

void CustomPlayer5::receiveObjective(const Point& objectiveReceived) {
  objective = objectiveReceived;
}

void CustomPlayer5::receiveCurrentNode(const int& currentNodeReceived) {
  currentNode = currentNodeReceived;
}

static_block {
  Factory::processing.insert<CustomPlayer5>();
};
