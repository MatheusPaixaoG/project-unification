#include "CustomPlayer.h"
#include "./RRTstar/RRTstar.h"
#include "Packages/SSLRobotCommand/SSLRobotCommand.h"
#include <iostream>
#include <cmath>

CustomPlayer::CustomPlayer(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer::buildParameters(Parameters::Handler& parameters) {
}

void CustomPlayer::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &CustomPlayer::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &CustomPlayer::receiveField,
          Qt::DirectConnection);
}

void CustomPlayer::init(const Modules* modules) {
  pathKey.setup(modules->gui()->gameVisualizer());
}

void CustomPlayer::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
}

void CustomPlayer::exec() {
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
  if (ballWithRobot) { // Verifica se o robô está com a bola
    vector<Point> pathNodes = vector<Point>();
    Point targ = field->enemyPenaltyAreaCenter();
    if (!target.isNull()) {
      if (targ.distTo(target) >= 20.0) {
        RRTSTAR* rrtstar = new RRTSTAR;
        receiveTarget(targ);
        rrtstar->setInitialPos(robot->position());
        rrtstar->nodes.clear();
        rrtstar->initialize();
        for (int o = 1; o < frame->allies().size(); o++) {
          Point topRight = Point(frame->allies().at(o).position().x() + BOT_RADIUS,
                                 frame->allies().at(o).position().y() + BOT_RADIUS);
          Point bottomLeft = Point(frame->allies().at(o).position().x() - BOT_RADIUS,
                                   frame->allies().at(o).position().y() - BOT_RADIUS);
          rrtstar->obstacles->addObstacle(topRight, bottomLeft);
        }
        rrtstar->setMaxIterations(2300);
        rrtstar->setStepSize(100);
        // RRTSTAR Algorithm
        rrtstar->RRTstarAlgorithm();
        Node* q;
        if (rrtstar->reached()) {
          q = rrtstar->lastNode;
        } else {
          // Se ainda não chegou ao objetivo, o menor caminho vai começar do nó mais próximo de
          // endPos
          q = rrtstar->nearest(rrtstar->endPos);
          cout << "Exceeded max iterations!" << endl;
        }
        // Gera o menor caminho para o objetivo
        while (q != NULL) {
          pathNodes.push_back(q->position);
          rrtstar->path.push_back(q);
          q = q->parent;
        }
        receivePathNodesList(pathNodes);
        pathKey.draw([path = pathNodes](GameVisualizerPainter2D* f) {
          if (!path.empty()) {
            for (int i = 0; i < (int) path.size() - 1; ++i) {
              f->drawFilledCircle(path[i], 30, Color::Magenta);
              f->drawLine(path[i], path[i + 1], Color::Red, 10);
            }
            f->drawFilledCircle(path.back(), 30, Color::Magenta);
          }
        });
        currentNode = (int) pathNodesList.size() - 1;
        objective = pathNodesList.at(currentNode);
        delete rrtstar;
      }
    }
    if (robot->position().distTo(objective) <= 100 && currentNode - 1 >= 0) {
      currentNode = currentNode - 1;
      objective = pathNodesList.at(currentNode);
    }
    bool lastNode = false;
    if (currentNode == 0) {
      lastNode = true;
    }
    SSLMotion::GoToPoint motion(objective,
                                (field->enemyGoalInsideBottom() - robot->position()).angle(),
                                true);
    SSLRobotCommand command(motion);
    command.set_dribbler(true);
    if (robot->distSquaredTo(field->enemyPenaltyAreaCenter()) <= 1.29712e+04) {
      command.set_dribbler(true);
      if (field->enemyPenaltyAreaContains(frame->ball().position())) {
        command.set_dribbler(false);
        command.set_front(true);
        command.set_kickSpeed(3);
        emit sendCommand(sslNavigation.run(robot.value(), command));
      } else {
        emit sendCommand(sslNavigation.run(robot.value(), command));
      }
    } else {
      emit sendCommand(sslNavigation.run(robot.value(), command));
    }
  } else {
    if (field->enemyGoalContains(frame->ball().position())) {
      vector<Point> pathNodes = vector<Point>();
      Point targ = Point(1.0, 1.0);
      if (!target.isNull()) {
        if (targ.distTo(target) >= 20.0) {
          SSLMotion::GoToPoint motion(field->enemyPenaltyAreaCenter(),
                                      (field->center() - robot->position()).angle(),
                                      true);
          SSLRobotCommand command(motion);
          emit sendCommand(sslNavigation.run(robot.value(), command));
          RRTSTAR* rrtstar = new RRTSTAR;
          receiveTarget(targ);
          rrtstar->setInitialPos(robot->position());
          rrtstar->setFinalPos(Point(0.0, 0.0));
          rrtstar->nodes.clear();
          rrtstar->initialize();
          for (int o = 1; o < frame->allies().size(); o++) {
            Point topRight = Point(frame->allies().at(o).position().x() + BOT_RADIUS,
                                   frame->allies().at(o).position().y() + BOT_RADIUS);
            Point bottomLeft = Point(frame->allies().at(o).position().x() - BOT_RADIUS,
                                     frame->allies().at(o).position().y() - BOT_RADIUS);
            rrtstar->obstacles->addObstacle(topRight, bottomLeft);
          }
          rrtstar->setMaxIterations(2300);
          rrtstar->setStepSize(100);
          // RRTSTAR Algorithm
          rrtstar->RRTstarAlgorithm();
          Node* q;
          if (rrtstar->reached()) {
            q = rrtstar->lastNode;
          } else {
            // Se ainda não chegou ao objetivo, o menor caminho vai começar do nó mais próximo de
            // endPos
            q = rrtstar->nearest(rrtstar->endPos);
            cout << "Exceeded max iterations!" << endl;
          }
          // Gera o menor caminho para o objetivo
          while (q != NULL) {
            pathNodes.push_back(q->position);
            rrtstar->path.push_back(q);
            q = q->parent;
          }
          receivePathNodesList(pathNodes);
          cout << "path depois: " << endl;
          for (int g = 0; g < (int) rrtstar->path.size(); g++) {
            cout << "rrtstar->path[" << g << "]: (" << rrtstar->path[g]->position.x() << ", "
                 << rrtstar->path[g]->position.y() << ")" << endl;
            cout << "pathNodes.size(): " << pathNodes.size() << endl;
          }
          pathKey.draw([path = pathNodes](GameVisualizerPainter2D* f) {
            if (!path.empty()) {
              for (int i = 0; i < (int) path.size() - 1; ++i) {
                f->drawFilledCircle(path[i], 30, Color::Magenta);
                f->drawLine(path[i], path[i + 1], Color::Red, 10);
              }
              f->drawFilledCircle(path.back(), 30, Color::Magenta);
            }
          });
          currentNode = (int) pathNodesList.size() - 1;
          objective = pathNodesList.at(currentNode);
          cout << "currentNode: " << currentNode << endl;
          delete rrtstar;
        }
      }

      if (robot->position().distTo(objective) <= 100 && currentNode - 1 >= 0) {
        currentNode = currentNode - 1;
        cout << "currentNode: " << currentNode << endl;
        objective = pathNodesList.at(currentNode);
      }
      bool lastNode = false;
      if (currentNode == 0) {
        lastNode = true;
      }

      SSLMotion::GoToPoint motion(objective, (field->center() - robot->position()).angle(), true);
      SSLRobotCommand command(motion);
      emit sendCommand(sslNavigation.run(robot.value(), command));
    } else {
      SSLMotion::GoToPoint motion(frame->ball().position(),
                                  (frame->ball().position() - robot->position()).angle(),
                                  true);
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), command));
    }
  }
  // TODO: here...
  // emit sendCommand(sslNavigation.run(robot.value(), command));
}

void CustomPlayer::receiveField(const Field& field) {
  shared->field = field;
}

void CustomPlayer::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

void CustomPlayer::receiveTarget(const Point& targetReceived) {
  target = targetReceived;
}

void CustomPlayer::receivePathNodesList(const vector<Point>& pathNodesListReceived) {
  pathNodesList = pathNodesListReceived;
}

void CustomPlayer::receiveObjective(const Point& objectiveReceived) {
  objective = objectiveReceived;
}

void CustomPlayer::receiveCurrentNode(const int& currentNodeReceived) {
  currentNode = currentNodeReceived;
}

static_block {
  Factory::processing.insert<CustomPlayer>();
};
