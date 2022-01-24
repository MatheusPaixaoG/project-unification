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
  // std::cout << robot->distSquaredTo(frame->ball().position());
  double distRobotBall = robot->distSquaredTo(frame->ball().position());
  // double distRobotHasball = pow((robot->position() - frame->ball().position()).norm(), 2);
  bool ballWithRobot = distRobotBall <= 1.49712e+04;
  Point targ = field->center();
  // receiveTarget(field->bottomLeft());
  if (shared->target.has_value()) {
    if (targ.distTo(shared->target.get()) >= 20.0) {
      receiveTarget(targ);
      RRTSTAR* rrtstar = new RRTSTAR;
      rrtstar->setMaxIterations(1500);
      rrtstar->setStepSize(75);
      cout << "robot->position(): (" << robot->position().x() << ", " << robot->position().y()
           << ")" << endl;
      // setInitialPos(robot->position());
      // rrtstar->setStartPos(robot->position());
      cout << "frame->ball().position(): (" << frame->ball().position().x() << ", "
           << frame->ball().position().y() << ")" << endl;
      // setFinalPos(frame->ball().position());
      // rrtstar->setEndPos(frame->ball().position());

      cout << "path antes: " << endl;
      for (int g = 0; g < (int) rrtstar->path.size(); g++) {
        cout << "rrtstar->path[" << g << "]: (" << rrtstar->path[g]->position.x() << ", "
             << rrtstar->path[g]->position.y() << ")" << endl;
      }
      // RRTSTAR Algorithm
      for (int i = 0; i < rrtstar->max_iter; i++) {
        Node* q = rrtstar->getRandomNode();
        if (q) {
          Node* qNearest = rrtstar->nearest(q->position);
          if (rrtstar->distance(q->position, qNearest->position) > rrtstar->step_size) {
            Point newConfigPosOrient;
            // DubinsPath path;
            if (BOT_FOLLOW_DUBIN) {
              // newConfigPosOrient = rrtstar->newDubinConfig(q, qNearest, path);
            } else {
              newConfigPosOrient = rrtstar->newConfig(q, qNearest);
            }
            Point newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
            if (!rrtstar->obstacles->isSegmentInObstacle(newConfigPos, qNearest->position)) {
              Node* qNew = new Node;
              qNew->position = newConfigPos;
              qNew->orientation = newConfigPosOrient.angle();
              // qNew->path = path;

              vector<Node*> Qnear;
              rrtstar->near(qNew->position, rrtstar->step_size * RRTSTAR_NEIGHBOR_FACTOR, Qnear);
              qDebug() << "Found Nearby " << Qnear.size() << "\n";
              Node* qMin = qNearest;
              double cmin = rrtstar->Cost(qNearest) + rrtstar->PathCost(qNearest, qNew);
              for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
                Node* qNear = Qnear[j];
                if (!rrtstar->obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
                    (rrtstar->Cost(qNear) + rrtstar->PathCost(qNear, qNew)) < cmin) {
                  qMin = qNear;
                  cmin = rrtstar->Cost(qNear) + rrtstar->PathCost(qNear, qNew);
                }
              }
              rrtstar->add(qMin, qNew);

              for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
                Node* qNear = Qnear[j];
                if (!rrtstar->obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
                    (rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear)) < rrtstar->Cost(qNear)) {
                  Node* qParent = qNear->parent;
                  // Remove edge between qParent and qNear
                  qParent->children.erase(
                      std::remove(qParent->children.begin(), qParent->children.end(), qNear),
                      qParent->children.end());

                  // Add edge between qNew and qNear
                  qNear->cost = rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear);
                  qNear->parent = qNew;
                  qNew->children.push_back(qNear);
                }
              }
            }
          }
        }
        if (rrtstar->reached()) {
          cout << "Reached destination" << endl;
          // ui->statusBox->setText(tr("Reached Destination!"));
          break;
        }
        // renderArea->update();
        qApp->processEvents();
      }

      Node* q;
      if (rrtstar->reached()) {
        q = rrtstar->lastNode;
      } else {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = rrtstar->nearest(rrtstar->endPos);
        cout << "Exceeded max iterations!" << endl;
      }
      // generate shortest path to destination.
      while (q != NULL) {
        rrtstar->path.push_back(q);
        q = q->parent;
      }

      cout << "path depois: " << endl;
      for (int g = 0; g < (int) rrtstar->path.size(); g++) {
        cout << "rrtstar->path[" << g << "]: (" << rrtstar->path[g]->position.x() << ", "
             << rrtstar->path[g]->position.y() << ")" << endl;
      }
    }
  }

  // receiveTarget(field->bottomLeft());
  // if (shared->target.has_value()) {
  //   if (targ.distTo(shared->target.get()) >= 20.0) {
  //     receiveTarget(targ);
  //     getInput(robot->position(), field->center(), 1, frame->allies(), frame->enemies());
  //     prepareInput();
  //     nodeCnt = 1;
  //     nodes.push_back(start);
  //     parents.push_back(0);
  //     cost.push_back(0);
  //     jumps.push_back(-1.0);
  //     for (int w = 0; w < 3; w++) {
  //       RRT();
  //     }
  //     for (int g = 0; g < (int) nodes.size(); g++) {
  //       cout << "nodes[" << g << "]: (" << nodes[g].x << ", " << nodes[g].y << ")" << endl;
  //     }
  //   }
  // }
  // RRTSTAR* rrtstar = new RRTSTAR;
  // rrtstar->setMaxIterations(50);
  // rrtstar->setStepSize(75);
  // cout << "robot->position(): (" << robot->position().x() << ", " << robot->position().y() << ")"
  //      << endl;
  // // setInitialPos(robot->position());
  // // rrtstar->setStartPos(robot->position());
  // cout << "frame->ball().position(): (" << frame->ball().position().x() << ", "
  //      << frame->ball().position().y() << ")" << endl;
  // // setFinalPos(frame->ball().position());
  // // rrtstar->setEndPos(frame->ball().position());

  // cout << "nodes antes: " << endl;
  // for (int g = 0; g < (int) rrtstar->nodes.size(); g++) {
  //   cout << "rrtstar->nodes[" << g << "]: (" << rrtstar->nodes[g]->position.x() << ", "
  //        << rrtstar->nodes[g]->position.y() << ")" << endl;
  // }
  // // RRTSTAR Algorithm
  // for (int i = 0; i < rrtstar->max_iter; i++) {
  //   Node* q = rrtstar->getRandomNode();
  //   if (q) {
  //     Node* qNearest = rrtstar->nearest(q->position);
  //     if (rrtstar->distance(q->position, qNearest->position) > rrtstar->step_size) {
  //       Point newConfigPosOrient;
  //       // DubinsPath path;
  //       if (BOT_FOLLOW_DUBIN) {
  //         // newConfigPosOrient = rrtstar->newDubinConfig(q, qNearest, path);
  //       } else {
  //         newConfigPosOrient = rrtstar->newConfig(q, qNearest);
  //       }
  //       Point newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
  //       if (!rrtstar->obstacles->isSegmentInObstacle(newConfigPos, qNearest->position)) {
  //         Node* qNew = new Node;
  //         qNew->position = newConfigPos;
  //         qNew->orientation = newConfigPosOrient.angle();
  //         // qNew->path = path;

  //         vector<Node*> Qnear;
  //         rrtstar->near(qNew->position, rrtstar->step_size * RRTSTAR_NEIGHBOR_FACTOR, Qnear);
  //         qDebug() << "Found Nearby " << Qnear.size() << "\n";
  //         Node* qMin = qNearest;
  //         double cmin = rrtstar->Cost(qNearest) + rrtstar->PathCost(qNearest, qNew);
  //         for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
  //           Node* qNear = Qnear[j];
  //           if (!rrtstar->obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
  //               (rrtstar->Cost(qNear) + rrtstar->PathCost(qNear, qNew)) < cmin) {
  //             qMin = qNear;
  //             cmin = rrtstar->Cost(qNear) + rrtstar->PathCost(qNear, qNew);
  //           }
  //         }
  //         rrtstar->add(qMin, qNew);

  //         for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
  //           Node* qNear = Qnear[j];
  //           if (!rrtstar->obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
  //               (rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear)) < rrtstar->Cost(qNear)) {
  //             Node* qParent = qNear->parent;
  //             // Remove edge between qParent and qNear
  //             qParent->children.erase(
  //                 std::remove(qParent->children.begin(), qParent->children.end(), qNear),
  //                 qParent->children.end());

  //             // Add edge between qNew and qNear
  //             qNear->cost = rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear);
  //             qNear->parent = qNew;
  //             qNew->children.push_back(qNear);
  //           }
  //         }
  //       }
  //     }
  //   }
  //   if (rrtstar->reached()) {
  //     cout << "Reached destination" << endl;
  //     // ui->statusBox->setText(tr("Reached Destination!"));
  //     break;
  //   }
  //   // renderArea->update();
  //   qApp->processEvents();
  // }

  // cout << "nodes depois: " << endl;
  // for (int g = 0; g < (int) rrtstar->nodes.size(); g++) {
  //   cout << "field->bottomLeft(): (" << field->bottomLeft().x() << ", " <<
  //   field->bottomLeft().y()
  //        << ")" << endl;
  //   cout << "rrtstar->nodes[" << g << "]: (" << rrtstar->nodes[g]->position.x() << ", "
  //        << rrtstar->nodes[g]->position.y() << ")" << endl;
  // }

  if (ballWithRobot) { // Verifica se o robô está com a bola
    // cout << "ballWithRobot antes do GoToPoint\n";
    SSLMotion::GoToPoint motion(field->enemyPenaltyAreaCenter(),
                                (field->enemyGoalInsideBottom() - robot->position()).angle(),
                                true);
    // cout << "ballWithRobot depois do GoToPoint\n";
    SSLRobotCommand command(motion);
    command.set_dribbler(true);
    if (robot->distSquaredTo(field->enemyPenaltyAreaCenter()) <= 1.29712e+04) {
      // SSLMotion::RotateOnSelf motion((field->enemyGoalInsideBottom() -
      // robot->position()).angle()); SSLRobotCommand command(motion);
      command.set_dribbler(true);
      if (field->enemyPenaltyAreaContains(frame->ball().position())) {
        // SSLMotion::RotateOnSelf m((field->enemyGoalInsideBottom() - robot->position()).angle());
        // SSLRobotCommand c(m);
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
    // cout << "ballNotWithRobot antes do GoToPoint\n";
    if (field->enemyGoalContains(frame->ball().position())) {
      // cout << "ballNotWithRobot dentro do if antes do GoToPoint\n";
      SSLMotion::GoToPoint motion(field->center(), field->center().angle(), true);
      SSLRobotCommand command(motion);
      // cout << "ballNotWithRobot dentro do if depois do GoToPoint\n";
      emit sendCommand(sslNavigation.run(robot.value(), command));
    } else {
      // cout << "ballNotWithRobot dentro do else antes do GoToPoint\n";
      SSLMotion::GoToPoint motion(frame->ball().position(),
                                  (frame->ball().position() - robot->position()).angle(),
                                  true);
      SSLRobotCommand command(motion);
      // cout << "ballNotWithRobot dentro do else depois do GoToPoint\n";
      command.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), command));
    }
  }
  // if (robot->angleTo(field->enemyGoalInsideCenter()) <= 1 && ballWithRobot) {
  //   std::cout << "a";
  //   SSLMotion::RotateOnSelf motion(field->enemyGoalInsideCenter().angle());
  //   SSLRobotCommand command(motion);
  //   command.set_kickSpeed(100000);
  //   emit sendCommand(sslNavigation.run(robot.value(), command));
  // }
  // if (field->enemyPenaltyAreaContains(
  //         robot->position())) { // Aqui seria útil usar a função para ver se um ponto está dentro
  //         de
  //   // uma área qualquer do campo
  //   SSLMotion::GoToPoint motion(robot->position(), field->enemyGoalInsideCenter().angle(), true);
  //   SSLRobotCommand command(motion);
  //   command.set_kickSpeed(1000);
  //   emit sendCommand(sslNavigation.run(robot.value(), command));
  // }
  // if (field->enemyGoalContains(frame->ball().position())) {
  //   SSLMotion::GoToPoint motion(field->center(), field->center().angle(), true);
  //   SSLRobotCommand command(motion);
  //   emit sendCommand(sslNavigation.run(robot.value(), command));
  // }
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

void CustomPlayer::receiveTarget(const Point& target) {
  shared->target = target;
}

static_block {
  Factory::processing.insert<CustomPlayer>();
};
