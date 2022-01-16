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
  Point targ = field->bottomCenter();
  // receiveTarget(field->bottomLeft());
  // if (shared->target.has_value()) {
  //   if (targ.distSquaredTo(shared->target.get()) >= 1.49712e+04) {
  //     cout << "target\n";
  //     receiveTarget(targ);
  //   }
  // }
  getInput(robot->position(), field->center(), 1, frame->allies(), frame->enemies());
  prepareInput();
  // RRT();
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
