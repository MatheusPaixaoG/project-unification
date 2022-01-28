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
  if (ballWithRobot) { // Verifica se o robô está com a bola
    currentState = 1;
  } else { // Se o robô não estiver com a bola
    currentState = 0;
  }
  switch (currentState) {
    case 0: { // Se o robô não está com a bola, então ele vai para o ponto (4200.0, 440.0)
      SSLMotion::GoToPoint motion(Point(4200.0, 440.0),
                                  (field->enemyPenaltyAreaCenter() - robot->position()).angle(),
                                  true);
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), command));
      break;
    }
    case 1: { // Se o robô está com a bola, então ele gira para o ponto
              // field->enemyGoalInsideBottom() e chuta
      SSLMotion::RotateOnSelf motion((field->enemyGoalInsideBottom() - robot->position()).angle());
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      if (abs(robot->angleTo(field->enemyGoalInsideBottom())) <= 0.01) {
        command.set_dribbler(false);
        command.set_front(true);
        command.set_kickSpeed(2);
      }
      emit sendCommand(sslNavigation.run(robot.value(), command));
      break;
    }
    default: cout << "currentState default: " << currentState << endl;
  }
  // TODO: here...
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

static_block {
  Factory::processing.insert<CustomPlayer5>();
};
