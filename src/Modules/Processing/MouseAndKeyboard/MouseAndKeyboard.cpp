//
// Created by jose-cruz on 06/08/2021.
//

#include "MouseAndKeyboard.h"
#include "Packages/Command/Command.h"
#include "Packages/SSLRobotCommand/SSLRobotCommand.h"

MouseAndKeyboard::MouseAndKeyboard(int index, QThreadPool* threadPool) :
    Processing(index, threadPool) {
}

void MouseAndKeyboard::buildParameters(Parameters::Handler& parameters) {
  parameters["League"] = Parameters::ComboBox(args.league, MagicEnum::values<Args::League>());
  parameters["League"]["SSL"]["Key"]["GoToPoint"] =
      Parameters::ComboBox(args.ssl.goToPointKey, MagicEnum::values<Qt::Key>());
  parameters["League"]["SSL"]["Key"]["RotateInPoint"] =
      Parameters::ComboBox(args.ssl.rotateInPointKey, MagicEnum::values<Qt::Key>());
  parameters["League"]["SSL"]["Key"]["RotateOnSelf"] =
      Parameters::ComboBox(args.ssl.rotateOnSelfKey, MagicEnum::values<Qt::Key>());

  parameters["League"]["VSS"]["Key"]["GoToPoint"] =
      Parameters::ComboBox(args.vss.goToPointKey, MagicEnum::values<Qt::Key>());
  parameters["League"]["VSS"]["Key"]["CCWSpin"] =
      Parameters::ComboBox(args.vss.ccwSpin, MagicEnum::values<Qt::Key>());
  parameters["League"]["VSS"]["Key"]["CWSpin"] =
      Parameters::ComboBox(args.vss.cwSpin, MagicEnum::values<Qt::Key>());
}

void MouseAndKeyboard::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &MouseAndKeyboard::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &MouseAndKeyboard::receiveField,
          Qt::DirectConnection);

  connect(modules->gui()->gameVisualizer(),
          &GameVisualizer::relativeMousePos,
          this,
          &MouseAndKeyboard::receiveMousePos,
          Qt::DirectConnection);

  connect(modules->gui()->gameVisualizer(),
          &GameVisualizer::onKeyPress,
          this,
          &MouseAndKeyboard::receiveKeyPressed,
          Qt::DirectConnection);

  connect(modules->gui()->gameVisualizer(),
          &GameVisualizer::onKeyRelease,
          this,
          &MouseAndKeyboard::receiveKeyReleased,
          Qt::DirectConnection);
}

void MouseAndKeyboard::init(const Modules* modules) {
  targetKey.setup(modules->gui()->gameVisualizer(), Painting::Layers::Top);
}

void MouseAndKeyboard::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
  shared->mouse.extract_to(mouse);

  if (args.league.updated()) {
    targetKey.clear();
  }
}

void MouseAndKeyboard::exec() {
  if (!field || !frame || !robot || !mouse) {
    return;
  }
  if (!frame->has_ball()) {
    return;
  }

  if (args.league == Args::League::SSL) {
    ssl();
  } else {
    vss();
  }
}

void MouseAndKeyboard::ssl() {
  targetKey.draw([mouse = this->mouse](GameVisualizerPainter2D* f) {
    f->drawFilledCircle(mouse.value(), 45, Color::Red);
  });

  const QSet<Qt::Key> keys = shared->keys.get();

  if (keys.contains(args.ssl.goToPointKey)) {
    SSLMotion::GoToPoint goToPoint(mouse.value(),
                                   (field->center() - robot->position()).angle(),
                                   true);
    auto command = sslNavigation.run(robot.value(), SSLRobotCommand(goToPoint));
    emit sendCommand(command);
  } else if (keys.contains(args.ssl.rotateInPointKey)) {
    SSLMotion::RotateInPoint rotateInPoint(mouse.value(),
                                           (mouse.value() - robot->position()).angle(),
                                           false,
                                           360);
    auto command = sslNavigation.run(robot.value(), SSLRobotCommand(rotateInPoint));
    emit sendCommand(command);
  } else if (keys.contains(args.ssl.rotateOnSelfKey)) {
    SSLMotion::RotateOnSelf rotateOnSelf((mouse.value() - robot->position()).angle());
    auto command = sslNavigation.run(robot.value(), SSLRobotCommand(rotateOnSelf));
    emit sendCommand(command);
  } else if (keys.contains(args.ssl.rotateToBallKey)) {
    SSLMotion::RotateOnSelf rotateToBall((frame->ball().position() - robot->position()).angle());
    auto command = sslNavigation.run(robot.value(), SSLRobotCommand(rotateToBall));
    emit sendCommand(command);
  } else if (keys.contains(args.ssl.kickKey)) {
    SSLMotion::RotateOnSelf rotateToBall((frame->ball().position() - robot->position()).angle());
    auto command = sslNavigation.run(robot.value(), SSLRobotCommand(rotateToBall));
    command.set_front(true);
    command.set_kickSpeed(100);
    emit sendCommand(command);
  } else if (keys.contains(args.ssl.goToBallKey)) {
    if (robot->distSquaredTo(frame->ball().position()) <=
        1.49712e+04) { // Verifica se o robô está com a bola
      SSLMotion::GoToPoint motion(field->center(),
                                  (field->enemyGoalInsideBottom() - robot->position()).angle(),
                                  true);
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      if (robot->distSquaredTo(field->center()) <= 1.49712e+04) {
        command.set_dribbler(false);
        command.set_front(true); // O front = true é que faz o robô chutar
        command.set_kickSpeed(100);
      }
      emit sendCommand(sslNavigation.run(robot.value(), command));
    } else {
      SSLMotion::GoToPoint motion(frame->ball().position(),
                                  (frame->ball().position() - robot->position()).angle(),
                                  true);
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), command));
    }
  } else {
    emit sendCommand(SSLCommand::halt(index()));
  }
}

void MouseAndKeyboard::vss() {
  targetKey.draw([mouse = this->mouse](GameVisualizerPainter2D* f) {
    f->drawFilledCircle(mouse.value(), 4.5, Color::Red);
  });

  const QSet<Qt::Key> keys = shared->keys.get();

  if (keys.contains(args.vss.goToPointKey)) {
    VSSMotion::GoToPoint goToPoint(mouse.value());
    auto command = vssNavigation.run(robot.value(), VSSRobotCommand(goToPoint));
    emit sendCommand(command);
  } else if (keys.contains(args.vss.ccwSpin)) {
    VSSMotion::Spin spin(false);
    auto command = vssNavigation.run(robot.value(), VSSRobotCommand(spin));
    emit sendCommand(command);
  } else if (keys.contains(args.vss.cwSpin)) {
    VSSMotion::Spin spin(true);
    auto command = vssNavigation.run(robot.value(), VSSRobotCommand(spin));
    emit sendCommand(command);
  } else {
    emit sendCommand(VSSCommand::halt(index()));
  }
}

void MouseAndKeyboard::receiveField(const Field& field) {
  shared->field = field;
}

void MouseAndKeyboard::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

void MouseAndKeyboard::receiveMousePos(const Point& mouse) {
  shared.apply([&mouse](Shared& obj) {
    if (!obj.keys->empty()) {
      obj.mouse = mouse;
    }
  });
}

void MouseAndKeyboard::receiveKeyPressed(Qt::Key key) {
  shared->keys->insert(key);
}

void MouseAndKeyboard::receiveKeyReleased(Qt::Key key) {
  shared->keys->remove(key);
}

static_block {
  Factory::processing.insert<MouseAndKeyboard>();
};
