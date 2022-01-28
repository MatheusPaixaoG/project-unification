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
    if (robot->distTo(field->enemyPenaltyAreaCenter()) <=
        150) { // Verifica se o robô está no objetivo e passa para o robô 5
      currentState = 2;
    } else { // Vai até o centro da área de pênalti
      currentState = 1;
    }
  } else {                                                    // Se a bola não estiver com o robô
    if (field->enemyGoalContains(frame->ball().position())) { // Verifica se foi gol
      currentState = 3;
    } else if (field->enemyPenaltyAreaContains(
                   frame->ball().position())) { // Verifica se a bola está dentro da área
      currentState = 4;
    } else { // vai para a bola
      currentState = 0;
    }
  }
  switch (currentState) {
    case 0: {
      SSLMotion::GoToPoint motion(frame->ball().position(),
                                  (frame->ball().position() - robot->position()).angle(),
                                  true);
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), command));
      break;
    }
    case 1: {
      if (!target.isNull()) { // Verifica se existe um target, para evitar erros
        vector<Point> pathNodes = vector<Point>();
        Point targ = field->enemyPenaltyAreaCenter();
        if (targ.distTo(target) >= 20.0) { // Se o meu objetivo não está perto do target, então eu
                                           // preciso executar o algoritmo (uma otimização para
                                           // evitar várias execuções do algoritmo no mesmo caso)
          RRTSTAR* rrtstar = new RRTSTAR;
          receiveTarget(targ); // Muda o target para ser o objetivo do movimento
          // Configura o algoritmo para a configuração inicial
          rrtstar->setInitialPos(robot->position());
          rrtstar->nodes.clear();
          rrtstar->initialize();
          // Os dois loops são para adicionar os obstáculos, que são os robôs inimigos
          for (int o = 1; o < frame->allies().size(); o++) {
            Point topRight = Point(frame->allies().at(o).position().x() + BOT_RADIUS,
                                   frame->allies().at(o).position().y() + BOT_RADIUS);
            Point bottomLeft = Point(frame->allies().at(o).position().x() - BOT_RADIUS,
                                     frame->allies().at(o).position().y() - BOT_RADIUS);
            rrtstar->obstacles->addObstacle(topRight, bottomLeft);
          }
          for (int o = 0; o < frame->enemies().size(); o++) {
            Point topRight = Point(frame->enemies().at(o).position().x() + BOT_RADIUS,
                                   frame->enemies().at(o).position().y() + BOT_RADIUS);
            Point bottomLeft = Point(frame->enemies().at(o).position().x() - BOT_RADIUS,
                                     frame->enemies().at(o).position().y() - BOT_RADIUS);
            rrtstar->obstacles->addObstacle(topRight, bottomLeft);
          }
          // Configura o número máximo de iterações do algoritmo, mesmo que ele não use todas para
          // chegar ao objetivo
          rrtstar->setMaxIterations(2300);
          // Configura o tamanho máximo de distância entre pontos
          rrtstar->setStepSize(100);
          // RRTSTAR Algorithm
          rrtstar->RRTstarAlgorithm();
          // Gera o caminho mais curto entre o ponto de partida e o objetivo, dados os pontos
          // gerados pelo algoritmo
          pathNodesList = rrtstar->generatePath(pathNodes);
          // As próximas linhas desenham os pontos na tela
          pathKey.draw([path = pathNodesList](GameVisualizerPainter2D* f) {
            if (!path.empty()) {
              for (int i = 0; i < (int) path.size() - 1; ++i) {
                f->drawFilledCircle(path[i], 30, Color::Magenta);
                f->drawLine(path[i], path[i + 1], Color::Red, 10);
              }
              f->drawFilledCircle(path.back(), 30, Color::Magenta);
            }
          });
          // Configura o índice do nó inicial e o objetivo inicial do movimento
          currentNode = (int) pathNodesList.size() - 1;
          objective = pathNodesList.at(currentNode);
          delete rrtstar; // Deleta a instância de rrtstar para liberar memória
        }
      }
      // Muda o objetivo sempre que o robô estiver perto dele
      if (robot->position().distTo(objective) <= 150 && currentNode - 1 >= 0) {
        currentNode = currentNode - 1;
        objective = pathNodesList.at(currentNode);
      }
      // bool lastNode = false;
      // if (currentNode == 0) {
      //   lastNode = true;
      // }
      SSLMotion::GoToPoint motion(objective,
                                  (field->enemyGoalInsideBottom() - robot->position()).angle() -
                                      3.14,
                                  true);
      SSLRobotCommand command(motion);
      command.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), command));
      break;
    }
    case 2: {
      // Verifica qual o aliado mais próximo a um determinado ponto do gol
      Robot closestAlly =
          *frame->allies().removedById(robot->id()).closestTo(field->enemyGoalOutsideTop());
      SSLMotion::RotateOnSelf m((closestAlly.position() - robot->position()).angle() - 0.03);
      SSLRobotCommand c(m);
      c.set_dribbler(true);
      if (abs(robot->angleTo(closestAlly)) <=
          0.07) { // Se o robô estiver virado para esse aliado, então ele faz o passe
        c.set_dribbler(false);
        c.set_front(true);
        c.set_kickSpeed(2);
      }
      emit sendCommand(sslNavigation.run(robot.value(), c));
      break;
    }
    case 3: {
      vector<Point> pathNodes = vector<Point>();
      Point targ = Point(1.0, 1.0);
      if (!target.isNull()) {              // Verifica se existe um target, para evitar erros
        if (targ.distTo(target) >= 20.0) { // Se o meu objetivo não está perto do target, então eu
                                           // preciso executar o algoritmo (uma otimização para
                                           // evitar várias execuções do algoritmo no mesmo caso)
          SSLMotion::GoToPoint motion(field->enemyPenaltyAreaCenter(),
                                      (field->center() - robot->position()).angle(),
                                      true);
          SSLRobotCommand command(motion);
          emit sendCommand(sslNavigation.run(robot.value(), command));
          RRTSTAR* rrtstar = new RRTSTAR;
          receiveTarget(targ); // Muda o target para ser o objetivo do movimento
          // Configura o algoritmo para a configuração inicial
          rrtstar->setInitialPos(robot->position());
          rrtstar->setFinalPos(Point(0.0, 0.0));
          rrtstar->nodes.clear();
          rrtstar->initialize();
          // Os dois loops são para adicionar os obstáculos, que são os robôs inimigos
          for (int o = 1; o < frame->allies().size(); o++) {
            Point topRight = Point(frame->allies().at(o).position().x() + BOT_RADIUS,
                                   frame->allies().at(o).position().y() + BOT_RADIUS);
            Point bottomLeft = Point(frame->allies().at(o).position().x() - BOT_RADIUS,
                                     frame->allies().at(o).position().y() - BOT_RADIUS);
            rrtstar->obstacles->addObstacle(topRight, bottomLeft);
          }
          for (int o = 0; o < frame->enemies().size(); o++) {
            Point topRight = Point(frame->enemies().at(o).position().x() + BOT_RADIUS,
                                   frame->enemies().at(o).position().y() + BOT_RADIUS);
            Point bottomLeft = Point(frame->enemies().at(o).position().x() - BOT_RADIUS,
                                     frame->enemies().at(o).position().y() - BOT_RADIUS);
            rrtstar->obstacles->addObstacle(topRight, bottomLeft);
          }
          // Configura o número máximo de iterações do algoritmo, mesmo que ele não use todas para
          // chegar ao objetivo
          rrtstar->setMaxIterations(2300);
          // Configura o tamanho máximo de distância entre pontos
          rrtstar->setStepSize(100);
          // RRTSTAR Algorithm
          rrtstar->RRTstarAlgorithm();
          // Gera o caminho mais curto entre o ponto de partida e o objetivo, dados os pontos
          // gerados pelo algoritmo
          pathNodesList = rrtstar->generatePath(pathNodes);
          // As próximas linhas desenham os pontos na tela
          pathKey.draw([path = pathNodesList](GameVisualizerPainter2D* f) {
            if (!path.empty()) {
              for (int i = 0; i < (int) path.size() - 1; ++i) {
                f->drawFilledCircle(path[i], 30, Color::Magenta);
                f->drawLine(path[i], path[i + 1], Color::Red, 10);
              }
              f->drawFilledCircle(path.back(), 30, Color::Magenta);
            }
          });
          // Configura o índice do nó inicial e o objetivo inicial do movimento
          currentNode = (int) pathNodesList.size() - 1;
          objective = pathNodesList.at(currentNode);
          delete rrtstar; // Deleta a instância de rrtstar para liberar memória
        }
      }
      // Muda o objetivo sempre que o robô estiver perto dele
      if (robot->position().distTo(objective) <= 150 && currentNode - 1 >= 0) {
        currentNode = currentNode - 1;
        objective = pathNodesList.at(currentNode);
      }
      // bool lastNode = false;
      // if (currentNode == 0) {
      //   lastNode = true;
      // }
      SSLMotion::GoToPoint motion(objective, (field->center() - robot->position()).angle(), true);
      SSLRobotCommand command(motion);
      emit sendCommand(sslNavigation.run(robot.value(), command));
      break;
    }
    case 4: {
      // Apenas fica parado virado para onde a bola está
      SSLMotion::RotateOnSelf motion((frame->ball().position() - robot->position()).angle());
      SSLRobotCommand command(motion);
      cout << "currentState default: " << currentState << endl;
      emit sendCommand(sslNavigation.run(robot.value(), command));
      break;
    }
    default: cout << "currentState default: " << currentState << endl;
  }
  // TODO: here...
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

static_block {
  Factory::processing.insert<CustomPlayer>();
};
