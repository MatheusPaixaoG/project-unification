#include "RRTstar.h"
#include <QRandomGenerator>
#include <iostream>
#include <random>

const double EPS = 1e-6;
bool pathFound = false;

RRTSTAR::RRTSTAR() {
  obstacles = new Obstacles;
  startPos.setX(START_POS_X);
  startPos.setY(START_POS_Y);
  endPos.setX(END_POS_X);
  endPos.setY(END_POS_Y);
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->orientation = START_ORIENT;
  root->cost = 0.0;
  lastNode = root;
  nodes.push_back(root);
  step_size = 18;
  max_iter = 3000;
}

/**
 * @brief Inicializa o nó raíz de RRT*
 */
void RRTSTAR::initialize() {
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->orientation = START_ORIENT;
  root->cost = 0.0;
  lastNode = root;
  nodes.push_back(root);
}

template <typename T> // Retorna um número aleatório no intervalo [low, high]
T randomCoordinate(T low, T high) {
  random_device random_device;
  mt19937 engine{random_device()};
  uniform_real_distribution<double> dist(low, high);
  return dist(engine);
}

/**
 * @brief Gera um nó aleatório no campo
 * @return
 */
Node* RRTSTAR::getRandomNode() {
  Node* ret;
  double random_sample = randomCoordinate(0.0, 1.0);
  Point point;
  if ((random_sample - GOAL_SAMPLING_PROB) <= EPS and !pathFound) {
    point = Point(END_POS_X, END_POS_Y) + Point(END_DIST_THRESHOLD, END_DIST_THRESHOLD);
  } else {
    point = Point(randomCoordinate(-WORLD_WIDTH / 2, WORLD_WIDTH / 2),
                  randomCoordinate(-WORLD_HEIGHT / 2, WORLD_HEIGHT / 2));
  }
  if (point.distTo(endPos) <= 250) {
    point = endPos;
  }
  float orient = (float) QRandomGenerator::global()->generateDouble() * 2 * 3.142;
  if (point.x() >= -WORLD_WIDTH / 2 && point.x() <= WORLD_WIDTH / 2 &&
      point.y() >= -WORLD_HEIGHT / 2 && point.y() <= WORLD_HEIGHT / 2 && orient > 0 &&
      orient < 2 * 3.142) {
    ret = new Node;
    ret->position = point;
    ret->orientation = orient;
    return ret;
  }
  return NULL;
}

/**
 * @brief Método auxiliar para achar a distancia entre dois pontos
 * @param p
 * @param q
 * @return
 */
double RRTSTAR::distance(Point& p, Point& q) {
  Point v = p - q;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Pega o nó mais próximo aa partir de uma dada configuração/posição
 * @param point
 * @return
 */
Node* RRTSTAR::nearest(Point point) {
  float minDist = 1e9;
  Node* closest = NULL;
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

/**
 * @brief Pega nós vizinhos de uma dada configuração/posição
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTSTAR::near(Point point, float radius, vector<Node*>& out_nodes) {
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < radius) {
      out_nodes.push_back(nodes[i]);
    }
  }
}

/**
 * @brief Acha uma configuração a uma distância step_size a partir do nó mais próximo para o nó
 * aleatório
 * @param q
 * @param qNearest
 * @return
 */
Point RRTSTAR::newConfig(Node* q, Node* qNearest) {
  Point to = q->position;
  Point from = qNearest->position;
  Point intermediate = to - from;
  intermediate = intermediate / intermediate.norm();
  Point pos = from + step_size * intermediate;
  Point ret(pos.x(), pos.y());
  return ret;
}

/**
 * @brief Retorna o custo da trajetória
 * @param q
 * @return
 */
double RRTSTAR::Cost(Node* q) {
  return q->cost;
}

/**
 * @brief Computa o custo do caminho
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTSTAR::PathCost(Node* qFrom, Node* qTo) {
  return distance(qTo->position, qFrom->position);
}

/**
 * @brief Adiciona o nó na árvore
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node* qNearest, Node* qNew) {
  qNew->parent = qNearest;
  qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
  qNearest->children.push_back(qNew);
  nodes.push_back(qNew);
  lastNode = qNew;
}

/**
 * @brief Checa se o último nó está próximo de endPos
 * @return
 */
bool RRTSTAR::reached() {
  if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD) {
    pathFound = true;
    return true;
  }
  return false;
}

void RRTSTAR::setStepSize(int step) {
  step_size = step;
}

void RRTSTAR::setMaxIterations(int iter) {
  max_iter = iter;
}

void RRTSTAR::setInitialPos(Point initialPos) {
  startPos.setX(initialPos.x());
  startPos.setY(initialPos.y());
}

void RRTSTAR::setFinalPos(Point finalPos) {
  endPos.setX(finalPos.x());
  endPos.setY(finalPos.x());
}

/**
 * @brief Deleta todos os nós usando DFS
 * @param root
 */
void RRTSTAR::deleteNodes(Node* root0) {
  for (int i = 0; i < (int) root0->children.size(); i++) {
    deleteNodes(root0->children[i]);
  }
  delete root0;
}

void RRTSTAR::RRTstarAlgorithm() {
  for (int i = 0; i < max_iter; i++) {
    Node* q = getRandomNode();
    if (q) {
      Node* qNearest = nearest(q->position);
      if (distance(q->position, qNearest->position) > step_size) {
        Point newConfigPosOrient;
        newConfigPosOrient = newConfig(q, qNearest);
        Point newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
        if (!obstacles->isSegmentInObstacle(newConfigPos, qNearest->position)) {
          Node* qNew = new Node;
          qNew->position = newConfigPos;
          qNew->orientation = newConfigPosOrient.angle();
          vector<Node*> Qnear;
          near(qNew->position, step_size * RRTSTAR_NEIGHBOR_FACTOR, Qnear);
          Node* qMin = qNearest;
          double cmin = Cost(qNearest) + PathCost(qNearest, qNew);
          for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
            Node* qNear = Qnear[j];
            if (!obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
                (Cost(qNear) + PathCost(qNear, qNew)) < cmin) {
              qMin = qNear;
              cmin = Cost(qNear) + PathCost(qNear, qNew);
            }
          }
          add(qMin, qNew);

          for (int j = 0; j < 0 || (unsigned) j < Qnear.size(); j++) {
            Node* qNear = Qnear[j];
            if (!obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
                (Cost(qNew) + PathCost(qNew, qNear)) < Cost(qNear)) {
              Node* qParent = qNear->parent;
              // Remove a aresta entre qParent e qNear
              qParent->children.erase(
                  std::remove(qParent->children.begin(), qParent->children.end(), qNear),
                  qParent->children.end());
              // Adiciona uma aresta entre qNew e qNear
              qNear->cost = Cost(qNew) + PathCost(qNew, qNear);
              qNear->parent = qNew;
              qNew->children.push_back(qNear);
            }
          }
        }
      }
    }
    if (reached()) {
      cout << "Reached destination" << endl;
      i = 0;
      setMaxIterations(200);
    }
  }
}

vector<Point> RRTSTAR::generatePath(vector<Point> pathNodes) {
  Node* q;
  if (reached()) {
    q = lastNode;
  } else {
    // Se ainda não chegou ao objetivo, o menor caminho vai começar do nó mais próximo de
    // endPos
    q = nearest(endPos);
    cout << "Exceeded max iterations!" << endl;
  }
  // Gera o menor caminho para o objetivo
  while (q != NULL) {
    pathNodes.push_back(q->position);
    path.push_back(q);
    q = q->parent;
  }
  return pathNodes;
}