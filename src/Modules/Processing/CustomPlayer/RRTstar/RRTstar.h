#ifndef PROJECT_UNIFICATION_RRTSTAR_H
#define PROJECT_UNIFICATION_RRTSTAR_H

#include "soccer-common/Extends/QPoint/ExtendsQPoint.h"
#include <math.h>
#include <qvectornd.h>
#include <random>
#include <type_traits>
#include <vector>
using namespace std;

#define ftype double

const ftype EPS = 1e-6;

Point stepNear(Point& p1, Point& p2, ftype DELTA) {
  if (abs(p1.distTo(p2)) - DELTA <= EPS) {
    return p2;
  } else {
    ftype theta = atan2(p2.y() - p1.y(), p2.x() - p1.x());
    return Point(p1.x() + DELTA * cos(theta), p1.y() + DELTA * sin(theta));
  }
}

ftype minimumDistance(Point v, Point w, Point p) {
  ftype l2 = v.distTo(w);
  l2 *= l2;
  if (l2 < EPS) {
    return p.distTo(v);
  }
  const ftype t = max(0.0, min(1.0, Geometry2D::dot(p - v, w - v)));

  Point projection = v + t * (w - v);
  return p.distTo(projection);
}

bool checkCollision(Point lineFrom, Point lineTo, Point location, ftype radius) {
  location += Point(radius, radius);
  ftype ab2, acab, h2;
  Point ac = location - lineFrom;
  Point ab = lineTo - lineFrom;
  ab2 = Geometry2D::dot(ab, ab);
  acab = Geometry2D::dot(ac, ab);
  ftype t = acab / ab2;

  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }

  Point h = ((ab * t) + lineFrom) - location;
  h2 = Geometry2D::dot(h, h);
  return (h2 <= (radius * radius));
}

bool pointInPolygon(Point point, const QVector<Point>& polygon) {
  return Geometry2D::pointInPolygon(polygon, point);
}

int sign(const ftype x) {
  return x >= 0 ? x ? 1 : 0 : -1;
}

bool intersectOnLine(ftype a, ftype b, ftype c, ftype d) {
  if ((a - b) > EPS) {
    swap(a, b);
  }
  if ((c - d) > EPS) {
    swap(c, d);
  }
  return max(a, c) <= min(b, d);
}

bool checkIntersection(const Point a, const Point b, const Point c, const Point d) {
  return Geometry2D::segmentsIntersect(a, b, c, d);
}

template <class PT>
bool lineSegmentIntersectPolygon(Point a, Point b, const QVector<PT>& polygon) {
  if (a.distTo(b) < EPS) {
    return pointInPolygon((a + b) / 2.0, polygon);
  }

  int num = polygon.size();
  QList<Point> points = polygon.toList();
  for (int i = 0; i < num; i++) {
    int next = i + 1;
    if (next == num) {
      next = 0;
    }
    if (checkIntersection(a, b, points.at(i), points.at(next))) {
      return true;
    }
  }
  return false;
}

#endif // PROJECT_UNIFICATION_RRTSTAR_H
