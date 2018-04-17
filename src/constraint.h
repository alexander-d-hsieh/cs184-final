#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <vector>

#include "CGL/CGL.h"
#include "pointMass.h"

using namespace std;

namespace CGL {

struct Constraint {
  Constraint(PointMass *a, PointMass *b)
      : pm_a(a), pm_b(b) {
    constraint_value = 0;
  }

  Constraint(PointMass *a, PointMass *b, double constraint_value)
      : pm_a(a), pm_b(b), constraint_value(constraint_value) {}

  double constraint_value;

  PointMass *pm_a;
  PointMass *pm_b;
}; // struct Constraint
}
#endif /* CONSTRAINT_H */
