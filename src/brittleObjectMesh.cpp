#include "brittleObjectMesh.h"
#include <iostream>

using namespace CGL;
using namespace std;

Vector3D Triangle::normal() {
  // bool clockwise = (x0 - x1) * (y0 + y1) + (x1 - x2) * (y1 + y2) + (x2 - x0) * (y2 + y0) > 0;

  Vector3D v1 = this->v1->pos;
  Vector3D v2 = this->v2->pos;
  Vector3D v3 = this->v3->pos;

  Vector3D a = v2 - v1;
  Vector3D b = v3 - v1;

  Vector3D norm = cross(a, b) / dot(a, b);

  return norm;
};