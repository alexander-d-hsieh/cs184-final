#include "iostream"
#include <nanogui/nanogui.h>

#include "../brittleObject.h"
#include "../shatterSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

bool Plane::collide(Tetrahedron *tet) {
  // TODO (Part 3.2): Handle collisions with planes.
  Vector3D last_pos_dir = tet->last_position - point;
  Vector3D pos_dir = tet->position - point;
  double dot1 = dot(last_pos_dir, normal);
  double dot2 = dot(pos_dir, normal);
  bool intersects = (dot1 <= 0 && dot2 > 0) || (dot1 >= 0 && dot2 < 0) || (dot1 < 0 && dot2 >= 0) || (dot1 > 0 && dot2 <= 0);
  // if (intersects) {
  //   double t = dot(point - tet->position, normal) / dot(-normal, normal);
  //   Vector3D tangent = tet->position + t * -normal;
  //   Vector3D correction = tangent - tet->last_position + (SURFACE_OFFSET * normal);
  //   tet->position = tet->last_position + correction * (1.0 - friction);
  // }
  return intersects;
}

double Plane::impact_force(Tetrahedron *tet, double delta_t) {
  // TODO (Part 3.2): Handle collisions with planes.
  return dot(normal, tet->last_position - tet->position) * delta_t;
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 4 * (sCross + sParallel);
  positions.col(1) << sPoint + 4 * (sCross - sParallel);
  positions.col(2) << sPoint + 4 * (-sCross + sParallel);
  positions.col(3) << sPoint + 4 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("in_color", false) != -1) {
    shader.setUniform("in_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  shader.uploadAttrib("in_normal", normals);

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
