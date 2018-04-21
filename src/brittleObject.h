#ifndef OBJECT_H
#define OBJECT_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include "collision/collisionObject.h"

using namespace CGL;
using namespace std;

class Vertex;
class Triangle;
class Tetrahedron;
class PointMass;
class Constraint;

class Vertex {
public:
  Vertex(double x, double y, double z, int id) 
      : pos(x, y, z), last_pos(x, y, z), id(id), updated(false) {}
  Vector3D pos;
  Vector3D last_pos;
  int id;
  bool updated;
};

class Triangle {
public:
  Triangle(Vertex *v1, Vertex *v2, Vertex *v3, bool face);
  // Static references to constituent mesh objects
  Vertex *v1, *v2, *v3;
  bool face;
  Constraint *c;
  vector<Tetrahedron *> tetrahedra;

  Vector3D normal(Vector3D camera_pos);
};

class Tetrahedron {
public:
  Tetrahedron(Triangle *t1, Triangle *t2, Triangle *t3, Triangle *t4, 
              Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4, double density);
  vector<Triangle *> triangles;
  vector<Vertex *> vertices;
  double volume;
  PointMass *pm;

  Tetrahedron* get_neighbor(Triangle *t);
};

class PointMass {
public:
  PointMass(Vector3D position, Tetrahedron *tetra, double density)
      : start_position(position),
        position(position),
        last_position(position),
        tetra(tetra) {
    computeVolume(density);
  }

  Vector3D normal();
  Vector3D velocity(double delta_t);
  void computeVolume(double density);

  // static values
  Vector3D start_position;
  Tetrahedron *tetra;
  double mass;

  // dynamic values
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;

};

class Constraint {
public:  
  Constraint(PointMass *a, PointMass *b)
      : pm_a(a), pm_b(b) {
    constraint_value = 0;
  }

  Constraint(PointMass *a, PointMass *b, double constraint_value)
      : pm_a(a), pm_b(b), constraint_value(constraint_value) {}

  double constraint_value;

  PointMass *pm_a;
  PointMass *pm_b;
};

struct BrittleObjectParameters {
  BrittleObjectParameters() {}
  BrittleObjectParameters(double fall_height, double constraint_strength_additive, double density)
      : fall_height(fall_height), 
        constraint_strength_additive(constraint_strength_additive),
        density(density) {}
  ~BrittleObjectParameters() {}

  double fall_height;
  double constraint_strength_additive;
  double density;
};

struct BrittleObject {
  BrittleObject();
  ~BrittleObject();

  void simulate(double frames_per_sec, double simulation_steps, BrittleObjectParameters *op,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset(double fall_height);

  // Object components
  vector<PointMass *> point_masses;
  vector<Constraint *> constraints;
  Vector3D start_position, position, last_position;
};

#endif /* CLOTH_H */
