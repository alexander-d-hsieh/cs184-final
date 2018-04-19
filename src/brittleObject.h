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
      : pos(x, y, z), id(id) {}
  Vector3D pos;
  int id; 
};

class Triangle {
public:
  Triangle(Vertex *v1, Vertex *v2, Vertex *v3, bool face);
  // Static references to constituent mesh objects
  Vertex *v1, *v2, *v3;
  bool face;
  Constraint *c;
  vector<Tetrahedron *> tetrahedra;

  Vector3D normal();
};

class Tetrahedron {
public:
  Tetrahedron(Triangle *t1, Triangle *t2, Triangle *t3, Triangle *t4, 
              Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4);
  Triangle *t1;
  Triangle *t2;
  Triangle *t3;
  Triangle *t4;
  double volume;
  PointMass *pm;

  Tetrahedron* get_neighbor(Triangle *t);
};

class PointMass {
public:
  PointMass(Vector3D position)
      : start_position(position), position(position),
        last_position(position) {}

  Vector3D normal();
  Vector3D velocity(double delta_t);

  // static values
  Vector3D start_position;

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
  BrittleObjectParameters(double fall_height, double constraint_strength_additive)
      : fall_height(fall_height), 
        constraint_strength_additive(constraint_strength_additive) {}
  ~BrittleObjectParameters() {}

  double fall_height;
  double constraint_strength_additive;
};

struct BrittleObject {
  BrittleObject();
  BrittleObject(double width, double height, double depth, int num_width_points,
        int num_height_points, int num_depth_points);
  ~BrittleObject();

  void simulate(double frames_per_sec, double simulation_steps, BrittleObjectParameters *op,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();

  // void build_spatial_map();
  // void self_collide(PointMass &pm, double simulation_steps);
  // float hash_position(Vector3D pos);

  // Object components
  vector<PointMass *> point_masses;
  vector<Constraint *> constraints;

  // Spatial hashing
  // unordered_map<float, vector<PointMass *> *> map;
};

#endif /* CLOTH_H */
