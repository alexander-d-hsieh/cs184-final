#ifndef OBJECT_H
#define OBJECT_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <nanogui/nanogui.h>  
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"
#include "collision/collisionObject.h"

typedef Eigen::SparseMatrix<double> SpMat;

using namespace Eigen;
using namespace CGL;
using namespace std;

class Vertex;
class Triangle;
class Tetrahedron;
class Constraint;

class Vertex {
public:
  Vertex(double x, double y, double z, int id) 
      : position(x, y, z), last_position(x, y, z), start_position(x, y, z), id(id) {}
  Vertex(Vertex *v);

  Vector3D position;
  Vector3D last_position;
  Vector3D start_position;
  int id;
};

class Triangle {
public:
  Triangle(Vertex *v1, Vertex *v2, Vertex *v3, bool face);
  // Static references to constituent mesh objects
  Vertex *v1, *v2, *v3;
  bool face;
  Constraint *c;
//  vector<Tetrahedron *> tetrahedra;
  Tetrahedron *tet;
  Triangle *pair;

  Vector3D normal(Vector3D camera_pos);
};

class Tetrahedron {
public:
  Tetrahedron(Triangle *t1, Triangle *t2, Triangle *t3, Triangle *t4, 
              Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4, double density, int index);
  int id;
  vector<Triangle *> triangles;
  vector<Vertex *> vertices;
  double volume;
  int shard;

    // static values
  Vector3D start_position;
  double mass;

  // dynamic values
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;
  bool traversed;

  Tetrahedron* get_neighbor(Triangle *t);
  // void group(vector<Tetrahedron*> &new_brittle_obj);
};

class Constraint {
public:  
  Constraint(Tetrahedron *a, Tetrahedron *b, double constraint_value, bool broken)
      : tet_a(a), tet_b(b), constraint_value(constraint_value), start_constraint_value(constraint_value), 
        volume_constraint(constraint_value), broken(broken) {}

  Constraint(Tetrahedron *a, Tetrahedron *b)
      : tet_a(a), tet_b(b) {
    constraint_value = 0.0;
    start_constraint_value = 0.0;
    volume_constraint = 0.0;
  }

  Constraint(Tetrahedron *a, Tetrahedron *b, double constraint_value)
      : tet_a(a), tet_b(b), distance((a->position - b->position).norm()), 
        constraint_value(constraint_value), start_constraint_value(constraint_value), 
        volume_constraint(constraint_value) {}

  double constraint_value;
  double start_constraint_value;
  double volume_constraint;
  bool broken;
  Vector3D distance;
  Tetrahedron *tet_a;
  Tetrahedron *tet_b;
};

struct BrittleObjectParameters {
  BrittleObjectParameters() {}
  BrittleObjectParameters(double fall_height, double constraint_strength_additive, double density, Vector3D rotation)
      : fall_height(fall_height), 
        constraint_strength_additive(constraint_strength_additive),
        density(density),
        rotation(rotation) {}
  ~BrittleObjectParameters() {}

  double fall_height;
  double constraint_strength_additive;
  double density;
  Vector3D rotation;
};

struct BrittleObject {
  BrittleObject();
  ~BrittleObject();

  void simulate(double frames_per_sec, double simulation_steps, BrittleObjectParameters *op,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset(BrittleObjectParameters *op);

  // void shatter();
  void shatter(CollisionObject *collision_object, double delta_t);
  void build_shatter_matrices(CollisionObject *collision_object, double delta_t);
  void explode();

  // Object components
  vector<Tetrahedron *> tetrahedra;
  vector<Constraint *> constraints;
  vector<vector<Tetrahedron *>> shards;
  vector<Vector3D> forces;
  Vector3D start_position, position, last_position;
  bool shattered;
  VectorXd Q, Q_hat;
  SpMat J, W;
  int shatter_iter;
  Vector3D center;

};

Matrix3x3 rotate_x_axis(float deg);
Matrix3x3 rotate_y_axis(float deg);
Matrix3x3 rotate_z_axis(float deg);

#endif /* CLOTH_H */
