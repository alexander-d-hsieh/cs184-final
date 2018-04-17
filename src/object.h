#ifndef OBJECT_H
#define OBJECT_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "objectMesh.h"
#include "collision/collisionObject.h"
#include "constraint.h"

using namespace CGL;
using namespace std;

// enum e_orientation { HORIZONTAL = 0, VERTICAL = 1 };

struct ObjectParameters {
  ObjectParameters() {}
  ObjectParameters(double damping,
                  double density, double ks)
      : damping(damping), density(density), ks(ks) {}
  ~ObjectParameters() {}

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Object {
  Object() {}
  Object(double width, double height, double depth, int num_width_points,
        int num_height_points, int num_depth_points);
  ~Object();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, ObjectParameters *op,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  // void buildObjectMesh();

  // void build_spatial_map();
  // void self_collide(PointMass &pm, double simulation_steps);
  // float hash_position(Vector3D pos);

  // Cloth properties
  double width;
  double height;
  double depth;
  int num_width_points;
  int num_height_points;
  int num_depth_points;
  // double thickness;

  // Cloth components
  vector<PointMass> point_masses;
  vector<Constraint> constraints;
  ObjectMesh *objectMesh;

  // Spatial hashing
  // unordered_map<float, vector<PointMass *> *> map;
};

#endif /* CLOTH_H */
