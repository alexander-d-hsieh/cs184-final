#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "brittleObject.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Triangle::Triangle(Vertex *v1, Vertex *v2, Vertex *v3, bool face) {
  this->v1 = v1;
  this->v2 = v2;
  this->v3 = v3;
  this->face = face;

  this->tetrahedra = vector<Tetrahedron *>();
}

Vector3D Triangle::normal(Vector3D camera_pos) {
  Vector3D v1 = this->v1->position;
  Vector3D v2 = this->v2->position;
  Vector3D v3 = this->v3->position;

  Vector3D a = v2 - v1;
  Vector3D b = v3 - v1;

  bool clockwise = dot(camera_pos, cross(a, b) / dot(a, b)) >= 0;
  return clockwise ? cross(a, b) / dot(a, b) : cross(b, a) / dot(a, b);
};

Tetrahedron::Tetrahedron(Triangle *t1, Triangle *t2, Triangle *t3, Triangle *t4,
                              Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4, double density, int index) {
  triangles = vector<Triangle *>();
  triangles.push_back(t1);
  triangles.push_back(t2);
  triangles.push_back(t3);
  triangles.push_back(t4);

  vertices = vector<Vertex *>();
  vertices.push_back(v1);
  vertices.push_back(v2);
  vertices.push_back(v3);
  vertices.push_back(v4);

  Vector3D center = Vector3D();
  center += v1->position;
  center += v2->position;
  center += v3->position;
  center += v4->position;

  center /= 4.0;

  this->volume = abs(dot(v1->position - v4->position, cross(v2->position - v4->position, v3->position - v4->position))) / 6.0;
  this->mass = this->volume * density;
  this->start_position = center;
  this->position = center;
  this->last_position = center;
  this->id = index;
}


BrittleObject::BrittleObject() {
  this->constraints = vector<Constraint *>();
  this->tetrahedra = vector<Tetrahedron *>();
  this->start_position = Vector3D();
  this->position = Vector3D();
  this->last_position = Vector3D();
}

BrittleObject::~BrittleObject() {
  tetrahedra.clear();
  constraints.clear();
}

void BrittleObject::simulate(double frames_per_sec, double simulation_steps, BrittleObjectParameters *op,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // Compute total force acting on each point mass.
  for (Tetrahedron *tet : tetrahedra) {
    Vector3D external_force = Vector3D();
    for (Vector3D &external_acc : external_accelerations) {
      external_force += external_acc;
    }
    tet->forces = external_force * tet->mass;

    // reset all vertex update booleans to false
    for (Vertex *v : tet->vertices) {
      v->updated = false;
    }
  }

  double damping = 0.2 / 100.0;

  // Verlet integration to compute new point mass positions
  for (Tetrahedron *tet : tetrahedra) {
    Vector3D total_acc = tet->forces / tet->mass;

    Vector3D tet_dir = tet->position - tet->last_position;
    Vector3D new_tet_position = tet->position + (1.0 - damping) * tet_dir + total_acc * pow(delta_t, 2);
    tet->last_position = tet->position;
    tet->position = new_tet_position;

    // update vertices
    for (Vertex *v : tet->vertices) {
      if (!v->updated) {
        Vector3D v_dir = v->position - v->last_position;
        Vector3D new_v_position = v->position + (1.0 - damping) * v_dir + total_acc * pow(delta_t, 2);
        v->last_position = v->position;
        v->position = new_v_position;
        v->updated = true;
      }
    }
  }

//
//     // Detect collision with plane
//     for (PointMass *pm : point_masses) {
//       for (CollisionObject *co : *collision_objects) {
//         co->collide(pm);
//       }
//     }


  // // TODO (Part 2.3): Constrain the changes to be such that the spring does not change
  // // in length more than 10% per timestep [Provot 1995].
  // for (Spring &s : springs) {
  //   double max_len = s.rest_length * 1.10;
  //   Vector3D dir = s.pm_a->position - s.pm_b->position;
  //   if (dir.norm() > max_len) {
  //     if (s.pm_a->pinned) {
  //       s.pm_b->position = s.pm_a->position - (dir.unit() * max_len);
  //     } else if (s.pm_b->pinned) {
  //       s.pm_a->position = s.pm_b->position + (dir.unit() * max_len);
  //     } else {
  //       s.pm_a->position -= dir.unit() * ((dir.norm() - max_len) / 2);
  //       s.pm_b->position += dir.unit() * ((dir.norm() - max_len) / 2);
  //     }
  //   }
  // }
}

void BrittleObject::reset() {

  // Vector3D height_additive (0., fall_height, 0.);
  for (Tetrahedron *tet : tetrahedra) {
    for (Vertex *v : tet->vertices) {
      v->updated = false;
    }
  }
  for (Tetrahedron *tet : tetrahedra) {
    tet->position = tet->start_position;
    tet->last_position = tet->start_position;
    for (Vertex *v : tet->vertices) {
      if (!v->updated) {
        v->position = v->start_position;
        v->last_position = v->start_position;
        v->updated = true;
      }
    }
  }
}

// vector<BrittleObject> BrittleObject::shatter() {
//   vector<Tetrahedron *> collide_tet = vector<Tetrahedron *>();
//   for (Tetrahedron *tet : tetrahedra) {
//   }
// }
