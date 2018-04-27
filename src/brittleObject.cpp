#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <nanogui/nanogui.h>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>


#include "brittleObject.h"
#include "collision/plane.h"
#include "collision/sphere.h"

#define DAMPING_FACTOR 0.002
#define CG_ITERS 10

using namespace std;
using namespace Eigen;

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::SparseVector<double> SpVec;

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
  this->shattered = false;
}

BrittleObject::~BrittleObject() {
  tetrahedra.clear();
  constraints.clear();
}

// Returns whether the BrittleObject has collided with any of the collision objects
bool collided_with_any_object(
    vector<CollisionObject *> &collision_objects, 
    vector<Tetrahedron *> &tetrahedra, 
    Vector3D *adjustment) {
  for (CollisionObject *co : collision_objects) {
    for (Tetrahedron *tet : tetrahedra) {
      if (co->collide(tet, adjustment)) {
        return true;
      }
    }
  }
  return false;
}

// Verlet integration under effects of gravity
void moveObject(
    double delta_t, 
    BrittleObjectParameters *op,
    vector<Vector3D> &external_accelerations,
    vector<Tetrahedron *> &tetrahedra) {

  Vector3D external_force = Vector3D();
  for (Vector3D &external_acc : external_accelerations) {
    external_force += external_acc;
  }

  // Compute total force acting on each point mass.
  for (Tetrahedron *tet : tetrahedra) {
    tet->forces = external_force * tet->mass;

    // reset all vertex update booleans to false
    for (Vertex *v : tet->vertices) {
      v->updated = false;
    }
  }

  // Verlet integration to compute new point mass positions
  for (Tetrahedron *tet : tetrahedra) {
    Vector3D total_acc = tet->forces / tet->mass;

    Vector3D tet_dir = tet->position - tet->last_position;
    Vector3D new_tet_position = 
        tet->position + (1.0 - DAMPING_FACTOR) * tet_dir + total_acc * pow(delta_t, 2);
    tet->last_position = tet->position;
    tet->position = new_tet_position;

    // update vertices
    for (Vertex *v : tet->vertices) {
      if (!v->updated) {
        Vector3D v_dir = v->position - v->last_position;
        Vector3D new_v_position = 
            v->position + (1.0 - DAMPING_FACTOR) * v_dir + total_acc * pow(delta_t, 2);
        v->last_position = v->position;
        v->position = new_v_position;
        v->updated = true;
      }
    }
  }
}

// Adjust each point mass and vertex so that after intersection the object rests on the plane.
void adjustToPlane(vector<Tetrahedron *> &tetrahedra, Vector3D adjustment) {
  for (Tetrahedron *tet : tetrahedra) {
    tet->position = tet->last_position + adjustment;

    // update vertices
    for (Vertex *v : tet->vertices) {
      v->position = v->last_position + adjustment;
    }
  }
}

void BrittleObject::simulate(double frames_per_sec, double simulation_steps, BrittleObjectParameters *op,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  if (!shattered) {
    moveObject(delta_t, op, external_accelerations, tetrahedra);
    Vector3D adjustment = Vector3D();
    if (collided_with_any_object(*collision_objects, tetrahedra, &adjustment)) {
      shatter((*collision_objects)[0], delta_t);
      shattered = true;
      adjustToPlane(tetrahedra, adjustment);
    }
  }
}

void BrittleObject::reset(double fall_height) {

  Vector3D height_additive (0., fall_height, 0.);
  for (Tetrahedron *tet : tetrahedra) {
    for (Vertex *v : tet->vertices) {
      v->updated = false;
    }
  }
  for (Tetrahedron *tet : tetrahedra) {
    tet->position = tet->start_position + height_additive;
    tet->last_position = tet->start_position + height_additive;
    for (Vertex *v : tet->vertices) {
      if (!v->updated) {
        v->position = v->start_position + height_additive;
        v->last_position = v->start_position + height_additive;
        v->updated = true;
      }
    }
  }

  shattered = false;
  for (Constraint *c : constraints) {
    c->broken = false;
  }
}

void BrittleObject::shatter(CollisionObject *collision_object, double delta_t) {
  cout << "shattering \n";
  VectorXd Q(3* tetrahedra.size());
  for (int i = 0; i < tetrahedra.size(); i++) {
    Tetrahedron *tet = tetrahedra[i];
    Vector3D adjustment = Vector3D();
    if (collision_object->collide(tet, &adjustment)) {
      Vector3D impact_force = collision_object->impact_force(tet, delta_t);
      Q(3*i) = impact_force.x;
      Q(3*i+1) = impact_force.y;
      Q(3*i+2) = impact_force.z;
    }
  }
  cout << "built Q\n";
  // cout << Q;
  
  SpMat J(constraints.size(), 3*tetrahedra.size());
  J.setZero();
  for (int i = 0; i < constraints.size(); i++) {
    Constraint *c = constraints[i];
    Tetrahedron *tet_a = c->tet_a;
    Tetrahedron *tet_b = c->tet_b;
    int ja = tet_a->id;
    int jb = tet_b->id;
    double dist = (tet_a->position - tet_b->position).norm();
    J.insert(i,3*ja) = tet_a->position.x / dist;
    J.insert(i,3*ja+1) = tet_a->position.y / dist;
    J.insert(i,3*ja+2) = tet_a->position.z / dist;

    J.insert(i,3*jb) = -tet_b->position.x / dist;
    J.insert(i,3*jb+1) = -tet_b->position.y / dist;
    J.insert(i,3*jb+2) = -tet_b->position.z / dist;
  }
  cout << "built J\n";

  SpMat W(3*tetrahedra.size(), 3*tetrahedra.size());
  W.setZero();
  for (int j = 0; j < tetrahedra.size(); j++) {
    Tetrahedron *tet = tetrahedra[j];
    W.insert(3*j, 3*j) = 1.0 / tet->mass;
    W.insert(3*j+1, 3*j+1) = 1.0 / tet->mass;
    W.insert(3*j+2, 3*j+2) = 1.0 / tet->mass;
  }
  cout << "built W\n";
  SpMat A = J * W * J.transpose();
  ConjugateGradient<SpMat, Lower|Upper> cg;
  cg.compute(A);
  cg.setTolerance(0.00001);
  cg.setMaxIterations(1000);

  for (int i = 0; i < CG_ITERS; i++) {
    VectorXd Q_iter = VectorXd(Q);
    if (i < 0.8 * CG_ITERS) {
      Q_iter = (double) i / 0.8 * CG_ITERS * Q_iter;
    }
    else {
      Q_iter = (double) (CG_ITERS - i) / 1.0 * CG_ITERS * Q_iter;
    }
    VectorXd B = -1.0f * J * W * Q_iter;
    // cg.setMaxIterations(500);
    cout << "running solver\n";
    VectorXd x = cg.solve(B);
    for (int i = 0; i < constraints.size(); i++) {
      Constraint *c = constraints[i];
      Tetrahedron *tet_a = c->tet_a;
      Tetrahedron *tet_b = c->tet_b;
      int ja = tet_a->id;
      int jb = tet_b->id;
      double fx = Q.coeff(3*ja) + Q.coeff(3*ja);
      double fy = Q.coeff(3*ja+1) + Q.coeff(3*ja+1);
      double fz = Q.coeff(3*ja+2) + Q.coeff(3*ja+2);
      Vector3D total_force(fx,fy,fz);
      Vector3D constraint_dir = (tet_a->position - tet_b->position).unit();
      double magnitude = dot(total_force, constraint_dir);
      if (magnitude > c->constraint_value) {
        c->broken = true;
        //TODO: Loop through neighbors and decrement their strengths for next iteration
      }
    } 
  }
}

