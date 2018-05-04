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
#define CG_ITERS 100


using namespace std;
using namespace Eigen;

typedef Eigen::SparseMatrix<double> SpMat;

Triangle::Triangle(Vertex *v1, Vertex *v2, Vertex *v3, bool face) {
  this->v1 = v1;
  this->v2 = v2;
  this->v3 = v3;
  this->face = face;
  this->c = NULL;
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
  this->traversed = false;
  this->shard = -1;
}

Vertex::Vertex(Vertex *v) {
  this->position = v->position;
  this->last_position = v->last_position;
  this->start_position = v->start_position;
  this->id = v->id;
  this->updated = false;
}

BrittleObject::BrittleObject() {
  this->constraints = vector<Constraint *>();
  this->tetrahedra = vector<Tetrahedron *>();
  this->start_position = Vector3D();
  this->position = Vector3D();
  this->last_position = Vector3D();
  this->shattered = false;
  this->Q = VectorXd();
  this->Q_hat = VectorXd();
  this->J = SpMat();
  this->W = SpMat();
  this->shatter_iter = 0;
  this->shards = vector<vector<Tetrahedron *>>();
  this->forces = vector<Vector3D>();
}

BrittleObject::~BrittleObject() {
  tetrahedra.clear();
  constraints.clear();
}

// Returns matrix for rotation around x-axis
Matrix3x3 rotate_x_axis(float deg) {
  // Part 3: Fill this in.
  return Matrix3x3({1.0, 0.0, 0.0,
                   0.0, cos(deg * PI / 180), -1 * sin(deg * PI / 180),
                   0.0, sin(deg * PI / 180), cos(deg * PI / 180)});
}

// Returns matrix for rotation around y-axis
Matrix3x3 rotate_y_axis(float deg) {
  // Part 3: Fill this in.
  return Matrix3x3({cos(deg * PI / 180), 0.0, sin(deg * PI / 180),
                   0.0, 1.0, 0.0,
                   -1 * sin(deg * PI / 180), 0.0, cos(deg * PI / 180)});
}

// Returns matrix for rotation around z-axis
Matrix3x3 rotate_z_axis(float deg) {
  // Part 3: Fill this in.
  return Matrix3x3({cos(deg * PI / 180.0), -1.0 * sin(deg * PI / 180.0), 0.0,
                   sin(deg * PI / 180.0), cos(deg * PI / 180.0), 0.0,
                   0.0, 0.0, 1.0});
}

// Returns whether the BrittleObject has collided with any of the collision objects
bool collided_with_any_object(
    vector<CollisionObject *> &collision_objects,
    Vector3D lowest_point,
    Vector3D *adjustment) {
  for (CollisionObject *co : collision_objects) {
    if (co->collide(lowest_point, adjustment)) {
      return true;
    }
  }
  return false;
}

// Verlet integration under effects of gravity

Vector3D moveObject(
    double delta_t, 
    BrittleObjectParameters *op,
    vector<Vector3D> &external_accelerations,
    vector<Tetrahedron *> &tetrahedra) {

  Vector3D lowest_point = Vector3D(MAXFLOAT);
  Vector3D external_force = Vector3D();
  for (Vector3D &external_acc : external_accelerations) {
    external_force += external_acc;
  }

  // Compute total force acting on each point mass.
  for (Tetrahedron *tet : tetrahedra) {
    tet->forces = external_force * tet->mass;

//    // reset all vertex update booleans to false
//    for (Vertex *v : tet->vertices) {
//      v->updated = false;
//    }
  }

  // Verlet integration to compute new point mass positions
  for (Tetrahedron *tet : tetrahedra) {
    Vector3D total_acc = tet->forces / tet->mass;

    Vector3D tet_dir = tet->position - tet->last_position;
    Vector3D new_tet_position = 
        tet->position + (1.0 - DAMPING_FACTOR) * tet_dir + total_acc * pow(delta_t, 2);
    tet->last_position = tet->position;
    tet->position = new_tet_position;
    if (tet->position.y < lowest_point.y) {
      lowest_point = tet->position;
    }

    // update vertices
    for (Vertex *v : tet->vertices) {
//      if (!v->updated) {
        Vector3D v_dir = v->position - v->last_position;
        Vector3D new_v_position = 
            v->position + (1.0 - DAMPING_FACTOR) * v_dir + total_acc * pow(delta_t, 2);
        v->last_position = v->position;
        v->position = new_v_position;
        if (v->position.y < lowest_point.y) {
          lowest_point = v->position;
        }
//        v->updated = true;
//      }
    }
  }
  return lowest_point;
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
    Vector3D lowest_point = moveObject(delta_t, op, external_accelerations, tetrahedra);
    Vector3D adjustment = Vector3D();
    if (collided_with_any_object(*collision_objects, lowest_point, &adjustment)) {
      build_shatter_matrices((*collision_objects)[0], delta_t);
      shattered = true;
      adjustToPlane(tetrahedra, adjustment);
    }
  }
  if (shattered && shatter_iter < CG_ITERS) {
    shatter((*collision_objects)[0], delta_t);
  }
  else if (shatter_iter == CG_ITERS) {
    explode();
    shatter_iter++;
    for (int i = 0; i < shards.size(); i++) {
      vector<Vector3D> force = {forces[i]};
      moveObject(delta_t, op, force, shards[i]);
    }
//  }
  } else {
    for (int i = 0; i < shards.size(); i++) {
      vector<Vector3D> force = {forces[i]};
      moveObject(delta_t, op, force, shards[i]);
    }
  }
}

void BrittleObject::explode() {
  int shard_num = -1;
  vector<Tetrahedron *> stack = vector<Tetrahedron *>();
  for (Tetrahedron* tetra : tetrahedra) {
    if (!tetra->traversed) {
      stack.push_back(tetra);
      vector<Tetrahedron *> shard = vector<Tetrahedron *>();
      shard_num++;
      while (stack.size() != 0) {
        Tetrahedron *tet = stack.back();
        stack.pop_back();
        if (!tet->traversed) {
          tet->traversed = true;
          tet->shard = shard_num;
          shard.push_back(tet);
          for (int i = 0; i < tet->triangles.size(); i++) {
            Triangle *t = tet->triangles[i];
            if (t->face) continue;
            Tetrahedron *neighbor = t->pair->tet;
            Constraint *c = t->c;
            if (c == NULL) continue;
            else if (!c->broken) {
              stack.push_back(neighbor);
            }
            else if (c->broken) {
              t->face = true;
            }
          }
        }
      }
      shards.push_back(shard);
    }
  }

  for (vector<Tetrahedron*> shard : shards) {
    Vector3D center = Vector3D();
    for (Tetrahedron *tet : shard) {
      center += tet->position;
      tet->last_position = tet->position;
    }
    center /= shard.size();
    forces.push_back(center);
//    cout << "center force: " << center << "\n";
  }
}

void BrittleObject::reset(BrittleObjectParameters *op) {
  Matrix3x3 x_rotate = rotate_x_axis(op->rotation.x);
  Matrix3x3 y_rotate = rotate_y_axis(op->rotation.y);
  Matrix3x3 z_rotate = rotate_z_axis(op->rotation.z);
  Matrix3x3 rotation = x_rotate * y_rotate * z_rotate;
  Vector3D height_additive (0., op->fall_height, 0.);
//  for (Tetrahedron *tet : tetrahedra) {
//    for (Vertex *v : tet->vertices) {
//      v->updated = false;
//    }
//  }
  for (Tetrahedron *tet : tetrahedra) {
    tet->position = (rotation * tet->start_position) + height_additive;
    tet->last_position = (rotation * tet->start_position) + height_additive;
    for (Vertex *v : tet->vertices) {
      if (v->id == 240) {
        cout << "here\n";
      }
//      if (!v->updated) {
        v->position = (rotation * v->start_position) + height_additive;
        v->last_position = (rotation * v->start_position) + height_additive;
//        v->updated = true;
//      }
    }
  }

  shattered = false;
  shatter_iter = 0;
  for (Constraint *c : constraints) {
    c->start_constraint_value = c->volume_constraint + op->constraint_strength_additive;
    c->constraint_value = c->start_constraint_value;
    c->broken = false;
  }
}

void BrittleObject::build_shatter_matrices(CollisionObject *collision_object, double delta_t) {
  cout << "shattering \n";

  // update constraints
  Q = VectorXd(3 * tetrahedra.size());
  Q_hat = VectorXd(3 * tetrahedra.size());
  Q_hat.setZero();
  Q.setZero();
  for (int i = 0; i < tetrahedra.size(); i++) {
    Tetrahedron *tet = tetrahedra[i];
    int id = tet->id;
    Vector3D adjustment = Vector3D();
    if (collision_object->collide(tet, &adjustment)) {
      Vector3D impact_force = collision_object->impact_force(tet, delta_t);
      Q(3 * id) += impact_force.x;
      Q(3 * id + 1) += impact_force.y;
      Q(3 * id + 2) += impact_force.z;
    }

  }
  cout << "built Q\n";
  // cout << Q;

  J = SpMat(constraints.size(), 3 * tetrahedra.size());
  J.setZero();
  for (int i = 0; i < constraints.size(); i++) {
    Constraint *c = constraints[i];
    Tetrahedron *tet_a = c->tet_a;
    Tetrahedron *tet_b = c->tet_b;
    int ja = tet_a->id;
    int jb = tet_b->id;
    double dist = 0.5 * (tet_a->position - tet_b->position).norm();
    J.insert(i, 3 * ja) = tet_a->position.x / dist;
    J.insert(i, 3 * ja + 1) = tet_a->position.y / dist;
    J.insert(i, 3 * ja + 2) = tet_a->position.z / dist;

    J.insert(i, 3 * jb) = -tet_b->position.x/ dist;
    J.insert(i, 3 * jb + 1) = -tet_b->position.y/ dist;
    J.insert(i, 3 * jb + 2) = -tet_b->position.z/ dist;
  }
  cout << "built J\n";

  W = SpMat(3 * tetrahedra.size(), 3 * tetrahedra.size());
  W.setZero();
  for (int j = 0; j < tetrahedra.size(); j++) {
    Tetrahedron *tet = tetrahedra[j];
    int jid = tet->id;
    W.insert(3 * jid, 3 * jid) = 1.0 / tet->mass;
    W.insert(3 * jid + 1, 3 * jid + 1) = 1.0 / tet->mass;
    W.insert(3 * jid + 2, 3 * jid + 2) = 1.0 / tet->mass;
  }
  cout << "built W\n";
}

void BrittleObject::shatter(CollisionObject *collision_object, double delta_t) {
  SpMat A = J * W * J.transpose();
  ConjugateGradient<SpMat, Lower | Upper> cg;
  cg.compute(A);
  cg.setTolerance(0.01);
  vector<int> num_broken_constraints = vector<int>();
  vector<Constraint *> broken_constraints = vector<Constraint *>();
  int i = ++shatter_iter;
  A = J * W * J.transpose();
  cg.compute(A);
  VectorXd Q_iter = VectorXd(3 * tetrahedra.size());
  if (i < 0.8 * CG_ITERS) {
    Q_iter = ((double) i / (0.8 * CG_ITERS)) * Q;
    cout << "Q strength: " << ((double) i / (0.8 * CG_ITERS)) << endl;
  }
  else {
    Q_iter = ((double) (CG_ITERS - i) / (0.2 * CG_ITERS)) * Q;
    cout << "Q strength: " << ((double) (CG_ITERS - i) / (0.2 * CG_ITERS)) << endl;
  }
  Q_iter += Q_hat;
  // Q += Q_hat;
  VectorXd B = -1.0 * J * W * Q_iter;
  cout << "running solver\n";
  VectorXd x = cg.solve(B);
  Q_hat = J.transpose() * x;
  broken_constraints.clear();
  int broken_this_iter = 0;
  for (int j = 0; j < constraints.size(); j++) {
    Constraint *c = constraints[j];
    if (c->broken) continue;

    Tetrahedron *tet_a = c->tet_a;
    Tetrahedron *tet_b = c->tet_b;
    int ja = tet_a->id;
    int jb = tet_b->id;
    double fx = Q_hat.coeff(3 * ja);
    double fy = Q_hat.coeff(3 * ja + 1);
    double fz = Q_hat.coeff(3 * ja + 2);
    Vector3D force_a(fx, fy, fz);
    fx = Q_hat.coeff(3 * jb);
    fy = Q_hat.coeff(3 * jb + 1);
    fz = Q_hat.coeff(3 * jb + 2);
    Vector3D force_b(fx, fy, fz);
    Vector3D b_to_a = (tet_a->position - tet_b->position).unit();

    double magnitude_a = dot(force_a, -b_to_a);
    double magnitude_b = dot(force_b, b_to_a);
    if (magnitude_a + magnitude_b > c->constraint_value) {
      c->broken = true;
      broken_constraints.push_back(c);
      broken_this_iter++;
    }
    else if (magnitude_a + magnitude_b < -8.0*c->constraint_value) {
      c->broken = true;
      broken_constraints.push_back(c);
      broken_this_iter++;
    }

  }
  // crack growth
  int weakened = 0;
  for (int k = 0; k < broken_constraints.size(); k++) {
    Constraint *c = broken_constraints[k];

    Tetrahedron *tet_a = c->tet_a;
    Tetrahedron *tet_b = c->tet_b;
    Vector3D a_to_b = tet_b->position - tet_a->position;
    for (Triangle *t : tet_a->triangles) {
      Constraint *neighbor_c = t->c;
      if (neighbor_c != NULL && c != neighbor_c && !neighbor_c->broken) {
        Tetrahedron *tet = t->pair->tet;
        Vector3D v1 = tet_b->position - tet_a->position;
        Vector3D v2 = tet->position - tet_a->position;
        double theta = acos(dot(v1, v2) / (v1.norm() * v2.norm()));
        neighbor_c->constraint_value = neighbor_c->constraint_value * (0.5005 - (0.4995 * sin(4.0 * theta + (PI / 2.0))));
        weakened++;
        if (theta < PI / 2.0) {
          neighbor_c->constraint_value = neighbor_c->constraint_value * (0.5005 - (0.4995 * sin(4.0 * theta + (PI / 2.0))));
          weakened++;
        }
      }
    }
    for (Triangle *t : tet_b->triangles) {
      Constraint *neighbor_c = t->c;
      if (neighbor_c != NULL && c != neighbor_c && !neighbor_c->broken) {
        Tetrahedron *tet = t->pair->tet;
        Vector3D v1 = tet_a->position - tet_b->position;
        Vector3D v2 = tet->position - tet_b->position;
        double theta = acos(dot(v1, v2) / (v1.norm() * v2.norm()));
        neighbor_c->constraint_value = neighbor_c->constraint_value * (0.5005 - (0.4995 * sin(4.0 * theta + (PI / 2.0))));
        weakened++;
        if (theta < PI / 2.0) {
          neighbor_c->constraint_value = neighbor_c->constraint_value * (0.5005 - (0.4995 * sin(4.0 * theta + (PI / 2.0))));
          weakened++;
        }
      }
    }
  }

  cout << "broken " << broken_this_iter << endl;
  cout << "weakened " << weakened << endl;
  
}

