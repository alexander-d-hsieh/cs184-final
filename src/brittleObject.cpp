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

Tetrahedron::Tetrahedron(Triangle *t1, Triangle *t2, Triangle *t3, Triangle *t4,
                              Vertex *v1, Vertex *v2, Vertex *v3, Vertex *v4) {
  this->t1 = t1;
  this->t2 = t2;
  this->t3 = t3;
  this->t4 = t4;

  Vector3D center = Vector3D();
  center += v1->pos;
  center += v2->pos;
  center += v3->pos;
  center += v4->pos;

  center /= 4.0;

  this->pm = new PointMass(center);
}

BrittleObject::BrittleObject() {
  this->constraints = vector<Constraint *>();
  this->point_masses = vector<PointMass *>();
}

BrittleObject::BrittleObject(double width, double height, double depth, int num_width_points,
            int num_height_points, int num_depth_points) {
  this->width = width;
  this->height = height;
  this->depth = depth;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->num_depth_points = num_depth_points;
  // this->thickness = thickness;

  buildGrid();

}

BrittleObject::~BrittleObject() {
  point_masses.clear();
  constraints.clear();
}

void BrittleObject::buildGrid() {
  // // TODO (Part 1.1): Build a grid of masses.
  // double width_spacing = width / (num_width_points - 1);
  // double height_spacing = height / (num_height_points - 1);
  // for (int j = 0; j < num_height_points; j ++) {
  //    for (int i = 0; i < num_width_points; i ++) {
  //     double x, y, z;
  //     if (orientation == HORIZONTAL) {
  //       y = 1.0;
  //       x = i * width_spacing;
  //       z = j * height_spacing;
  //     } else {
  //       double random = (double) rand();
  //       z = (random / RAND_MAX) / 500.0 - (1.0 / 1000.0);
  //       x = i * width_spacing;
  //       y = j * height_spacing;
  //     }
  //     Vector3D coord = Vector3D(x, y, z);
  //     bool pin = false;
  //     for (int k = 0; k < pinned.size(); k++) {
  //       if (pinned[k][0] == i && pinned[k][1] == j) {
  //         pin = true;
  //         break;
  //       }
  //     }
  //     point_masses.emplace_back(coord, pin);
  //   }
  // }
  // // TODO (Part 1.2): Add springs
  // PointMass *first = &point_masses[0];
  // for (int y = 0; y < num_height_points;y++) {
  //   for (int x = 0; x < num_width_points; x++) {
  //     int index = y * num_width_points + x;
  //     PointMass *pm = first + index;
  //     if (x != 0) {
  //       PointMass *left = pm - 1;
  //       springs.emplace_back(left, pm, STRUCTURAL);
  //     }
  //     if (y != 0) {
  //       PointMass *above = pm - num_width_points;
  //       springs.emplace_back(above, pm, STRUCTURAL);
  //     }
  //     if (x != 0 && y != 0) {
  //       PointMass *upper_left = pm - num_width_points - 1;
  //       springs.emplace_back(upper_left, pm, SHEARING);
  //     }
  //     if (x != num_width_points - 1 && y != 0) {
  //       PointMass *upper_right = pm - num_width_points + 1;
  //       springs.emplace_back(upper_right, pm, SHEARING);
  //     }
  //     if (x > 1) {
  //       PointMass *two_to_left = pm - 2;
  //       springs.emplace_back(two_to_left, pm, BENDING);
  //     }
  //     if (y > 1) {
  //       PointMass *two_above = pm - (2 * num_width_points);
  //       springs.emplace_back(two_above, pm, BENDING);
  //     }
  //   }
  // }
}

void BrittleObject::simulate(double frames_per_sec, double simulation_steps, BrittleObjectParameters *op,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = 0.0;
  // double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2.1): Compute total force acting on each point mass.
  Vector3D external_force = Vector3D();
  for (Vector3D &external_acc : external_accelerations) {
    external_force += mass * external_acc;
  }
  for (PointMass *pm : point_masses) {
    pm->forces = Vector3D();
    pm->forces += external_force;
  }
  // for (Spring &s : springs) {
  //   if ((!cp->enable_structural_constraints && s.spring_type == STRUCTURAL) ||
  //       (!cp->enable_shearing_constraints && s.spring_type == SHEARING) ||
  //       (!cp->enable_bending_constraints && s.spring_type == BENDING)) {
  //     continue;
  //   }
  //   double dist = (s.pm_a->position - s.pm_b->position).norm();
  //   Vector3D spring_corr_force = cp->ks * (dist - s.rest_length) * (s.pm_a->position - s.pm_b->position).unit();
  //   if (s.spring_type == BENDING) {
  //     spring_corr_force *= 0.2;
  //   }
  //   s.pm_a->forces -= spring_corr_force;
  //   s.pm_b->forces += spring_corr_force;
  // }

  // // TODO (Part 2.2): Use Verlet integration to compute new point mass positions
  // for (PointMass &pm : point_masses) {
  //   if (pm.pinned) {
  //     continue;
  //   }
  //   Vector3D total_acc = pm.forces / mass;
  //   double d = cp->damping / 100;
  //   Vector3D dir = pm.position - pm.last_position;
  //   Vector3D new_pos = pm.position + (1.0 - d) * dir + total_acc * pow(delta_t, 2);
  //   pm.last_position = pm.position;
  //   pm.position = new_pos;
  // }


  // // This won't do anything until you complete Part 4.
  // build_spatial_map();
  // for (PointMass &pm : point_masses) {
  //   self_collide(pm, simulation_steps);
  // }

  // // This won't do anything until you complete Part 3.
  // for (PointMass &pm : point_masses) {
  //   for (CollisionObject *co : *collision_objects) {
  //     co->collide(pm);
  //   }
  // }


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

// void Object::build_spatial_map() {
//   for (const auto &entry : map) {
//     delete(entry.second);
//   }
//   map.clear();

//   // TODO (Part 4.2): Build a spatial map out of all of the point masses.
//   for (PointMass &pm : point_masses) {
//     float hash = hash_position(pm.position);
//     vector<PointMass *> *v;
//     if (map.count(hash) == 0) {
//       v = new vector<PointMass *>();
//       v->push_back(&pm);
//       map[hash] = v;
//     } else {
//       v = map[hash];
//       v->push_back(&pm);
//     }
//   }
// }

// void Cloth::self_collide(PointMass &pm, double simulation_steps) {
//   // TODO (Part 4.3): Handle self-collision for a given point mass.
//   Vector3D final_correction = Vector3D();
//   int count = 0;
//   float hash = hash_position(pm.position);
//   vector<PointMass *> *v = map[hash];
//   for (PointMass *p : *v) {
//     if (p == &pm) {
//       continue;
//     }
//     Vector3D dir = pm.position - p->position;
//     if (dir.norm() < 2.0 * thickness) {
//       Vector3D correct_point = p->position + (dir.unit() * 2 * thickness);
//       Vector3D correction = correct_point - pm.position;
//       final_correction += correction;
//       count++;
//     }
//   }
//   if (count > 0) {
//     final_correction /= (count * simulation_steps);
//     pm.position += final_correction;
//   }
// }

// float Cloth::hash_position(Vector3D pos) {
//   // TODO (Part 4.1): Hash a 3D position into a unique float identifier that represents
//   // membership in some uniquely identified 3D box volume.
//   float w = 3.0 * width / num_width_points;
//   float h = 3.0 * height / num_height_points;
//   float t = max(w, h);
//   float x = floor(pos.x / w);
//   float y = floor(pos.y / h);
//   float z = floor(pos.z / t);
//   float hash = (x * 31 + y) * 31 + z;
//   return hash;
// }

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void BrittleObject::reset() {
  PointMass *pm = point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

// void Object::buildObjectMesh() {
//   if (point_masses.size() == 0) return;

//   ObjectMesh *objectMesh = new ObjectMesh();
//   vector<Triangle *> triangles;

//   // Create vector of triangles
//   for (int y = 0; y < num_height_points - 1; y++) {
//     for (int x = 0; x < num_width_points - 1; x++) {
//       PointMass *pm = &point_masses[y * num_width_points + x];
//       // Both triangles defined by vertices in counter-clockwise orientation
//       triangles.push_back(new Triangle(pm, pm + num_width_points, pm + 1));
//       triangles.push_back(new Triangle(pm + 1, pm + num_width_points,
//                                        pm + num_width_points + 1));
//     }
//   }

//   // For each triangle in row-order, create 3 edges and 3 internal halfedges
//   for (int i = 0; i < triangles.size(); i++) {
//     Triangle *t = triangles[i];

//     // Allocate new halfedges on heap
//     Halfedge *h1 = new Halfedge();
//     Halfedge *h2 = new Halfedge();
//     Halfedge *h3 = new Halfedge();

//     // Allocate new edges on heap
//     Edge *e1 = new Edge();
//     Edge *e2 = new Edge();
//     Edge *e3 = new Edge();

//     // Assign a halfedge pointer to the triangle
//     t->halfedge = h1;

//     // Assign halfedge pointers to point masses
//     t->pm1->halfedge = h1;
//     t->pm2->halfedge = h2;
//     t->pm3->halfedge = h3;

//     // Update all halfedge pointers
//     h1->edge = e1;
//     h1->next = h2;
//     h1->pm = t->pm1;
//     h1->triangle = t;

//     h2->edge = e2;
//     h2->next = h3;
//     h2->pm = t->pm2;
//     h2->triangle = t;

//     h3->edge = e3;
//     h3->next = h1;
//     h3->pm = t->pm3;
//     h3->triangle = t;
//   }

//   // Go back through the cloth mesh and link triangles together using halfedge
//   // twin pointers

//   // Convenient variables for math
//   int num_height_tris = (num_height_points - 1) * 2;
//   int num_width_tris = (num_width_points - 1) * 2;

//   bool topLeft = true;
//   for (int i = 0; i < triangles.size(); i++) {
//     Triangle *t = triangles[i];

//     if (topLeft) {
//       // Get left triangle, if it exists
//       if (i % num_width_tris != 0) { // Not a left-most triangle
//         Triangle *temp = triangles[i - 1];
//         t->pm1->halfedge->twin = temp->pm3->halfedge;
//       } else {
//         t->pm1->halfedge->twin = nullptr;
//       }

//       // Get triangle above, if it exists
//       if (i >= num_width_tris) { // Not a top-most triangle
//         Triangle *temp = triangles[i - num_width_tris + 1];
//         t->pm3->halfedge->twin = temp->pm2->halfedge;
//       } else {
//         t->pm3->halfedge->twin = nullptr;
//       }

//       // Get triangle to bottom right; guaranteed to exist
//       Triangle *temp = triangles[i + 1];
//       t->pm2->halfedge->twin = temp->pm1->halfedge;
//     } else {
//       // Get right triangle, if it exists
//       if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
//         Triangle *temp = triangles[i + 1];
//         t->pm3->halfedge->twin = temp->pm1->halfedge;
//       } else {
//         t->pm3->halfedge->twin = nullptr;
//       }

//       // Get triangle below, if it exists
//       if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
//         Triangle *temp = triangles[i + num_width_tris - 1];
//         t->pm2->halfedge->twin = temp->pm3->halfedge;
//       } else {
//         t->pm2->halfedge->twin = nullptr;
//       }

//       // Get triangle to top left; guaranteed to exist
//       Triangle *temp = triangles[i - 1];
//       t->pm1->halfedge->twin = temp->pm2->halfedge;
//     }

//     topLeft = !topLeft;
//   }

//   clothMesh->triangles = triangles;
//   this->clothMesh = clothMesh;
// }
