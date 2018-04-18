#ifndef OBJECT_MESH_H
#define OBJECT_MESH_H

#include <vector>

#include "CGL/CGL.h"
#include "pointMass.h"
#include "constraint.h"

using namespace CGL;
using namespace std;

class Triangle;
class Tetrahedron;

class Vertex {
public:
  Vertex(double x, double y, double z, int id) 
      : pos(x, y, z), id(id) {}
  Vector3D pos;
  int id; 
};

class Triangle {
public:
  Triangle(Vertex *v1, Vertex *v2, Vertex *v3, bool face)
      : v1(v1), v2(v2), v3(v3), face(face) {}
  // Static references to constituent mesh objects
  Vertex *v1, *v2, *v3;
  bool face;
  Constraint *c;
  vector<Tetrahedron *> tetrahedra;

  Vector3D normal();
}; // struct Triangle

class Tetrahedron {
public:
  Tetrahedron(Triangle *t1, Triangle *t2, Triangle *t3, Triangle *t4)
      : t1(t1), t2(t2), t3(t3), t4(t4) {}
  Triangle *t1;
  Triangle *t2;
  Triangle *t3;
  Triangle *t4;
  double volume;
  PointMass *pm;

  Tetrahedron* get_neighbor(Triangle *t);

};



// class Edge {
// public:
//   Halfedge *halfedge;
// }; // struct Edge

// class Halfedge {
// public:
//   Edge *edge;
//   Halfedge *next;
//   Halfedge *twin;
//   Triangle *triangle;
//   PointMass *pm;
// }; // struct Halfedge

class BrittleObjectMesh {
public:
  ~BrittleObjectMesh() {}

  vector<Tetrahedron *> tetrahedra;
}; // struct ClothMesh

#endif // OBJECT_MESH_H
