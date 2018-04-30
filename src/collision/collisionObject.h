#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../brittleObject.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

// Forward declaration
class Tetrahedron;

class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual bool collide(Tetrahedron *tet, Vector3D *adjustment) = 0;
  virtual bool collide(Vector3D lowest_point, Vector3D *adjustment) = 0;
  virtual Vector3D impact_force(Tetrahedron *tet, double delta_t) = 0;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
