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
  virtual bool collide(Tetrahedron *tet) = 0;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
