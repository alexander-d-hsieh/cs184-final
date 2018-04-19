#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../brittleObject.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

// Forward declaration
class PointMass;

class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual void collide(PointMass &pm) = 0;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
