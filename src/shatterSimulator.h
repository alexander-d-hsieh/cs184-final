#ifndef CGL_SHATTER_SIMULATOR_H
#define CGL_SHATTER_SIMULATOR_H

#include <nanogui/nanogui.h>

#include "camera.h"
#include "brittleObject.h"
#include "collision/collisionObject.h"

using namespace nanogui;

class ShatterSimulator {
public:
  ShatterSimulator(Screen *screen);
  ~ShatterSimulator();

  void init();

  void loadObject(BrittleObject *brittle_object);
  void loadBrittleObjectParameters(BrittleObjectParameters *op);
  void loadCollisionObjects(vector<CollisionObject *> *objects);
  virtual bool isAlive();
  virtual void drawContents();

  // Screen events

  virtual bool cursorPosCallbackEvent(double x, double y);
  virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
  virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
  virtual bool dropCallbackEvent(int count, const char **filenames);
  virtual bool scrollCallbackEvent(double x, double y);
  virtual bool resizeCallbackEvent(int width, int height);

private:
  virtual void initGUI(Screen *screen);
  void drawWireframeConstraints(GLShader &shader);
  void drawWireframeCracks(GLShader &shader);
  void drawPhong(GLShader &shader);

  // Camera methods

  virtual void resetCamera();
  virtual Matrix4f getProjectionMatrix();
  virtual Matrix4f getViewMatrix();

  // Default simulation values

  int frames_per_sec = 90;
  int simulation_steps = 1;

  CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);
  nanogui::Color color = nanogui::Color(1.0f, 0.0f, 0.0f, 1.0f);

  BrittleObject *brittle_object;
  BrittleObjectParameters *op;
  vector<CollisionObject *> *collision_objects;

  // OpenGL attributes

  enum e_shader { WIREFRAME_CONSTRAINTS = 0, WIREFRAME_CRACKS = 1, PHONG = 2 };
  e_shader activeShader = WIREFRAME_CONSTRAINTS;

  vector<GLShader> shaders;

  GLShader wireframeConstraintsShader;
  GLShader wireframeCracksShader;
  GLShader phongShader;

  // Camera attributes

  CGL::Camera camera;
  CGL::Camera canonicalCamera;

  double view_distance;
  double canonical_view_distance;
  double min_view_distance;
  double max_view_distance;

  double scroll_rate;

  // Screen methods

  Screen *screen;
  void mouseLeftDragged(double x, double y);
  void mouseRightDragged(double x, double y);
  void mouseMoved(double x, double y);

  // Mouse flags

  bool left_down = false;
  bool right_down = false;
  bool middle_down = false;

  // Keyboard flags

  bool ctrl_down = false;

  // Simulation flags

  bool is_paused = false;

  // Screen attributes

  int mouse_x;
  int mouse_y;

  int screen_w;
  int screen_h;

  bool is_alive = true;

  Vector2i default_window_size = Vector2i(1024, 800);

  IntBox<int> *num_broken_constraints_box;
  IntBox<int> *num_satisfied_constraints_box;
};

#endif // CGL_CLOTH_SIM_H
