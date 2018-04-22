#include <getopt.h>
#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unordered_set>

#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "brittleObject.h"
#include "shatterSimulator.h"
#include "json.hpp"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ShatteringSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string OBJECT = "object";

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, OBJECT};

ShatterSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Shatter Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

void loadObjectsFromFile(string filename, BrittleObject *brittleObject, BrittleObjectParameters *op, vector<CollisionObject *>* coll_objects) {
  // Read JSON from file
  ifstream i(filename);
  json j;
  i >> j;

  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }
    json object = it.value();

    if (key == OBJECT) {
      string node_file, face_file, ele_file;
//      double fall_height, constraint_strength_additive, density;

      auto it_node_file = object.find("node");
      if (it_node_file != object.end()) {
        node_file = *it_node_file;
      } else {
        incompleteObjectError("object", "node");
      }

      auto it_face_file = object.find("face");
      if (it_face_file != object.end()) {
        face_file = *it_face_file;
      } else {
        incompleteObjectError("object", "face");
      }

      auto it_ele_file = object.find("ele");
      if (it_ele_file != object.end()) {
        ele_file = *it_ele_file;
      } else {
        incompleteObjectError("object", "ele");
      }

      auto it_fall_height_param = object.find("fall_height");
      if (it_fall_height_param != object.end()) {
        op->fall_height = *it_fall_height_param;
      } else {
        incompleteObjectError("object", "fall_height");
      }

      auto it_constraint_strength_additive_param = 
          object.find("constraint_strength_additive");
      if (it_constraint_strength_additive_param != object.end()) {
        op->constraint_strength_additive = *it_constraint_strength_additive_param;
      } else {
        incompleteObjectError("object", "constraint_strength_additive");
      }

      auto it_density_param =
          object.find("density");
      if (it_constraint_strength_additive_param != object.end()) {
        op->density = *it_density_param;
      } else {
        incompleteObjectError("object", "density");
      }

      ifstream node(node_file);
      ifstream face(face_file);
      ifstream ele(ele_file);


      // Initialize dummy variables used for parsing
      int d1, d2, d3;

      vector<Vertex *> vertices = vector<Vertex *>();

      int num_vertex;
      node >> num_vertex >> d1 >> d2 >> d3;
      double x, y, z;
      int vertex_index;

      while (node >> vertex_index >> x >> y >> z) {
        Vertex* v = new Vertex(x, y + op->fall_height, z, vertex_index);
        //TODO make vertices list
        vertices.push_back(v);
      }

      // unordered_map<vector<int>, Triangle *> triangle_map = unordered_map<vector<int>, Triangle *>();

      // int num_face;
      // face >> num_face >> d1;
      int face_index, v1, v2, v3, v4;
      // //TODO add unordered set of triangles
      // while (face >> face_index >> v1 >> v2 >> v3 >> d1) {
      //   std::vector<int> vec;
      //   vec.push_back(v1);
      //   vec.push_back(v2);
      //   vec.push_back(v3);
      //   std::sort( vec.begin(), vec.end() );
      //   v1 = vec[0];
      //   v2 = vec[1];
      //   v3 = vec[2];
      //   Triangle *t = new Triangle(vertices[v1], vertices[v2], vertices[v3], true);
      //   triangle_map[vec] = t;
      // }
      
      unordered_map<uint64_t, Triangle *> triangle_map = unordered_map<uint64_t, Triangle *>();
      vector<Triangle *> all_triangles = vector<Triangle *>();
      int num_tetra;
      ele >> num_tetra >> d1 >> d2;
      int ele_index;
      int lines = 0;

      while (ele >> ele_index >> v1 >> v2 >> v3 >> v4) {
        lines++;
        vector<uint64_t> vec = vector<uint64_t>();
        Triangle *t1, *t2, *t3, *t4;

        vec.push_back(v1);
        vec.push_back(v2);
        vec.push_back(v3);
        std::sort( vec.begin(), vec.end() );
        uint64_t hash = (((vec[0] << 20) + vec[1]) << 20) + vec[2];
        if (triangle_map.find(hash) == triangle_map.end()) {
          t1 = new Triangle(vertices[v1], vertices[v2], vertices[v3], false);
          triangle_map[hash] = t1;
          all_triangles.push_back(t1);
        } else {
          t1 = triangle_map[hash];
        }
        vec.clear();

        vec.push_back(v1);
        vec.push_back(v2);
        vec.push_back(v4);
        std::sort( vec.begin(), vec.end() );
        hash = (((vec[0] << 20) + vec[1]) << 20) + vec[2];
        if (triangle_map.find(hash) == triangle_map.end()) {
          t2 = new Triangle(vertices[v1], vertices[v2], vertices[v4], false);
          triangle_map[hash] = t2;
          all_triangles.push_back(t2);
        } else {
          t2 = triangle_map[hash];
        }
        vec.clear();

        vec.push_back(v1);
        vec.push_back(v3);
        vec.push_back(v4);
        std::sort( vec.begin(), vec.end() );
        hash = (((vec[0] << 20) + vec[1]) << 20) + vec[2];
        if (triangle_map.find(hash) == triangle_map.end()) {
          t3 = new Triangle(vertices[v1], vertices[v3], vertices[v4], false);
          triangle_map[hash] = t3;
          all_triangles.push_back(t3);
        } else {
          t3 = triangle_map[hash];
        }
        vec.clear();

        vec.push_back(v2);
        vec.push_back(v3);
        vec.push_back(v4);
        std::sort( vec.begin(), vec.end() );
        hash = (((vec[0] << 20) + vec[1]) << 20) + vec[2];
        if (triangle_map.find(hash) == triangle_map.end()) {
          t4 = new Triangle(vertices[v2], vertices[v3], vertices[v4], false);
          triangle_map[hash] = t4;
          all_triangles.push_back(t4);
        } else {
          t4 = triangle_map[hash];
        }
        vec.clear();
      
        Tetrahedron* tet = new Tetrahedron(t1, t2, t3, t4, vertices[v1], vertices[v2], vertices[v3], vertices[v4], op->density);
        t1->tetrahedra.push_back(tet);
        t2->tetrahedra.push_back(tet);
        t3->tetrahedra.push_back(tet);
        t4->tetrahedra.push_back(tet);
        brittleObject->point_masses.push_back(tet->pm);
      }

      vector<Triangle *> surface_triangles = vector<Triangle *>();
      int wrong_triangles = 0;
      for (Triangle *triangle: all_triangles) {
        if (triangle->tetrahedra.size() == 1) {
          triangle->face = true;
          surface_triangles.push_back(triangle);
        } else if (triangle->tetrahedra.size() == 2) {
          Tetrahedron *a = triangle->tetrahedra[0];
          Tetrahedron *b = triangle->tetrahedra[1];
          Constraint *c = new Constraint(a->pm, b->pm);
          brittleObject->constraints.push_back(c);
        } else {
          wrong_triangles++;
        }
      }
      cout << "total surface triangles is: " << surface_triangles.size() <<"\n";
      cout << "total triangles is: " << all_triangles.size() <<"\n";
      cout << "total constraints is: " << brittleObject->constraints.size() <<"\n";
      cout << "total pm is: " << brittleObject->point_masses.size() <<"\n";

      cout << "total triangles with too many tetrahedra : " << wrong_triangles <<"\n";

      

    } else if (key == PLANE) {
      Vector3D point, normal;
      double friction;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      Plane *p = new Plane(point, normal, friction);
      coll_objects->push_back(p);
    }
  }

  i.close();
}

int main(int argc, char **argv) {
  BrittleObject brittleObject;
  BrittleObjectParameters op;
  vector<CollisionObject *> coll_objects;

  if (argc == 1) { // No arguments, default initialization
    string default_file_name = "../scene/teapot.json";
    loadObjectsFromFile(default_file_name, &brittleObject, &op, &coll_objects);
  } else {
    int c;

    while ((c = getopt (argc, argv, "f:")) != -1) {
      switch (c) {
        case 'f':
          loadObjectsFromFile(optarg, &brittleObject, &op, &coll_objects);
          break;
        default:
          usageError(argv[0]);
      }
    }
  }

  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the object

  // Initialize the ClothSimulator object
  app = new ShatterSimulator(screen);
  app->loadObject(&brittleObject);
  app->loadBrittleObjectParameters(&op);
  app->loadCollisionObjects(&coll_objects);
  app->init();

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }

  return 0;
}
