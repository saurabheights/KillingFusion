//
// Created by Saurabh Khanduja on 18.10.18.
//
#include <string>

#include "KillingFusion.h"
#include "DatasetReader.h"
#include "config.h"

// OPENGL Stuff
#include <cstdio>
#include <cstdint>
#include "Transform.h"
#include <GL/glut.h>

// Mesh to draw
#include "SimpleMesh.h"

// Screenshot of drawn mesh
#include "FreeImage.h"

enum PlayerCameraState
{
  PCS_MOVING_FORWARD = 1 << 0,
  PCS_MOVING_BACKWARD = 1 << 1,
  PCS_MOVING_LEFT = 1 << 2,
  PCS_MOVING_RIGHT = 1 << 3,
};

static unsigned camera_state = 0;
static Transform camera(
    Vec3f(0, 0, 0),
    Quat(0, 0, 0, -1));
static bool rotating_camera = false;

static Quat mouse_rotate(const Quat &in, float x, float y, float sensitivity)
{
  const Quat xq(Vec3f_Y(), -x * sensitivity);
  const Quat yq(Vec3f_X(), -y * sensitivity);
  return xq * (in * yq);
}

static void get_camera_vectors(Vec3f *look_dir, Vec3f *up, Vec3f *right, const Quat &orient)
{
  assert (look_dir != nullptr);
  assert (up != nullptr);
  assert (right != nullptr);
  const Mat4 m = to_mat4(inverse(orient));
  *right = {m[0], m[4], m[8]};
  *up = {m[1], m[5], m[9]};
  *look_dir = {-m[2], -m[6], -m[10]};
}

static Vec3f get_walk_direction()
{
  constexpr float sincos_45 = 0.7071067f;

  Vec3f look_dir, up, right;
  get_camera_vectors(&look_dir, &up, &right, camera.orientation);

  float fb_move = 0.0f;
  float lr_move = 0.0f;
  if (camera_state & PCS_MOVING_FORWARD)
    fb_move += 0.10f;
  if (camera_state & PCS_MOVING_BACKWARD)
    fb_move -= .10f;
  if (camera_state & PCS_MOVING_LEFT)
    lr_move -= .10f;
  if (camera_state & PCS_MOVING_RIGHT)
    lr_move += .10f;

  if (camera_state & (PCS_MOVING_FORWARD | PCS_MOVING_BACKWARD) &&
      camera_state & (PCS_MOVING_LEFT | PCS_MOVING_RIGHT))
  {
    fb_move *= sincos_45;
    lr_move *= sincos_45;
  }

  return look_dir * Vec3f(fb_move) + right * Vec3f(lr_move);
}

//----------------------------------------------------------------------------
// GLUT
//----------------------------------------------------------------------------

static void keyboardDown(unsigned char key, int x, int y)
{
  Vec3f look_dir, up, right;

  switch (key)
  {
  case 'Q':
  case 'q':
  case 27:
    exit(0);
    break;
  case 'w':
    camera_state |= PCS_MOVING_FORWARD;
    break;
  case 'a':
    camera_state |= PCS_MOVING_LEFT;
    break;
  case 's':
    camera_state |= PCS_MOVING_BACKWARD;
    break;
  case 'd':
    camera_state |= PCS_MOVING_RIGHT;
    break;
  case 'p':
    printf("pos: %f %f %f\n", VEC3(camera.translation));
    printf("orient %f %f %f %f\n", VEC4(camera.orientation));
    break;
  }
}

static void keyboardUp(unsigned char key, int x, int y)
{
  switch (key)
  {
  case 'w':
    camera_state &= ~PCS_MOVING_FORWARD;
    break;
  case 'a':
    camera_state &= ~PCS_MOVING_LEFT;
    break;
  case 's':
    camera_state &= ~PCS_MOVING_BACKWARD;
    break;
  case 'd':
    camera_state &= ~PCS_MOVING_RIGHT;
    break;
  }
}

static void keyboardSpecialDown(int k, int x, int y)
{
}

static void keyboardSpecialUp(int k, int x, int y)
{
}

static void reshape(int width, int height)
{
  GLfloat fieldOfView = 90.0f;
  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fieldOfView, (GLfloat)width / height, 0.1, 500.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

static void mouseClick(int button, int state, int x, int y)
{
  rotating_camera = state == GLUT_DOWN;
}

static void mouseMotion(int x, int y)
{
  static int last_x = 0, last_y = 0;
  const int dx = x - last_x;
  const int dy = y - last_y;
  last_x = x;
  last_y = y;

  if (rotating_camera)
    camera.orientation = mouse_rotate(camera.orientation, dx, dy, 0.25);
}

/* Axes display list */
static GLuint axes_list;

static void draw_axes()
{
  /* Create a display list for drawing axes */
  axes_list = glGenLists(1);
  glNewList(axes_list, GL_COMPILE);

  glColor4ub(0, 0, 255, 255);
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.75f, 0.25f, 0.0f);
  glVertex3f(0.75f, -0.25f, 0.0f);
  glVertex3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0.75f, 0.0f, 0.25f);
  glVertex3f(0.75f, 0.0f, -0.25f);
  glVertex3f(1.0f, 0.0f, 0.0f);
  glEnd();
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.0f, 0.75f, 0.25f);
  glVertex3f(0.0f, 0.75f, -0.25f);
  glVertex3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0.25f, 0.75f, 0.0f);
  glVertex3f(-0.25f, 0.75f, 0.0f);
  glVertex3f(0.0f, 1.0f, 0.0f);
  glEnd();
  glBegin(GL_LINE_STRIP);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.25f, 0.0f, 0.75f);
  glVertex3f(-0.25f, 0.0f, 0.75f);
  glVertex3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0.0f, 0.25f, 0.75f);
  glVertex3f(0.0f, -0.25f, 0.75f);
  glVertex3f(0.0f, 0.0f, 1.0f);
  glEnd();

  glColor4ub(255, 255, 0, 255);
  glRasterPos3f(1.1f, 0.0f, 0.0f);

  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'x');
  glRasterPos3f(0.0f, 1.1f, 0.0f);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'y');
  glRasterPos3f(0.0f, 0.0f, 1.1f);
  glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'z');

  glEndList();
}

static void idle()
{
}

/* initialize OpenGL settings */
static void initGL(int width, int height)
{
  reshape(width, height);

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}

DatasetReader* datasetReader;
KillingFusion* fusion;

static void draw()
{
  static int last_time = 0;
  const int current_time = glutGet(GLUT_ELAPSED_TIME);
  const float delta = float(current_time - last_time) / 1000.0f;
  camera.translation += get_walk_direction() * Vec3f(delta) * Vec3f(0.05);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glLoadMatrixf(to_mat4(inverse(camera)).data);

  // LIGHT SETUP HERE
  const Vec3f tmp = normalize(Vec3f(0, 0, 1));
  const Vec4f light_dir(tmp.x, tmp.y, tmp.z, 0);
  const Vec4f ambient(0.4, 0.4, 0.4, 1);
  const Vec4f diffuse(1, 1, 1, 1);
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient.data);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse.data);
  glLightfv(GL_LIGHT0, GL_POSITION, light_dir.data);

  // RENDER HERE
  glPushMatrix();
  draw_axes();
  glPopMatrix();

  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  glBegin(GL_TRIANGLES);
  // Read the mesh from KillingFusion
  int currentFrameIndex = fusion->getCurrentFrameIndex();
  SimpleMesh* mesh = fusion->processNextFrame();
  std::vector<Vertex> vertices = mesh->GetVertices();
  std::vector<Triangle> triangles = mesh->GetTriangles();
  for (Triangle triangle : triangles)
  {
    glColor3f(0.5, 0, 0);
    Vertex &v0 = vertices[triangle.idx0];
    Vertex &v1 = vertices[triangle.idx1];
    Vertex &v2 = vertices[triangle.idx2];
    v0(1) = -v0(1);
    v1(1) = -v1(1);
    v2(1) = -v2(1);
    Vertex normal = (v1 - v0).cross(v2 - v1).normalized();
    glNormal3fv(normal.data());
    glVertex3fv(v0.data());
    glNormal3fv(normal.data());
    glVertex3fv(v1.data());
    glNormal3fv(normal.data());
    glVertex3fv(v2.data());
  }
  glEnd();
  glutSwapBuffers();
  glutPostRedisplay();

  // Convert to FreeImage format & save to file
  glFinish();  
  int width = 800;
  int height = 600;
  BYTE* pixels = new BYTE[3 * width * height];
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
  glFinish();
  FIBITMAP* image = FreeImage_ConvertFromRawBits(pixels, width, height, 3 * width, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);
  FreeImage_Save(FIF_PNG, image, ("SnapShot" + std::to_string(currentFrameIndex) + ".png").c_str(), 0);
  FreeImage_Unload(image);
  delete [] pixels;
  delete mesh;
}

int main(int argc, char **argv)
{
  datasetReader = new DatasetReader(DATA_DIR);
  fusion = new KillingFusion(*datasetReader);

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(800, 600);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("Perspective's GLUT Template");

  // register glut call backs
  glutKeyboardFunc(keyboardDown);
  glutKeyboardUpFunc(keyboardUp);
  glutSpecialFunc(keyboardSpecialDown);
  glutSpecialUpFunc(keyboardSpecialUp);
  glutMouseFunc(mouseClick);
  glutMotionFunc(mouseMotion);
  glutPassiveMotionFunc(mouseMotion);
  glutReshapeFunc(reshape);
  glutDisplayFunc(draw);
  glutIdleFunc(idle);
  glutIgnoreKeyRepeat(true); // ignore keys held down
  initGL(800, 600);

  glutMainLoop();
  return 0;
}
