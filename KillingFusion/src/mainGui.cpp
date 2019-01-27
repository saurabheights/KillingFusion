//
// Created by Saurabh Khanduja on 18.10.18.
//
#include <string>
#include <vector>

#include "KillingFusion.h"
#include "DatasetReader.h"
#include "config.h"

// OPENGL Stuff
#include <GL/glut.h>

// Mesh to draw
#include "SimpleMesh.h"

// Screenshot of drawn mesh
#include "FreeImage.h"

static void reshape(int width, int height)
{
  // Set the viewport to cover the new window
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION); // To operate on the Projection matrix
  glLoadIdentity();            // Reset
}

/* initialize OpenGL settings */
static void initGL(int width, int height)
{
  // reshape(width, height);

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);

  glEnable(GL_DEPTH_TEST);

  glDepthFunc(GL_LEQUAL);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}

void renderMesh(SimpleMesh *mesh)
{
  // glEnable(GL_CULL_FACE);
  // glCullFace(GL_BACK);

  glBegin(GL_TRIANGLES);
  std::vector<Vertex> vertices = mesh->GetVertices();
  std::vector<Triangle> triangles = mesh->GetTriangles();
  for (Triangle triangle : triangles)
  {
    Vertex &v0 = vertices[triangle.idx0];
    Vertex &v1 = vertices[triangle.idx1];
    Vertex &v2 = vertices[triangle.idx2];
    v0(0) = -v0(0);
    v1(0) = -v1(0);
    v2(0) = -v2(0);
    v0(1) = -v0(1);
    v1(1) = -v1(1);
    v2(1) = -v2(1);
    // The negative sign is hack, since in SDF Z value increases inwards and in Opengl, its outwards.
    Vertex normal = -(v1 - v0).cross(v2 - v1).normalized();
    glNormal3fv(normal.data());
    glVertex3fv(v0.data());
    glVertex3fv(v1.data());
    glVertex3fv(v2.data());
  }
  glEnd();
}

void renderImage(cv::Mat image)
{
  if (!image.isContinuous())
    image = image.clone();

  cv::Mat flippedImage;
  cv::flip(image, flippedImage, 0); // Opencv 0,0 is on topleft but opengl has 0,0 on bottom left
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glRasterPos2i(0, 0);
  glDrawPixels(flippedImage.cols, flippedImage.rows, GL_RGB, GL_UNSIGNED_BYTE, flippedImage.data);
}

void renderDepthImage(cv::Mat depthImage)
{
  double min = datasetDepthMinMaxValues[datasetType][0], max = datasetDepthMinMaxValues[datasetType][1];
  cv::Mat adjMap;

  // Contrast Enhancement.
  float scale = 255 / (max - min);
  depthImage.convertTo(adjMap, CV_8UC1, scale, -min * scale);

  // Apply color map
  cv::Mat falseColorsMap;
  applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);
  if (!falseColorsMap.isContinuous())
    falseColorsMap = falseColorsMap.clone();

  renderImage(falseColorsMap);
}

void displayText(int x, int y, int r, int g, int b, int screenWidth, int screenHeight, std::string str)
{
  // Switch to window coordinates to render
  glDisable(GL_TEXTURE_2D);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, screenWidth, 0, screenHeight);

  glDisable(GL_LIGHTING);
  glRasterPos2i(x, y); // or wherever in window coordinates
  for (int i = 0; i < str.length(); i++)
  {
    glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
  }
  glEnable(GL_LIGHTING);
  
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glEnable(GL_TEXTURE_2D);
}

DatasetReader *datasetReader;
KillingFusion *fusion;

static void draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  int screenWidth = glutGet(GLUT_SCREEN_WIDTH);
  int screenHeight = glutGet(GLUT_SCREEN_HEIGHT);
  glViewport(0, 0, screenWidth, screenHeight);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity(); //Resets to identity Matrix.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Get the frame index to be processed next
  int currentFrameIndex = fusion->getCurrentFrameIndex();

  // Read the color image and the depth frame for the corresponding frame index
  std::vector<cv::Mat> colorDepthImageVec = datasetReader->getImages(currentFrameIndex);
  cv::Mat colorResizedImage, depthResizedImage;
  cv::resize(colorDepthImageVec[0], colorResizedImage, cv::Size(320, 240));
  cv::resize(colorDepthImageVec[1], depthResizedImage, cv::Size(320, 240));

  // Render Image in proper viewport and
  int TextHeight = 50; // In Pixels

  // Image - Color Image
  // Text - Color Image
  // Image - Depth Image
  // Text - Depth Image
  glViewport(0, screenHeight / 2 + TextHeight, colorResizedImage.cols, colorResizedImage.rows);
  renderImage(colorResizedImage);
  glViewport(0, 0, screenWidth, screenHeight);
  displayText(colorResizedImage.cols/2, screenHeight / 2 + TextHeight, 128, 128, 128, screenWidth, screenHeight, "Input Frame");
  glViewport(0, TextHeight, depthResizedImage.cols, depthResizedImage.rows);
  renderDepthImage(depthResizedImage);
  glViewport(0, 0, screenWidth, screenHeight);
  displayText(colorResizedImage.cols/2, TextHeight, 128, 128, 128, screenWidth, screenHeight, "Depth Frame");

  SimpleMesh *mesh = fusion->processNextFrame();

  // SDF Z axis is different from opengl. and somehow it also affects y-axis.
  gluLookAt(0, 0, -2,
            0, 0, -0.5,
            0, 1, 0);

  // Render on a Square ViewPort on top right
  glViewport(screenWidth / 2, 0, screenHeight / 2, screenHeight / 2);
  glOrtho(-1, 1, -1, 1, 0, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  renderMesh(mesh);

  // Render second mesh on a Square ViewPort on bottom right
  std::cout << mesh->getMinLoc().transpose() << " " << mesh->getMaxLoc().transpose() << std::endl;
  glViewport(screenWidth / 2, screenHeight / 2, screenHeight / 2, screenHeight / 2);
  glOrtho(-1, 1, -1, 1, 0, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  renderMesh(mesh);


  glutSwapBuffers();
  glutPostRedisplay();

  // Save Screenshot
  // Convert to FreeImage format & save to file
  glFinish(); // When doing heavy processing, the glReadPixels does not load all data.
  BYTE *pixels = new BYTE[3 * screenWidth * screenHeight];
  glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);
  glFinish();
  FIBITMAP *image = FreeImage_ConvertFromRawBits(pixels, screenWidth, screenHeight, 3 * screenWidth, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);
  std::string VoxelSizeStr = std::to_string(VoxelSize);
  std::string MergeStrategy = std::to_string(FUSE_BY_MERGE);
  std::string filename = std::to_string(currentFrameIndex) + "-Data-LS-Fuse-" + MergeStrategy + "-Result-VoxelSize_" + VoxelSizeStr + ".png";
  FreeImage_Save(FIF_PNG, image, (OUTPUT_DIR + outputDir[datasetType] + filename).c_str(), 0);
  FreeImage_Unload(image);
  delete[] pixels;
  delete mesh;
}

int main(int argc, char **argv)
{
  datasetReader = new DatasetReader(DATA_DIR);
  fusion = new KillingFusion(*datasetReader);

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutCreateWindow("KillingFusion");
  glutFullScreen();

  // register glut call backs
  glutReshapeFunc(reshape);
  glutDisplayFunc(draw);
  initGL(800, 600);

  // Run Main Loop
  glutMainLoop();
  delete datasetReader;
  delete fusion;
  return 0;
}
