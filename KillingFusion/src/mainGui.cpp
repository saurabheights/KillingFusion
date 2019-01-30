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

#include <opencv2/core/utils/filesystem.hpp>

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
  reshape(width, height);
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
}

cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
{
  cv::Mat output;

  double h1 = dstSize.width * (input.rows / (double)input.cols);
  double w2 = dstSize.height * (input.cols / (double)input.rows);
  if (h1 <= dstSize.height)
  {
    cv::resize(input, output, cv::Size(dstSize.width, h1));
  }
  else
  {
    cv::resize(input, output, cv::Size(w2, dstSize.height));
  }

  int top = (dstSize.height - output.rows) / 2;
  int down = (dstSize.height - output.rows + 1) / 2;
  int left = (dstSize.width - output.cols) / 2;
  int right = (dstSize.width - output.cols + 1) / 2;

  cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor);

  return output;
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
    // The negative sign is hack, since in SDF Z value increases inwards and in Opengl, its outwards. Somehow also affects y-sign.
    Vertex normal = (v1 - v0).cross(v2 - v1).normalized();
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
  glRasterPos2i(-1, -1);
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
  const int font_margin = 8;
  const int font_char_pixel_wdith = 6;
  glRasterPos2i(x - str.length() * font_char_pixel_wdith, y + font_margin); // or wherever in window coordinates
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
std::string outputDirPath;
bool lastFrameSeen = false;

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

  // Resize to 1/3 of screen width, 1/2 of screen height-Text
  int TextHeight = 40; // In Pixels
  int elementWidth = screenWidth / 3;
  int elementHeight = screenHeight / 2 - TextHeight;
  cv::Mat colorResizedImage = resizeKeepAspectRatio(colorDepthImageVec[0], cv::Size(elementWidth, elementHeight), cv::Scalar(0, 0, 0));
  cv::Mat depthResizedImage = resizeKeepAspectRatio(colorDepthImageVec[1], cv::Size(elementWidth, elementHeight), cv::Scalar(0, 0, 0));

  // Render Image in proper viewport and the text
  // Image - Color Image
  // Text - Color Image
  // Image - Depth Image
  // Text - Depth Image
  glViewport(0, elementHeight + 2 * TextHeight, elementWidth, elementHeight);
  renderImage(colorResizedImage);

  glViewport(0, 0, screenWidth, screenHeight);
  std::stringstream name;
  name << "Input Frame" << std::setfill(' ') << std::setw(4) << currentFrameIndex;
  displayText(elementWidth / 2, elementHeight + TextHeight, 128, 128, 128, screenWidth, screenHeight, name.str().c_str());

  glViewport(0, TextHeight, elementWidth, elementHeight);
  renderDepthImage(depthResizedImage);

  glViewport(0, 0, screenWidth, screenHeight);
  std::stringstream().swap(name); // Clear the name stringstream
  name << "Depth Frame" << std::setfill(' ') << std::setw(4) << currentFrameIndex;
  displayText(elementWidth / 2, 0, 128, 128, 128, screenWidth, screenHeight, name.str().c_str());

  // Perform KillingFusion on currentFrame
  std::vector<SimpleMesh *> meshes = fusion->processNextFrame();

  // SDF Z axis is different from opengl. and somehow it also affects y-axis.
  gluLookAt(0, 0, -2,
            0, 0, -0.5,
            0, 1, 0);

  int minElementHeightAndWidth = MIN(elementHeight, elementWidth);

  // Render Current Frame SDF mesh on a Square ViewPort on top
  if (meshes[0] != nullptr)
  {
    glViewport(elementWidth, elementHeight + TextHeight, minElementHeightAndWidth, minElementHeightAndWidth);
    glOrtho(-1, 1, -1, 1, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    renderMesh(meshes[0]);
  }

  glViewport(0, 0, screenWidth, screenHeight);
  displayText(elementWidth * 1.5, elementHeight + TextHeight, 128, 128, 128, screenWidth, screenHeight, "Current Frame SDF");

  // Render Current Frame Deformed SDF mesh on a Square ViewPort on top right
  if (meshes[1] != nullptr)
  {
    glViewport(elementWidth * 2, elementHeight + TextHeight, minElementHeightAndWidth, minElementHeightAndWidth);
    glOrtho(-1, 1, -1, 1, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    renderMesh(meshes[1]);
  }

  glViewport(0, 0, screenWidth, screenHeight);
  displayText(elementWidth * 2.5, elementHeight + TextHeight, 128, 128, 128, screenWidth, screenHeight, "Current Deformed SDF");

  // Render Canonical SDF mesh on a Square ViewPort on bottom right
  if (meshes[2] != nullptr)
  {
    glViewport(elementWidth * 2, TextHeight, minElementHeightAndWidth, minElementHeightAndWidth);
    glOrtho(-1, 1, -1, 1, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    renderMesh(meshes[2]);
  }

  glViewport(0, 0, screenWidth, screenHeight);
  displayText(elementWidth * 2.5, 0, 128, 128, 128, screenWidth, screenHeight, "Merged Canonical SDF");

  glutSwapBuffers();
  glutPostRedisplay();

  /**
   * Save Screenshot
   */
  // Convert to FreeImage format & save to file
  glFinish(); // When doing heavy processing, the glReadPixels does not load all data.
  BYTE *pixels = new BYTE[3 * screenWidth * screenHeight];
  glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);
  glFinish();
  FIBITMAP *image = FreeImage_ConvertFromRawBits(pixels, screenWidth, screenHeight, 3 * screenWidth, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);

  std::stringstream filenameStream;
  filenameStream << outputDirPath << std::setfill('0') << std::setw(3) << std::to_string(currentFrameIndex) << ".png";
  FreeImage_Save(FIF_PNG, image, filenameStream.str().c_str(), 0);
  FreeImage_Unload(image);
  delete[] pixels;

  if(!lastFrameSeen && currentFrameIndex > fusion->getEndFrameIndex()) 
  {
    lastFrameSeen = true;
    meshes[2]->WriteMesh(outputDirPath+"FinalCanonicalMesh.obj");
  }

  /**
   * Release all KillingFusion resources
   */
  for (auto &&meshPtr : meshes)
    if (meshPtr != nullptr)
      delete meshPtr;
}

int main(int argc, char **argv)
{
  datasetReader = new DatasetReader(DATA_DIR);
  fusion = new KillingFusion(*datasetReader);

  // Create output directory to save screenshot
  std::stringstream outputDirectoryStream;
  outputDirectoryStream << OUTPUT_DIR << outputDir[datasetType]
                 << "Data-" << EnergyTypeUsed[0]
                 << "_LS-" << EnergyTypeUsed[1] << "_omegaLS-" << omegaLevelSet
                 << "_KVF-" << EnergyTypeUsed[2] << "_omegaK-" << omegaKilling << "_gammaK-" << gammaKilling
                 << "_Fuse-" << (FUSE_BY_MERGE ? "Merge" : "Math")
                 << "_VoxelSize-" << VoxelSize
                 << "_UnknownClipDist-" << UnknownClipDistance
                 << "_MaxSurfaceVoxelDist-" << MaxSurfaceVoxelDistance
                 << "_Iter-" << KILLING_MAX_ITERATIONS
                 << "_alpha-" << alpha 
                 << '/';
  outputDirPath = outputDirectoryStream.str();
  std::cout << outputDirPath << std::endl;
  cv::utils::fs::createDirectory(outputDirPath);

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutCreateWindow("KillingFusion");

  // register glut call backs
  glutReshapeFunc(reshape); // Using FullScreen
  glFinish(); // First go full screen
  glutDisplayFunc(draw);
  initGL(1920, 1080);
  glutFullScreen();

  // Run Main Loop
  glutMainLoop();
  delete datasetReader;
  delete fusion;
  return 0;
}
