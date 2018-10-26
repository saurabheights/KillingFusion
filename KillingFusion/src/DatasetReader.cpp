//
// Created by Saurabh Khanduja on 18.10.18.
//

#include "DatasetReader.h"
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

DatasetReader::DatasetReader(std::string imageDir,
                             std::string depthFilePattern,
                             std::string colorFilePattern,
                             std::string omaskFilePattern,
                             std::string intrinsicParamsFile,
                             int numImageFiles) : m_imageDir(imageDir),
                                                  m_depthFilePattern(depthFilePattern),
                                                  m_colorFilePattern(colorFilePattern),
                                                  m_omaskFilePattern(omaskFilePattern),
                                                  m_numImageFiles(numImageFiles) {
  m_intrinsicParams = LoadMatrixFromFile(intrinsicParamsFile, 3 * 3);
}

/**
 * This method reads the value from intrinsic file into a vector.
 * ToDo: Check C++14 standard which allows to return by reference instead of value.
 *
 * @param filename The file to be read
 * @param M The number of elements in matrix
 * @return A vector of all the values read as float.
 */
std::vector<float> DatasetReader::LoadMatrixFromFile(std::string filename, int M) {
  std::vector<float> matrix;
  FILE *fp = fopen(filename.c_str(), "r");
  for (int i = 0; i < M; i++) {
    float tmp;
    int iret = fscanf(fp, "%f", &tmp);
    matrix.push_back(tmp);
  }
  fclose(fp);
  return matrix;
}

void DatasetReader::loadImages() {
  std::ostringstream frameSuffix;
  for (int i = 0; i < m_numImageFiles; ++i) {
    frameSuffix << std::setw(6) << std::setfill('0') << i;
    std::string suffix = frameSuffix.str();
    std::string depthFile = m_imageDir + "/depth_" + suffix + ".png";
    std::string colorFile = m_imageDir + "/color_" + suffix + ".png";
    std::string omaskFile = m_imageDir + "/omask_" + suffix + ".png";
  }
}

int DatasetReader::getNumImageFiles() const {
  return m_numImageFiles;
}

void ReadDepth(std::string filename, int H, int W, float * depth) {
  cv::Mat depth_mat = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
  if (depth_mat.empty()) {
    std::cout << "Error: depth image file not read!" << std::endl;
    cv::waitKey(0);
  }
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c) {
      depth[r * W + c] = (float)(depth_mat.at<unsigned short>(r, c)) / 1000.0f;
      if (depth[r * W + c] > 6.0f) // Only consider depth < 6m
        depth[r * W + c] = 0;
    }
}
