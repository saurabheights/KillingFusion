//
// Created by Saurabh Khanduja on 18.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_DATASETREADER_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_DATASETREADER_H

#include <string>
#include <vector>
#include <config.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

class DatasetReader
{
private:
  std::string m_imageDir;
  Eigen::Matrix3f m_depthIntrinsicMatrix;
  int m_numImageFiles;
  int m_depthHeight, m_depthWidth;
  std::pair<float, float> m_minMaxDepth;

  std::vector<float> LoadMatrixFromFile(std::string filename, int M);
  cv::Mat readDepthImage(std::string depthFilename);
  void analyzeMinMaxDepthValues(const DEFORMABLE_DATASET dataset);

public:
  DatasetReader() = delete;

  DatasetReader(const std::string DatasetRootDir);

  std::vector<cv::Mat> getImages(int frameIndex);

  int getNumImageFiles() const;
  int getDepthHeight();
  int getDepthWidth();
  Eigen::Matrix3f getDepthIntrinsicMatrix();

  float getMinimumDepthThreshold() { return m_minMaxDepth.first; }
  float getMaximumDepthThreshold() { return m_minMaxDepth.second; }
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_DATASETREADER_H
