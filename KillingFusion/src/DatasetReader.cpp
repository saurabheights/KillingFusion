//
// Created by Saurabh Khanduja on 18.10.18.
//

#include "DatasetReader.h"
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <limits>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

DatasetReader::DatasetReader(const std::string DatasetRootDir, const DEFORMABLE_DATASET dataset)
{
  m_imageDir = DatasetRootDir + imageDir[dataset];
  m_numImageFiles = numImageFiles[dataset];
  std::vector<float> intrinsicParams = LoadMatrixFromFile(DatasetRootDir + intrinsicParamsFile, 3 * 3);
  m_depthIntrinsicMatrix << intrinsicParams[0], intrinsicParams[1], intrinsicParams[2],
      intrinsicParams[3], intrinsicParams[4], intrinsicParams[5],
      intrinsicParams[6], intrinsicParams[7], intrinsicParams[8];
  m_depthHeight = 480;
  m_depthWidth = 640;
  analyzeMinMaxDepthValues(dataset);
}

/**
 * This method reads the value from intrinsic file into a vector.
 * ToDo: Check C++14 standard which allows to return by reference instead of value.
 *
 * @param filename The file to be read
 * @param M The number of elements in matrix
 * @return A vector of all the values read as float.
 */
std::vector<float> DatasetReader::LoadMatrixFromFile(std::string filename, int M)
{
  std::vector<float> matrix;
  FILE *fp = fopen(filename.c_str(), "r");
  for (int i = 0; i < M; i++)
  {
    float tmp;
    int iret = fscanf(fp, "%f", &tmp);
    matrix.push_back(tmp);
  }
  fclose(fp);
  return matrix;
}

std::vector<cv::Mat> DatasetReader::getImages(int frameIndex)
{
  std::ostringstream frameSuffix;
  frameSuffix << std::setw(6) << std::setfill('0') << frameIndex;
  std::string suffix = frameSuffix.str();
  std::string colorFile = m_imageDir + "/color_" + suffix + ".png";
  std::string depthFile = m_imageDir + "/depth_" + suffix + ".png";
  std::string omaskFile = m_imageDir + "/omask_" + suffix + ".png";

  std::vector<cv::Mat> cdoImages;
  cdoImages.push_back(cv::imread(colorFile));
  cdoImages.push_back(readDepthImage(depthFile));
  cdoImages.push_back(cv::imread(omaskFile, cv::IMREAD_GRAYSCALE));
  return cdoImages;
}

int DatasetReader::getNumImageFiles() const
{
  return m_numImageFiles;
}

void DatasetReader::analyzeMinMaxDepthValues(const DEFORMABLE_DATASET dataset)
{
  if (dataset < 2) // Always true in current case.
  {
    // Using cached values
    m_minMaxDepth = std::pair<float, float>(datasetDepthMinMaxValues[dataset][0],
                                            datasetDepthMinMaxValues[dataset][1]);
    return;
  }

  std::vector<float> minDepths, maxDepths;
  minDepths.reserve(getNumImageFiles());
  maxDepths.reserve(getNumImageFiles());

  for (size_t frameIndex = 0; frameIndex < getNumImageFiles(); frameIndex++)
  {
    std::vector<cv::Mat> cdoImages = getImages(frameIndex);
    cv::Mat depthMat = cdoImages.at(1);
    cv::Mat omaskMat = cdoImages.at(2);
    int H = depthMat.rows;
    int W = depthMat.cols;
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    for (int r = 0; r < H; ++r)
    {
      for (int c = 0; c < W; ++c)
      {
        uchar isForegroundPixel = omaskMat.at<uchar>(r, c);
        if (isForegroundPixel != 0)
        {
          float depth = depthMat.at<float>(r, c);
          // Ignore 0 values which represent invalid data
          if (min > depth && depth > 0)
          {
            min = depth;
          }
          if (max < depth)
          {
            max = depth;
          }
        }
      }
    }
    minDepths.push_back(min);
    maxDepths.push_back(max);
    std::cout << "Depth Frame " << frameIndex << " has valid min*, max value as : " << min << ", " << max << '\n';
  }
  std::sort(minDepths.begin(), minDepths.end());
  std::sort(maxDepths.begin(), maxDepths.end());
  std::cout << "Minimum and Maximum depth in all frames= " << minDepths.at(0) << " " << maxDepths.at(maxDepths.size() - 1) << std::endl;
  m_minMaxDepth = std::pair<float, float>(minDepths.at(0), maxDepths.at(maxDepths.size() - 1));
}

cv::Mat DatasetReader::readDepthImage(std::string depthFilename)
{
  cv::Mat depthImage = cv::imread(depthFilename, CV_LOAD_IMAGE_UNCHANGED);
  if (depthImage.empty())
  {
    std::cout << "Error: depth image file not read!\n";
    cv::waitKey(0);
    exit(-1);
  }

  cv::Mat depthFloatImage(depthImage.size(), CV_32FC1);
  depthImage.convertTo(depthFloatImage, CV_32FC1, 1.0 / 1000); // ToDo:  Move depthShift to config.h
  double minVal, maxVal;
  cv::minMaxLoc(depthFloatImage, &minVal, &maxVal);
  std::cout << "Depth Frame " << depthFilename.substr(depthFilename.size() - 7,3) << " has min, max value as : " << minVal << ", " << maxVal << '\n';
  return depthFloatImage;
}

int DatasetReader::getDepthHeight()
{
  return m_depthHeight;
}

int DatasetReader::getDepthWidth()
{
  return m_depthWidth;
}

Eigen::Matrix3f DatasetReader::getDepthIntrinsicMatrix()
{
  return m_depthIntrinsicMatrix;
}