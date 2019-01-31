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
#include "config.h"

DatasetReader::DatasetReader(const std::string DatasetRootDir)
{
  m_imageDir = DatasetRootDir + imageDir[datasetType];
  m_numImageFiles = numImageFiles[datasetType];
  std::vector<double> intrinsicParams = LoadMatrixFromFile(DatasetRootDir + intrinsicParamsFile, 3 * 3);
  m_depthIntrinsicMatrix << intrinsicParams[0], intrinsicParams[1], intrinsicParams[2],
      intrinsicParams[3], intrinsicParams[4], intrinsicParams[5],
      intrinsicParams[6], intrinsicParams[7], intrinsicParams[8];
  m_depthHeight = 480;
  m_depthWidth = 640;
  analyzeMinMaxDepthValues(datasetType);
}

/**
 * This method reads the value from intrinsic file into a vector.
 * ToDo: Check C++14 standard which allows to return by reference instead of value.
 *
 * @param filename The file to be read
 * @param M The number of elements in matrix
 * @return A vector of all the values read as double.
 */
std::vector<double> DatasetReader::LoadMatrixFromFile(std::string filename, int M)
{
  std::vector<double> matrix;
  FILE *fp = fopen(filename.c_str(), "r");
  for (int i = 0; i < M; i++)
  {
    double tmp;
    int iret = fscanf(fp, "%lf", &tmp);
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

  std::vector<cv::Mat> cdImages;
  cdImages.push_back(cv::imread(colorFile));
  cv::Mat depthImage = readDepthImage(depthFile);
  cv::Mat mask = cv::imread(omaskFile, cv::IMREAD_GRAYSCALE);
  cv::Mat maskedDepthImage;
  depthImage.copyTo(maskedDepthImage, mask);
  cdImages.push_back(maskedDepthImage);
  return cdImages;
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
    m_minMaxDepth = std::pair<double, double>(datasetDepthMinMaxValues[dataset][0],
                                            datasetDepthMinMaxValues[dataset][1]);
    return;
  }

  std::vector<double> minDepths, maxDepths;
  minDepths.reserve(getNumImageFiles());
  maxDepths.reserve(getNumImageFiles());

  for (size_t frameIndex = 0; frameIndex < getNumImageFiles(); frameIndex++)
  {
    std::vector<cv::Mat> cdoImages = getImages(frameIndex);
    cv::Mat depthMat = cdoImages.at(1);
    int H = depthMat.rows;
    int W = depthMat.cols;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    for (int r = 0; r < H; ++r)
    {
      for (int c = 0; c < W; ++c)
      {
        double depth = depthMat.at<double>(r, c);
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
    minDepths.push_back(min);
    maxDepths.push_back(max);
    std::cout << "Depth Frame " << frameIndex << " has valid min*, max value as : " << min << ", " << max << '\n';
  }
  std::sort(minDepths.begin(), minDepths.end());
  std::sort(maxDepths.begin(), maxDepths.end());
  std::cout << "Minimum and Maximum depth in all frames= " << minDepths.at(0) << " " << maxDepths.at(maxDepths.size() - 1) << std::endl;
  m_minMaxDepth = std::pair<double, double>(minDepths.at(0), maxDepths.at(maxDepths.size() - 1));
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

  cv::Mat depthFloatImage(depthImage.size(), CV_64FC1);
  depthImage.convertTo(depthFloatImage, CV_64FC1, 1.0 / 1000); // Future tasks Move depthShift to config.h
  return depthFloatImage;
  // double minVal, maxVal;
  // cv::minMaxLoc(depthFloatImage, &minVal, &maxVal);
  // std::cout << "Depth Frame " << depthFilename.substr(depthFilename.size() - 7, 3) << " has minimum and maximum value as : " << minVal << ", " << maxVal << std::endl;
  // cv::Mat depthFloatFilteredImage;
  // cv::bilateralFilter(depthFloatImage, depthFloatFilteredImage, 5, 50, 50, cv::BORDER_DEFAULT);
  // cv::Mat dst;
  // cv::hconcat(depthFloatImage, depthFloatFilteredImage, dst);
  // dst = (dst - minVal) / (maxVal - minVal);
  // cv::imshow("Depth Image And Depth Image Filtered", dst);
  // cv::waitKey(100);
  // return depthFloatFilteredImage;
}

int DatasetReader::getDepthHeight()
{
  return m_depthHeight;
}

int DatasetReader::getDepthWidth()
{
  return m_depthWidth;
}

Eigen::Matrix3d DatasetReader::getDepthIntrinsicMatrix()
{
  return m_depthIntrinsicMatrix;
}