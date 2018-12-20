//
// Created by Saurabh Khanduja on 26.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <iostream>

class SDF
{
  Eigen::Vector3i m_gridSize;
  Eigen::Vector3f m_min3dLoc;
  Eigen::Vector3f m_max3dLoc;
  float *m_voxelGridTSDF;
  float *m_voxelGridWeight;
  float m_voxelSize;
  Eigen::Vector3f m_bound;
  float m_truncationDistanceInVoxelSize;
  void computeVoxelGridSize();
  void allocateMemoryForSDF();

public:
  SDF(float _voxelSize,
      Eigen::Vector3f _min3dLoc,
      Eigen::Vector3f _max3dLoc,
      float truncationDistanceInVoxelSize);

  ~SDF();

  /**
     * Main function to merge depth frames into the SDF volume.
     * The method can be called multiple times but need to be initialized with volume bounds beforehand.
     * @param depthFrame
     * @param maskFrame
     * @param depthFrameC2WPose
     * @param depthIntrinsicMatrix
     * @param minDepth
     * @param maxDepth
     */
  void integrateDepthFrame(cv::Mat depthFrame,
                           cv::Mat maskFrame,
                           Eigen::Matrix4f depthFrameC2WPose,
                           Eigen::Matrix3f depthIntrinsicMatrix,
                           float minDepth,
                           float maxDepth);

  /**
     * Writes the SDF to a file
     * @param outputFilePath
     */
  void dumpToBinFile(std::string outputFilePath,
                     float truncationDistanceInVoxelSizeUnit,
                     float minimumWeightThreshold);
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H
