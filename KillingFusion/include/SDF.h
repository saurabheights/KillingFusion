//
// Created by Saurabh Khanduja on 26.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "DisplacementField.h"

class SDF
{
  Eigen::Vector3i m_gridSize;
  Eigen::Vector3f m_min3dLoc;
  Eigen::Vector3f m_max3dLoc;
  // ToDo - Change to vector of vector of vector.
  // Makes notation much simpler as well as reduces the computation of indices.
  std::vector<float> m_voxelGridTSDF;
  std::vector<long> m_voxelGridWeight;
  float m_voxelSize;
  Eigen::Vector3i m_gridSpacingPerAxis;
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
   * Get value at spatial index. 
   */
  float getDistanceAtIndex(const Eigen::Vector3i &gridSpatialIndex) const;

  /**
   * Get value at grid location. 
   */
  float getDistance(const Eigen::Vector3f &gridLocation) const;

  /**
   * This function computes distance gradient at any index location.
   * 
   */
  Eigen::Vector3f computeDistanceGradient(const Eigen::Vector3f &worldLocation) const;

  /**
   * Fuses otherSdf which should be of same size as this.
   **/
  void fuse(const SDF *otherSdf);

  /**
   * Fuses otherSdf using its DisplacementField
   **/
  void fuse(const SDF *otherSdf, const DisplacementField *otherDisplacementField);

  /**
   * Writes the SDF to a file
   * @param outputFilePath
   */
  void dumpToBinFile(std::string outputFilePath,
                     float truncationDistanceInVoxelSizeUnit,
                     float minimumWeightThreshold);

  /**
   * Getters
   */
  Eigen::Vector3f getMin3dLoc() const
  {
    return m_min3dLoc;
  };

  Eigen::Vector3f getMax3dLoc() const
  {
    return m_max3dLoc;
  };

  Eigen::Vector3i getGridSize() const
  {
    return m_gridSize;
  };
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H
