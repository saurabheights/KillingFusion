//
// Created by Saurabh Khanduja on 22.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H

#include "DatasetReader.h"
#include "SDF.h"
#include <Eigen/Eigen>

class KillingFusion
{
  DatasetReader m_datasetReader;

  SDF *m_canonicalSdf;

  /**
   * Computes SDF for frame frameIndex. 
   */
  SDF *computeSDF(int frameIndex);

  /**
   * ToDo: Fix this Single SDF for all frames issue.
   * Computes Bound of SDF for all the frames. 
   */
  std::pair<Eigen::Vector3f, Eigen::Vector3f> computeBounds(int w, int h, float minDepth, float maxDepth);
 public:
  KillingFusion() = delete;
  KillingFusion(DatasetReader datasetReader);
  ~KillingFusion();
  void process();
  float getVoxelSize() const
  {
    return DatasetReader::getVoxelSize();
  }
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
