//
// Created by Saurabh Khanduja on 22.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H

#include "DatasetReader.h"
#include "SDF.h"
#include <Eigen/Eigen>

class KillingFusion {
  DatasetReader m_datasetReader;
  SDF* m_canonicalSdf;
  SDF *m_prevSdf;
  SDF *m_currSdf;
  SDF* computeSDF(int frameIndex);
  std::pair<Eigen::Vector3f, Eigen::Vector3f> computeBounds(int w, int h, float minDepth, float maxDepth);
 public:
  KillingFusion() = delete;
  KillingFusion(DatasetReader datasetReader);
  ~KillingFusion();
  void process();
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
