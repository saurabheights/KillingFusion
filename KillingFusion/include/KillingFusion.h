//
// Created by Saurabh Khanduja on 22.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H

#include "DatasetReader.h"

class KillingFusion {
  DatasetReader m_datasetReader;
 public:
  KillingFusion() = delete;
  KillingFusion(DatasetReader datasetReader);
  void process();
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
