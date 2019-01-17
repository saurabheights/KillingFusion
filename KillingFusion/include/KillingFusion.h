//
// Created by Saurabh Khanduja on 22.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H

#include "DatasetReader.h"
#include "DisplacementField.h"
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
  DisplacementField *createZeroDisplacementField(const SDF &sdf);

  /**
   * Main Killing Methods
   */
  void computeDisplacementField(const SDF *src,
                                const SDF *dest,
                                DisplacementField *srcToDest);
  Eigen::Vector3f computeEnergyGradient(const SDF *src,
                                        const SDF *dest,
                                        const DisplacementField *srcDisplacementField,
                                        const Eigen::Vector3i &spatialIndex,
                                        const Eigen::Vector3f &p);
  Eigen::Vector3f computeDataEnergyGradient(const SDF *src,
                                            const SDF *dest,
                                            const DisplacementField *srcDisplacementField,
                                            const Eigen::Vector3i &spatialIndex,
                                            const Eigen::Vector3f &p);
  Eigen::Vector3f computeKillingEnergyGradient(const SDF *src,
                                               const DisplacementField *srcDisplacementField,
                                               const Eigen::Vector3i &spatialIndex,
                                               const Eigen::Vector3f &p);
  Eigen::Vector3f computeLevelSetEnergyGradient(const SDF *src,
                                                const DisplacementField *srcDisplacementField,
                                                const Eigen::Vector3i &spatialIndex,
                                                const Eigen::Vector3f &p);

public:
  KillingFusion() = delete;
  KillingFusion(DatasetReader datasetReader);
  ~KillingFusion();
  void process();
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
