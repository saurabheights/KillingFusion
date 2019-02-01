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
  // Variables for processing one frame at a time
  int m_startFrame;
  int m_endFrame;
  int m_currFrameIndex;
  int m_stride;
  DisplacementField *m_prev2CanDisplacementField;
  ////////////////

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
                                        const Eigen::Vector3i &spatialIndex);
  Eigen::Vector3f computeDataEnergyGradient(const SDF *src,
                                            const SDF *dest,
                                            const DisplacementField *srcDisplacementField,
                                            const Eigen::Vector3i &spatialIndex);
  Eigen::Vector3f computeKillingEnergyGradient(const DisplacementField *srcDisplacementField,
                                               const Eigen::Vector3i &spatialIndex);
  Eigen::Vector3f computeLevelSetEnergyGradient(const SDF *src,
                                                const SDF *dest,
                                                const DisplacementField *srcDisplacementField,
                                                const Eigen::Vector3i &spatialIndex);

public:
  KillingFusion() = delete;
  KillingFusion(DatasetReader datasetReader);
  ~KillingFusion();

  /**
   * Performs fusion of all frames.
   */
  void process();

  /**
   * Fuses one frame in the current canonincal model and returns 3 meshes. 
   * First is of currentFrame SDF.
   * Second is of deformed current frame SDF.
   * Third is of new canonincal SDF.
   */
  std::vector<SimpleMesh *> processNextFrame();

  /**
   * Test KillingFusion on two Sphere SDF
   */
  void processTest(int testType);

  /**
   * Get the frame index on which processNextFrame works on, in its next call.
   */
  int getCurrentFrameIndex()
  {
    return m_currFrameIndex;
  }

  int getEndFrameIndex()
  {
    return m_endFrame;
  }
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_FUSION_H
