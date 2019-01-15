//
// Created by Saurabh Khanduja on 22.10.18.
//

#include "KillingFusion.h"
#include "SDF.h"
#include "Timer.h"
using namespace std;

KillingFusion::KillingFusion(DatasetReader datasetReader)
    : m_datasetReader(datasetReader),
      m_canonicalSdf(nullptr)
{
  // Create a canonical SDF
  int w = m_datasetReader.getDepthWidth();
  int h = m_datasetReader.getDepthHeight();
  float minDepth = m_datasetReader.getMinimumDepthThreshold();
  float maxDepth = m_datasetReader.getMaximumDepthThreshold();
  std::pair<Eigen::Vector3f, Eigen::Vector3f> frameBound = computeBounds(w, h, minDepth, maxDepth);
  m_canonicalSdf = new SDF(DatasetReader::getVoxelSize(),
                           frameBound.first,
                           frameBound.second,
                           DatasetReader::getTruncationDistanceInVoxelSize());
}

KillingFusion::~KillingFusion()
{
  // ToDo - Use Unique Ptr
  if (m_canonicalSdf != nullptr)
    delete m_canonicalSdf;
}

void KillingFusion::process()
{
  DisplacementField *prev2CanDisplacementField;
  DisplacementField *curr2CanDisplacementField;

  // Set prevSdf to SDF of first frame
  const SDF *prevSdf = computeSDF(0);
  prev2CanDisplacementField = createZeroDisplacementField(*prevSdf);
  m_canonicalSdf->fuse(prevSdf);

  // For each file in DatasetReader
  Timer totalTimer, timer;
  cout << "Iter   Compute SDF    KillingOptimize    Fuse SDF\n";
  for (int i = 1; i < m_datasetReader.getNumImageFiles(); ++i)
  {
    // Convert current frame to SDF - currSdf
    timer.reset();
    SDF *currSdf = computeSDF(i);
    double sdfTime = timer.elapsed();

    // Future Task - Implement SDF-2-SDF to register currSDF to prevSDF
    // Future Task - ToDo - DisplacementField should have same shape as their SDF.
    curr2CanDisplacementField = prev2CanDisplacementField;

    // Compute Deformation Field for current frame SDF to merge with m_canonicalSdf
    timer.reset();
    computeDisplacementField(currSdf, m_canonicalSdf, curr2CanDisplacementField);
    double killingTime = timer.elapsed();

    // Merge the m_currSdf to m_canonicalSdf using m_currSdf displacement field.
    timer.reset();
    m_canonicalSdf->fuse(currSdf, curr2CanDisplacementField);
    double fuseTime = timer.elapsed();

    // Delete m_prevSdf and assign m_currSdf to m_prevSdf
    delete prevSdf;
    prevSdf = currSdf;
    prev2CanDisplacementField = curr2CanDisplacementField;
    printf("%03d\t%0.6fs\t%0.6fs\t%0.6fs\n", i, sdfTime, killingTime, fuseTime);
  }
  cout << "Total time spent " << totalTimer.elapsed() << endl;
  m_canonicalSdf->dumpToBinFile("output.bin",
                                DatasetReader::getTruncationDistanceInVoxelSize(), 1.0f);
}

SDF *KillingFusion::computeSDF(int frameIndex)
{ // ToDo: SDF class should compute itself
  // cout << "Computing SDF of frame " << frameIndex << endl;
  int w = m_datasetReader.getDepthWidth();
  int h = m_datasetReader.getDepthHeight();
  float minDepth = m_datasetReader.getMinimumDepthThreshold();
  float maxDepth = m_datasetReader.getMaximumDepthThreshold();
  std::pair<Eigen::Vector3f, Eigen::Vector3f> frameBound = computeBounds(w, h, minDepth, maxDepth);
  SDF *sdf = new SDF(DatasetReader::getVoxelSize(),
                     frameBound.first,
                     frameBound.second,
                     DatasetReader::getTruncationDistanceInVoxelSize());
  std::vector<cv::Mat> cdoImages = m_datasetReader.getImages(frameIndex);
  sdf->integrateDepthFrame(cdoImages.at(1),
                           cdoImages.at(2),
                           Eigen::Matrix4f::Identity(),
                           m_datasetReader.getDepthIntrinsicMatrix(),
                           minDepth,
                           maxDepth);
  return sdf;
}

DisplacementField *KillingFusion::createZeroDisplacementField(const SDF &sdf)
{
  return new DisplacementField(sdf.getGridSize(),
                               DatasetReader::getVoxelSize(),
                               sdf.getMin3dLoc(),
                               sdf.getMax3dLoc());
}

void KillingFusion::computeDisplacementField(const SDF *src,
                                             const SDF *dest,
                                             DisplacementField *srcToDest)
{
  // ToDo: Use Cuda.
  // Process at each voxel location
  Eigen::Vector3i srcGridSize = src->getGridSize();

  // Compute gradient of src voxel displacement to move it toward destination voxel

#pragma omp parallel for schedule(dynamic)
  for (int z = 0; z < srcGridSize(2); z++)
  {
    for (int y = 0; y < srcGridSize(1); y++)
    {
      for (int x = 0; x < srcGridSize(0); x++)
      {
        // Actual 3D Point on Desination Grid, where to optimize for.
        Eigen::Vector3i spatialIndex(x, y, z);
        Eigen::Vector3f p = (spatialIndex.array().cast<float>() + 0.5f) * getVoxelSize();

        // Optimize Killing Energy between Source Grid and Desination Grid
        Eigen::Vector3f gradient;
        do
        {
          gradient = computeEnergyGradient(src, dest, srcToDest, spatialIndex, p);
          srcToDest->update(spatialIndex, -alpha * gradient);

          if (gradient.norm() <= threshold)
            break;

          // perform check on deformation field to see if it has diverged. Ideally shouldn't happen
          if (!srcToDest->getDisplacementAt(spatialIndex).array().isFinite().all())
          {
            std::cout << "Error: deformation field has diverged: " << srcToDest->getDisplacementAt(spatialIndex) << " at: " << p << std::endl;
            throw - 1;
          }
        } while(gradient.norm() > threshold); // ToDo: Need to reduce threshold Currently runs too long.
      }
    }
  }
}

Eigen::Vector3f KillingFusion::computeEnergyGradient(const SDF *src,
                                                     const SDF *dest,
                                                     const DisplacementField *srcDisplacementField,
                                                     const Eigen::Vector3i &spatialIndex,
                                                     const Eigen::Vector3f &p)
{
  return computeDataEnergyGradient(src, dest, srcDisplacementField, spatialIndex, p) +
         computeKillingEnergyGradient(src, srcDisplacementField, spatialIndex, p) * omegaKilling +
         computeLevelSetEnergyGradient(src, srcDisplacementField, spatialIndex, p) * omegaLevelSet;
}

Eigen::Vector3f KillingFusion::computeDataEnergyGradient(const SDF *src,
                                                         const SDF *dest,
                                                         const DisplacementField *srcDisplacementField,
                                                         const Eigen::Vector3i &spatialIndex,
                                                         const Eigen::Vector3f &p)
{
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KillingFusion::computeKillingEnergyGradient(const SDF *src,
                                                            const DisplacementField *srcDisplacementField,
                                                            const Eigen::Vector3i &spatialIndex,
                                                            const Eigen::Vector3f &p)
{
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KillingFusion::computeLevelSetEnergyGradient(const SDF *src,
                                                             const DisplacementField *srcDisplacementField,
                                                             const Eigen::Vector3i &spatialIndex,
                                                             const Eigen::Vector3f &p)
{
  return Eigen::Vector3f::Zero();
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> KillingFusion::computeBounds(int w, int h, float minDepth, float maxDepth)
{
  // Create frustum for the camera
  Eigen::MatrixXf cornerPoints;
  cornerPoints.resize(3, 8);
  cornerPoints << 0, 0, w - 1, w - 1, 0, 0, w - 1, w - 1,
      0, h - 1, h - 1, 0, 0, h - 1, h - 1, 0,
      1, 1, 1, 1, 1, 1, 1, 1;

  Eigen::Matrix<float, 1, 8> cornersDepth;
  cornersDepth << minDepth, minDepth, minDepth, minDepth,
      maxDepth, maxDepth, maxDepth, maxDepth;

  // Compute depthIntrinsicMatrix
  Eigen::Matrix3f depthIntrinsicMatrix = m_datasetReader.getDepthIntrinsicMatrix();
  Eigen::Matrix3f depthIntrinsicMatrixInv = depthIntrinsicMatrix.inverse();

  // Compute the corner location in the Camera Coordinate System(CCS)
  Eigen::MatrixXf imagePoints = depthIntrinsicMatrixInv * cornerPoints;

  // Compute the frustum in the Camera Coordinate System(CCS)
  imagePoints.conservativeResize(4, 8); // Resize from 3x8 to 4x8
  imagePoints.topLeftCorner(2, 4) =
      imagePoints.topLeftCorner(2, 4) * minDepth; // Multiply front four corners with minDepth
  imagePoints.topRightCorner(2, 4) =
      imagePoints.topRightCorner(2, 4) * maxDepth; // Multiply back four corners with maxDepth
  imagePoints.row(3) = imagePoints.row(2);         // Move ones below
  imagePoints.row(2) = cornersDepth.transpose();   // Replace 3rd row with min/max depth values

  // Compute world location of this frustum
  // Since world is same as camera, is Eigen::Matrix4f::Identity()
  Eigen::Matrix4f camera_to_world_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix<float, 4, 8> frustumWorldPoints = camera_to_world_pose * imagePoints;
  // Compute the bounds of parallelogram containing the frustum
  Eigen::Vector4f minXYZ = frustumWorldPoints.rowwise().minCoeff().array();
  Eigen::Vector4f maxXYZ = frustumWorldPoints.rowwise().maxCoeff().array();

  Eigen::Vector3f min3dLoc(minXYZ(0), minXYZ(1), minXYZ(2));
  Eigen::Vector3f max3dLoc(maxXYZ(0), maxXYZ(1), maxXYZ(2));

  return pair<Eigen::Vector3f, Eigen::Vector3f>(min3dLoc, max3dLoc);
}
