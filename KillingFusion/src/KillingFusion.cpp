//
// Created by Saurabh Khanduja on 22.10.18.
//

#include "KillingFusion.h"
#include "SDF.h"
using namespace std;

KillingFusion::KillingFusion(DatasetReader datasetReader) : m_datasetReader(datasetReader),
                                                            m_canonicalSdf(nullptr),
                                                            m_currSdf(nullptr),
                                                            m_prevSdf(nullptr) {}

KillingFusion::~KillingFusion()
{
  if (m_canonicalSdf != nullptr)
    delete m_canonicalSdf;

  if (m_prevSdf != nullptr) 
    delete m_prevSdf;

  if (m_currSdf != nullptr)
    delete m_currSdf;
}

void KillingFusion::process()
{
  // Create a canonical SDF
  m_canonicalSdf = computeSDF(0);

  // Save prevSdf to firest frame SDF
  m_prevSdf = computeSDF(0);

  // Save prev2can (previous to canonical displacement field) to null

  // For each file in DatasetReader
  for (int i = 1; i < m_datasetReader.getNumImageFiles(); ++i)
  {
    // Convert to SDF - currSdf
    m_currSdf = computeSDF(i);

    // Register currSDF to prevSDF using SDF-2-SDF framework and compure cur2prev displacement field

    // Compute cur2can displacement field by adding cur2prev to prev2can sdfDisplacementField

    // Apply VariationalFramework to minimize the distance between currSdf

    // Merge the m_currSdf to m_canonicalSdf using displacement field. 
  }
  m_canonicalSdf->dumpToBinFile("output.bin", DatasetReader::getTruncationDistanceInVoxelSize(), 1.0f);
}

SDF *KillingFusion::computeSDF(int frameIndex)
{ // ToDo: SDF class should compute itself
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
