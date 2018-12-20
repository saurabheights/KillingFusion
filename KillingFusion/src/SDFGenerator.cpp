//
// Created by Saurabh Khanduja on 25.11.18.
//
#include <iostream>
#include <memory>
#include "SDFGenerator.h"
using namespace std;

SDFGenerator::SDFGenerator(std::string sensFolderPath) {
    this->rootDir = sensFolderPath;
    if (rootDir.at(rootDir.length() - 1) == '/') {
        rootDir = rootDir.substr(0, rootDir.length() - 1);
    }

    // Load the sens data summary
    scannetDataLoader.reset(new ScannetDataLoader(rootDir));
    alignAxis = false;
}

std::shared_ptr<SDF> SDFGenerator::createSDF() {
    // Compute the bound of SDF using minDepth, maxDepth and poses of each image.
    std::pair<Eigen::Vector3f, Eigen::Vector3f> bound = computeBounds();
#ifdef DEBUG
    cout << "Lower bound of world coordinates: \n" << bound.first << "\n";
    cout << "Upper bound of world coordinates: \n" << bound.second << "\n";
#endif
    // Initialize the SDF
    std::shared_ptr<SDF> sdf = std::make_shared<SDF>(scannetDataLoader->getVoxelSize(),
                                                     bound.first,
                                                     bound.second,
                                                     ScannetDataLoader::getTruncationDistanceInVoxelSize());

    // Merge each image into the SDF
    for (int i = 0; i < scannetDataLoader->getNumDepthFrames(); ++i) {
        std::cout << "============================================================================\n";
        std::cout << "Integrating " << i + 1 << " of " << scannetDataLoader->getNumDepthFrames() << " frames.\n";
        Eigen::Matrix3f depthIntrinsicMatrix = scannetDataLoader->getDepthIntrinsicMatrix();
        Eigen::Matrix4f depthFramePose = scannetDataLoader->getCameraToWorldPose(i, alignAxis);
        cv::Mat depthFrame = scannetDataLoader->getDepthFrame(i);
        sdf->integrateDepthFrame(depthFrame,
                                 depthFramePose,
                                 depthIntrinsicMatrix,
                                 scannetDataLoader->getMinimumDepthThreshold(),
                                 scannetDataLoader->getMaximumDepthThreshold());
    }
    cout << "============================================================================\n";
    return sdf;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> SDFGenerator::computeBounds() {
    cout << "============================================================================\n";
    cout << "Computing Full Frustum of all depth frame...\n";
    float minVal = std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::min();
    Eigen::Vector4f minXYZ(minVal, minVal, minVal, 1);
    Eigen::Vector4f maxXYZ(maxVal, maxVal, maxVal, 1);

    // Should we use depth image or color image
    int w = scannetDataLoader->getDepthWidth();
    int h = scannetDataLoader->getDepthHeight();

    // Read from ScannetDataLoader. Probably needs to be hardcoded
    float minDepth = scannetDataLoader->getMinimumDepthThreshold();
    float maxDepth = scannetDataLoader->getMaximumDepthThreshold();
    Eigen::Matrix<float, 1, 8> cornersDepth;
    cornersDepth << minDepth, minDepth, minDepth, minDepth,
        maxDepth, maxDepth, maxDepth, maxDepth;

    // Create frustum for the camera
    Eigen::MatrixXf cornerPoints;
    cornerPoints.resize(3, 8);
    cornerPoints << 0, 0, w - 1, w - 1, 0, 0, w - 1, w - 1,
        0, h - 1, h - 1, 0, 0, h - 1, h - 1, 0,
        1, 1, 1, 1, 1, 1, 1, 1;
#ifdef DEBUG
    cout << "The image corners in pixel values are \n" << cornerPoints << "\n";
#endif
    Eigen::Matrix3f depthIntrinsicInverse = scannetDataLoader->getDepthIntrinsicMatrix().inverse();

    for (int i = 0; i < scannetDataLoader->getNumDepthFrames(); ++i) {
        // Compute the corner location in the Camera Coordinate System(CCS)
        Eigen::MatrixXf imagePoints = depthIntrinsicInverse * cornerPoints;
#ifdef DEBUG
        cout << "The image corners in Camera Coordinate System are \n" << imagePoints << "\n";
#endif
        // Compute the frustum in the Camera Coordinate System(CCS)
        imagePoints.conservativeResize(4, 8);          // Resize from 3x8 to 4x8
        imagePoints.topLeftCorner(2, 4) =
            imagePoints.topLeftCorner(2, 4) * minDepth; // Multiply front four corners with minDepth
        imagePoints.topRightCorner(2, 4) =
            imagePoints.topRightCorner(2, 4) * maxDepth; // Multiply back four corners with maxDepth
        imagePoints.row(3) = imagePoints.row(2);       // Move ones below
        imagePoints.row(2) = cornersDepth.transpose(); // Replace 3rd row with min/max depth values
#ifdef DEBUG
        cout << "The frustum corners in Camera Coordinate System are \n" << imagePoints << "\n";
#endif
        // Compute world location of this frustum using pose of caemra for ith image.
        Eigen::Matrix4f camera_to_world_pose = scannetDataLoader->getCameraToWorldPose(i, alignAxis);
        Eigen::Matrix<float, 4, 8> frustumWorldPoints = camera_to_world_pose * imagePoints;
#ifdef DEBUG
        std::cout << "Corner Points in WCS are \n" << frustumWorldPoints << "\n";
#endif
        // Update the bounds of sdf to contain this frustum
#ifdef DEBUG
        cout << "Minimum and Maximum Frustum X, Y, Z location is \n"
             << frustumWorldPoints.rowwise().minCoeff().array().transpose() << "\n"
             << frustumWorldPoints.rowwise().maxCoeff().array().transpose() << "\n";
        cout << "Old SDF Bound is \n" << minXYZ.transpose() << "\n" << maxXYZ.transpose() << "\n";
#endif
        minXYZ = minXYZ.array().cwiseMin(frustumWorldPoints.rowwise().minCoeff().array());
        maxXYZ = maxXYZ.array().cwiseMax(frustumWorldPoints.rowwise().maxCoeff().array());
#ifdef DEBUG
        cout << "New SDF Bound is \n" << minXYZ.transpose() << "\n" << maxXYZ.transpose() << "\n";
#endif
    }

    Eigen::Vector3f min3dLoc(minXYZ(0), minXYZ(1), minXYZ(2));
    Eigen::Vector3f max3dLoc(maxXYZ(0), maxXYZ(1), maxXYZ(2));
    return std::pair<Eigen::Vector3f, Eigen::Vector3f>(min3dLoc, max3dLoc);
}
