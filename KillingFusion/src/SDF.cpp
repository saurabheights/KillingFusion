//
// Created by Saurabh Khanduja on 26.10.18.
//

#include "SDF.h"
using namespace std;

SDF::SDF(float _voxelSize,
         Eigen::Vector3f _min3dLoc,
         Eigen::Vector3f _max3dLoc,
         float truncationDistanceInVoxelSize) : m_voxelSize{_voxelSize}, m_bound{_max3dLoc - _min3dLoc} {
    m_min3dLoc = _min3dLoc;
    m_max3dLoc = _max3dLoc;

    m_truncationDistanceInVoxelSize = truncationDistanceInVoxelSize;

    m_voxelGridTSDF = nullptr;
    m_voxelGridWeight = nullptr;

    computeVoxelGridSize();
    allocateMemoryForSDF();
}

SDF::~SDF() {
    if (m_voxelGridTSDF != nullptr) {
        delete[](m_voxelGridTSDF);
        delete[](m_voxelGridWeight);
        m_voxelGridTSDF = nullptr;
        m_voxelGridWeight = nullptr;
    }
}

void SDF::computeVoxelGridSize() {
    cout << "Bound is: " << m_bound.transpose() << "\n";
    cout << "Voxel Size is: " << m_voxelSize << "\n";
    Eigen::Vector3i ones(1, 1, 1);
    // Casting may reduce size by 1.
    m_gridSize = ((m_bound / m_voxelSize)).cast<int>() + ones;
    cout << "Grid Size computed is: " << m_gridSize.transpose() << "\n";
}

void SDF::allocateMemoryForSDF() {
    // Initialize voxel grid
    if (m_voxelGridTSDF != nullptr) {
        delete[](m_voxelGridTSDF);
        delete[](m_voxelGridWeight);
        m_voxelGridTSDF = nullptr;
        m_voxelGridWeight = nullptr;
    }
    int num_grid_elements = m_gridSize.prod();
    m_voxelGridTSDF = new float[num_grid_elements];
    m_voxelGridWeight = new float[num_grid_elements];
    // Set the distance to 1, since 1 represents each point is too far from the surface.
    // This is done for any voxel which had no correspondences in the image and thus was never processed.
    for (int i = 0; i < num_grid_elements; ++i)
        m_voxelGridTSDF[i] = 1.0f;
    memset(m_voxelGridWeight, 0, sizeof(float) * num_grid_elements);
}

void SDF::integrateDepthFrame(cv::Mat depthFrame,
                              cv::Mat maskFrame,
                              Eigen::Matrix4f depthFrameC2WPose,
                              Eigen::Matrix3f depthIntrinsicMatrix,
                              float minDepth,
                              float maxDepth)
{
    int w = depthFrame.cols;
    int h = depthFrame.rows;

    Eigen::Matrix3f depthIntrinsicMatrixInv = depthIntrinsicMatrix.inverse();
    Eigen::Matrix4f world_to_camera_pose = depthFrameC2WPose.inverse();

#ifdef DEBUG
    cout << "Lower bound of Voxels to loop over: \n" << minLocIndex.transpose() << "\n";
    cout << "Upper bound of Voxels to loop over: \n" << maxLocIndex.transpose() << "\n";
#endif

    float truncationDistance = m_truncationDistanceInVoxelSize * m_voxelSize;

    // ToDo - Add truncated number of voxels to SDF
    for (int z = 0; z < m_gridSize(2); z++) {
        float Z = m_voxelSize * (z + 0.5f) + m_min3dLoc(2);
        for (int y = 0; y < m_gridSize(1); y++) {
            float Y = m_voxelSize * (y + 0.5f) + m_min3dLoc(1);
            for (int x = 0; x < m_gridSize(0); x++) {
                float X = m_voxelSize * (x + 0.5f) + m_min3dLoc(0);
#ifdef DEBUG
                    cout << "Voxel Center location in grid voxelsize is : " << x << ", " << y << ", " << z << "\n";
                    cout << "Voxel Center location in world is : " << X << ", " << Y << ", " << Z << "\n";
#endif
                // Compute 3d location of center of voxel
                // Backproject it to the depth image
                Eigen::Vector4f voxelCenter(X, Y, Z, 1);
                Eigen::Vector4f voxelCenterInCamera = world_to_camera_pose * voxelCenter;
                Eigen::Vector3f
                    voxelPixelLocation(voxelCenterInCamera(0)/voxelCenterInCamera(2),
                        voxelCenterInCamera(1)/voxelCenterInCamera(2), 1);
                voxelPixelLocation = depthIntrinsicMatrix * voxelPixelLocation;
                
#ifdef DEBUG
                    cout << "Voxel Center's World homogeneous location is : " << voxelCenter.transpose() << "\n";
                    cout << "Voxel Center's Image homogeneous location is : " << voxelCenterInCamera.transpose() << "\n";
                    cout << "Voxel Center's Image pixel location is : " << voxelPixelLocation.transpose() << "\n";
#endif

                // If pixel is outside the image, check next voxel
                int col = roundf(voxelPixelLocation(0) + 0.5f);
#ifdef DEBUG
                cout << "Col : " << col << "\n";
#endif
                if (col < 0 || col >= depthFrame.cols)
                    continue;

                int row = roundf(voxelPixelLocation(1) + 0.5f);
#ifdef DEBUG
                cout << "Row : " << row << "\n";
#endif
                if (row < 0 || row >= depthFrame.rows)
                    continue;

                uchar isForegroundPixel = maskFrame.at<uchar>(row, col);
                if ( isForegroundPixel == 0)
                    continue;

                // If depth image is valid at the pixel
                float voxelCenterDepth = voxelCenterInCamera(2);
                float depth = depthFrame.at<float>(row, col);
                if (depth < minDepth || depth > maxDepth) // ToDo: Is clipping between 0.4 to 4 ok
                    continue;

                float diff = depth - voxelCenterDepth;

                // Check if diff is too negative: truncate_margin
                if (diff < -truncationDistance) // Ignore any voxel at certain threshold behind the surface.
                    continue;

                // Compute normalized signed distance - from -1 to 1.
                float dist = fmin(1.0f, diff / truncationDistance);

                // Merge the signed distance in this voxel
                int index = z * (m_gridSize(1) * m_gridSize(0)) + y * m_gridSize(0) + x;
                m_voxelGridTSDF[index] =
                    (m_voxelGridTSDF[index] * m_voxelGridWeight[index] + dist) / (m_voxelGridWeight[index] + 1);
                m_voxelGridWeight[index] += 1;
            }
        }
    }
}

void SDF::dumpToBinFile(string outputFilePath,
                        float truncationDistanceInVoxelSizeUnit,
                        float minimumWeightThreshold) {
    // Save TSDF voxel grid and its parameters to disk as binary file (float array)
    cout << "============================================================================\n";
    cout << "Saving TSDF voxel grid values at " << outputFilePath << "\n";
    ofstream outFile(outputFilePath, ios::binary | ios::out);
    outFile.write((char *) m_gridSize.data(), 3 * sizeof(int));
    outFile.write((char *) m_min3dLoc.data(), 3 * sizeof(float));
    outFile.write((char *) &m_voxelSize, sizeof(float));
    outFile.write((char *) &truncationDistanceInVoxelSizeUnit, sizeof(float));
    outFile.write((char *) m_voxelGridTSDF, m_gridSize(0) * m_gridSize(1) * m_gridSize(2) * sizeof(float));
    outFile.close();

    float min = m_voxelGridTSDF[0];
    float max = m_voxelGridTSDF[0];
    for (size_t i = 0; i < m_gridSize(0) * m_gridSize(1) * m_gridSize(2); i++) {
        if (m_voxelGridTSDF[i] < min)
            min = m_voxelGridTSDF[i];
        else if (m_voxelGridTSDF[i] > max)
            max = m_voxelGridTSDF[i];
    }

    cout << "Minimum SDF value is " << min << " and max value is " << max << "\n";
    cout << "TSDF voxel grid values saved at " << outputFilePath << "\n";
    cout << "============================================================================\n";
}