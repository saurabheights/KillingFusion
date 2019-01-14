//
// Created by Saurabh Khanduja on 26.10.18.
//

#include "SDF.h"
using namespace std;

SDF::SDF(float _voxelSize,
         Eigen::Vector3f _min3dLoc,
         Eigen::Vector3f _max3dLoc,
         float truncationDistanceInVoxelSize)
    : m_voxelSize{_voxelSize},
      m_bound{_max3dLoc - _min3dLoc}
{
    m_min3dLoc = _min3dLoc;
    m_max3dLoc = _max3dLoc;

    m_truncationDistanceInVoxelSize = truncationDistanceInVoxelSize;

    computeVoxelGridSize();
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
    allocateMemoryForSDF();
}

SDF::~SDF()
{
}

void SDF::computeVoxelGridSize()
{
    cout << "Bound is: " << m_bound.transpose() << "\n";
    cout << "Voxel Size is: " << m_voxelSize << "\n";
    // Casting may reduce size by 1.
    m_gridSize = ((m_bound / m_voxelSize)).cast<int>().array() + 1;
    cout << "Grid Size computed is: " << m_gridSize.transpose() << "\n";
}

void SDF::allocateMemoryForSDF()
{
    // Initialize voxel grid
    int num_grid_elements = m_gridSize.prod();
    m_voxelGridTSDF.resize(num_grid_elements);
    m_voxelGridWeight.resize(num_grid_elements);
    // Set the distance to 1, since 1 represents each point is too far from the surface.
    // This is done for any voxel which had no correspondences in the image and thus was never processed.
    std::fill(m_voxelGridTSDF.begin(), m_voxelGridTSDF.end(), 1.0f);
    std::fill(m_voxelGridWeight.begin(), m_voxelGridWeight.end(), 0);
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
                m_voxelGridTSDF.at(index) =
                    (m_voxelGridTSDF.at(index) * m_voxelGridWeight.at(index) + dist) / 
                    (m_voxelGridWeight.at(index) + 1);
                m_voxelGridWeight.at(index) += 1;
            }
        }
    }
}

void SDF::fuse(const SDF *otherSdf)
{
    for(size_t voxelIndex = 0; voxelIndex < m_gridSize.prod(); voxelIndex++)
    {
        long w2 = otherSdf->m_voxelGridWeight.at(voxelIndex);
        // Ignore voxels that are at distance -1 behind the surface in otherSDF. No change needed.
        if (w2 == 0) {
            continue;
        }

        long w1 = m_voxelGridWeight.at(voxelIndex);
        float dist2 = otherSdf->m_voxelGridTSDF.at(voxelIndex);
        if (w1 == 0) {
            m_voxelGridTSDF.at(voxelIndex) = dist2;
            m_voxelGridWeight.at(voxelIndex) = w2;
        } else {
            float dist1 = m_voxelGridTSDF.at(voxelIndex);
            m_voxelGridTSDF.at(voxelIndex) = (w1 * dist1 + w2 * dist2) / (w1+w2);
            m_voxelGridWeight.at(voxelIndex) = (w1+w2);
        }
    }
}

void SDF::fuse(const SDF *otherSdf, const DisplacementField *otherDisplacementField)
{

}

void SDF::dumpToBinFile(string outputFilePath,
                        float truncationDistanceInVoxelSizeUnit,
                        float minimumWeightThreshold)
{
    // Save TSDF voxel grid and its parameters to disk as binary file (float array)
    cout << "============================================================================\n";
    cout << "Saving TSDF voxel grid values at " << outputFilePath << "\n";
    ofstream outFile(outputFilePath, ios::binary | ios::out);
    outFile.write((char *) m_gridSize.data(), 3 * sizeof(int));
    outFile.write((char *) m_min3dLoc.data(), 3 * sizeof(float));
    outFile.write((char *) &m_voxelSize, sizeof(float));
    outFile.write((char *) &truncationDistanceInVoxelSizeUnit, sizeof(float));
    outFile.write((char *)(&m_voxelGridTSDF[0]), m_gridSize.prod() * sizeof(float));
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

//  ToDo - Move to utils
float interpolate1D(float v_0, float v_1, float x)
{
    return v_0 * (1 - x) + v_1 * x;
}

float interpolate2D(float v_00, float v_01, float v_10, float v_11, float x, float y)
{

    float s = interpolate1D(v_00, v_01, x);
    float t = interpolate1D(v_10, v_11, x);
    return interpolate1D(s, t, y);
}

float interpolate3D(float v_000, float v_001, float v_010, float v_011,
                    float v_100, float v_101, float v_110, float v_111,
                    float x, float y, float z)
{
    float s = interpolate2D(v_000, v_001, v_010, v_011, x, y);
    float t = interpolate2D(v_100, v_101, v_110, v_111, x, y);
    return interpolate1D(s, t, z);
}

float SDF::getDistanceAtIndex(const Eigen::Vector3i &gridSpatialIndex) const
{
    return m_voxelGridTSDF.at(gridSpatialIndex.dot(m_gridSpacingPerAxis));
}

float SDF::getDistance(const Eigen::Vector3f &gridLocation) const
{
    // Substract 0.5f since, grid index (x,y,z) stores distance value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3f trueGridLocation = gridLocation.array() - 0.5f;
    if ((gridLocation.array() < 0.0f).any() ||
        (gridLocation.array() >= m_gridSize.array().cast<float>()).any())
        return -1;

    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = trueGridLocation.cast<int>();
    float vertex_000 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    float vertex_001 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    float vertex_010 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    float vertex_011 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    float vertex_100 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    float vertex_101 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    float vertex_110 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    float vertex_111 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3f weights = trueGridLocation - bottomLeftFrontIndex.cast<float>();
    return interpolate3D(vertex_000, vertex_001, vertex_010, vertex_011,
                  vertex_100, vertex_101, vertex_110, vertex_111, 
                  weights(0), weights(1), weights(2));
}
