//
// Created by Saurabh Khanduja on 26.10.18.
//

#include "SDF.h"
#include "config.h"
using namespace std;

SDF::SDF(float _voxelSize,
         Eigen::Vector3f _min3dLoc,
         Eigen::Vector3f _max3dLoc,
         float unknownClipDistance)
    : m_voxelSize{_voxelSize},
      m_bound{_max3dLoc - _min3dLoc}
{
    m_min3dLoc = _min3dLoc;
    m_max3dLoc = _max3dLoc;

    m_unknownClipDistance = unknownClipDistance;

    computeVoxelGridSize(); // Sets m_totalNumberOfVoxels and m_gridSize
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
    m_totalNumberOfVoxels = m_gridSize.prod();
    allocateMemoryForSDF();
}

SDF::SDF(const SDF &copy) noexcept
{
    m_voxelSize = copy.m_voxelSize;
    m_bound = copy.m_bound;
    m_min3dLoc = copy.m_min3dLoc;
    m_max3dLoc = copy.m_max3dLoc;
    m_unknownClipDistance = copy.m_unknownClipDistance;
    m_gridSize = copy.m_gridSize;
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
    m_totalNumberOfVoxels = m_gridSize.prod();
    allocateMemoryForSDF();
}

SDF::SDF(SDF &&other) noexcept
    : m_voxelGridTSDF(std::move(other.m_voxelGridTSDF)),
      m_voxelGridWeight(std::move(other.m_voxelGridWeight))
{
    m_voxelSize = other.m_voxelSize;
    m_bound = other.m_bound;
    m_min3dLoc = other.m_min3dLoc;
    m_max3dLoc = other.m_max3dLoc;
    m_unknownClipDistance = other.m_unknownClipDistance;
    m_gridSize = other.m_gridSize;
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
    m_totalNumberOfVoxels = m_gridSize.prod();
}

SDF::~SDF()
{
}

// ToDo - Add generateSphere method. Make it more generic.
vector<SDF> SDF::getDataEnergyTestSample(float _voxelSize,
                                         float unknownClipDistance)
{
    Eigen::Vector3f _min3dLoc = Eigen::Vector3f::Zero();
    Eigen::Vector3f _max3dLoc = Eigen::Vector3f::Zero().array() + _voxelSize * 9;
    SDF canSdf(_voxelSize, _min3dLoc, _max3dLoc, unknownClipDistance);
    SDF nextSdf(_voxelSize, _min3dLoc, _max3dLoc, unknownClipDistance);

    // Build Sphere with different center
    Eigen::Vector3f canSdfSphereCenter = (canSdf.m_gridSize / 2).cast<float>() + Eigen::Vector3f(0.5,0.5,-0.5);
    cout << "Center for First Sphere is " << canSdfSphereCenter.transpose() << endl;
    Eigen::Vector3f nextSdfSphereCenter = (canSdf.m_gridSize / 2).cast<float>() + Eigen::Vector3f(0.5,0.5,0.5);
    cout << "Center for Second Sphere is " << nextSdfSphereCenter.transpose() << endl;
    float radius = 3 * canSdf.m_voxelSize;
    for (int z = 0; z < canSdf.m_gridSize(2); z++)
    {
        for (int y = 0; y < canSdf.m_gridSize(1); y++)
        {
            for (int x = 0; x < canSdf.m_gridSize(0); x++)
            {
                Eigen::Vector3f voxelLocation = Eigen::Vector3f(x + 0.5f, y + 0.5f, z + 0.5f);
                int voxelIndex = z * canSdf.m_gridSpacingPerAxis(2) + y * canSdf.m_gridSpacingPerAxis(1) + x;
                float canSdfDistance = (canSdfSphereCenter - voxelLocation).norm() * _voxelSize - radius;
                float nextSdfDistance = (nextSdfSphereCenter - voxelLocation).norm() * _voxelSize - radius;
                if (canSdfDistance >= -unknownClipDistance)
                {
                    canSdf.m_voxelGridTSDF.at(voxelIndex) = canSdfDistance;
                    canSdf.m_voxelGridWeight.at(voxelIndex) = 1;
                }
                else
                {
                    canSdf.m_voxelGridWeight.at(voxelIndex) = 0;
                }

                if (nextSdfDistance >= -unknownClipDistance)
                {
                    nextSdf.m_voxelGridTSDF.at(voxelIndex) = nextSdfDistance;
                    nextSdf.m_voxelGridWeight.at(voxelIndex) = 1;
                }
                else
                {
                    nextSdf.m_voxelGridWeight.at(voxelIndex) = 0;
                }
            }
        }
    }

    std::vector<SDF> sdfs;
    sdfs.push_back(std::move(canSdf));
    sdfs.push_back(std::move(nextSdf));
    return sdfs;
}

void SDF::computeVoxelGridSize()
{
    // cout << "Bound is: " << m_bound.transpose() << "\n";
    // cout << "Voxel Size is: " << m_voxelSize << "\n";
    // Casting may reduce size by 1.
    m_gridSize = ((m_bound / m_voxelSize)).cast<int>().array() + 1;
    // cout << "Grid Size computed is: " << m_gridSize.transpose() << "\n";
}

void SDF::allocateMemoryForSDF()
{
    // Initialize voxel grid
    m_voxelGridTSDF.resize(m_totalNumberOfVoxels);
    m_voxelGridWeight.resize(m_totalNumberOfVoxels);
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

    for (int z = 0; z < m_gridSize(2); z++)
    {
        float Z = m_voxelSize * (z + 0.5f) + m_min3dLoc(2);
        for (int y = 0; y < m_gridSize(1); y++)
        {
            float Y = m_voxelSize * (y + 0.5f) + m_min3dLoc(1);
            for (int x = 0; x < m_gridSize(0); x++)
            {
                float X = m_voxelSize * (x + 0.5f) + m_min3dLoc(0);
                // Compute 3d location of center of voxel
                // Backproject it to the depth image
                Eigen::Vector4f voxelCenter(X, Y, Z, 1);
                Eigen::Vector4f voxelCenterInCamera = world_to_camera_pose * voxelCenter;
                Eigen::Vector3f voxelPixelLocation(voxelCenterInCamera(0) / voxelCenterInCamera(2),
                                                   voxelCenterInCamera(1) / voxelCenterInCamera(2), 1);
                voxelPixelLocation = depthIntrinsicMatrix * voxelPixelLocation;

                // If pixel is outside the image, check next voxel
                int col = roundf(voxelPixelLocation(0) + 0.5f);
                if (col < 0 || col >= depthFrame.cols)
                    continue;

                int row = roundf(voxelPixelLocation(1) + 0.5f);
                if (row < 0 || row >= depthFrame.rows)
                    continue;

                uchar isForegroundPixel = maskFrame.at<uchar>(row, col);
                if (isForegroundPixel == 0)
                    continue;

                // If depth image is valid at the pixel
                float voxelCenterDepth = voxelCenterInCamera(2);
                float depth = depthFrame.at<float>(row, col);
                if (depth < minDepth || depth > maxDepth)
                    continue;

                float dist = depth - voxelCenterDepth;

                // Check if diff is too negative: m_unknownClipDistance
                if (dist < -m_unknownClipDistance) // Ignore any voxel at certain threshold behind the surface.
                    continue;

                // Merge the signed distance in this voxel
                int index = z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x;
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
#ifndef MY_DEBUG
#pragma omp parallel for
#endif
    for (size_t voxelIndex = 0; voxelIndex < m_totalNumberOfVoxels; voxelIndex++)
    {
        long w2 = otherSdf->m_voxelGridWeight.at(voxelIndex);
        // Ignore voxels that are at distance -1 behind the surface in otherSDF. No change needed.
        if (w2 == 0)
        {
            continue;
        }

        long w1 = m_voxelGridWeight.at(voxelIndex);
        float dist2 = otherSdf->m_voxelGridTSDF.at(voxelIndex);
        if (w1 == 0)
        {
            m_voxelGridTSDF.at(voxelIndex) = dist2;
            m_voxelGridWeight.at(voxelIndex) = w2;
        }
        else
        {
            float dist1 = m_voxelGridTSDF.at(voxelIndex);
            if (FUSE_BY_MERGE)
            { // If voxels are aligned, this addition makes sense
                m_voxelGridTSDF.at(voxelIndex) = (w1 * dist1 + w2 * dist2) / (w1 + w2);
                m_voxelGridWeight.at(voxelIndex) = (w1 + w2);
            }
            else
            { // If you think of SDF mathematically, this is how you should fuse.
                if (fabs(dist1) < fabs(dist2))
                {
                    m_voxelGridTSDF.at(voxelIndex) = dist1;
                    m_voxelGridWeight.at(voxelIndex) = w1;
                }
                else
                {
                    m_voxelGridTSDF.at(voxelIndex) = dist2;
                    m_voxelGridWeight.at(voxelIndex) = w2;
                }
            }
        }
    }
}

void SDF::fuse(const SDF *otherSdf, const DisplacementField *otherDisplacementField)
{
#pragma omp parallel for
    for (int z = 0; z < m_gridSize(2); z++)
    {
        for (int y = 0; y < m_gridSize(1); y++)
        {
            for (int x = 0; x < m_gridSize(0); x++)
            {
                Eigen::Vector3f otherSdfIndex = Eigen::Vector3f(x + 0.5f, y + 0.5f, z + 0.5f) +
                                                otherDisplacementField->getDisplacementAt(x, y, z);
                // Tricky Part: We need to first get weight at otherSdfIndex.
                float w2 = otherSdf->getWeight(otherSdfIndex);
                // Ignore voxels that are at distance -1 behind the surface in otherSDF. No change needed.
                // http://realtimecollisiondetection.net/blog/?p=89
                if (fabs(w2) < 1e-5f) // Nearly Zero
                {
                    continue; // Make no change to this SDF.
                }

                long w1 = getWeightAtIndex(x, y, z);
                float dist2 = otherSdf->getDistance(otherSdfIndex);
                int voxelIndex = z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x;
                if (w1 == 0)
                {
                    m_voxelGridTSDF.at(voxelIndex) = dist2;
                    m_voxelGridWeight.at(voxelIndex) = w2;
                }
                else
                {
                    float dist1 = m_voxelGridTSDF.at(voxelIndex);
                    if (FUSE_BY_MERGE)
                    { // If voxels are aligned, this addition makes sense
                        m_voxelGridTSDF.at(voxelIndex) = (w1 * dist1 + w2 * dist2) / (w1 + w2);
                        m_voxelGridWeight.at(voxelIndex) = (w1 + w2);
                    }
                    else
                    { // If you think of SDF mathematically, this is how you should fuse.
                        if (fabs(dist1) < fabs(dist2))
                        {
                            m_voxelGridTSDF.at(voxelIndex) = dist1;
                            m_voxelGridWeight.at(voxelIndex) = w1;
                        }
                        else
                        {
                            m_voxelGridTSDF.at(voxelIndex) = dist2;
                            m_voxelGridWeight.at(voxelIndex) = w2;
                        }
                    }
                }
            }
        }
    }
}

void SDF::dumpToBinFile(string outputFilePath,
                        float truncationDistanceInVoxelSizeUnit,
                        float minimumWeightThreshold) const
{
    // Save TSDF voxel grid and its parameters to disk as binary file (float array)
    cout << "============================================================================\n";
    cout << "Saving TSDF voxel grid values at " << outputFilePath << "\n";
    ofstream outFile(outputFilePath, ios::binary | ios::out);
    outFile.write((char *)m_gridSize.data(), 3 * sizeof(int));
    outFile.write((char *)m_min3dLoc.data(), 3 * sizeof(float));
    outFile.write((char *)&m_voxelSize, sizeof(float));
    outFile.write((char *)&truncationDistanceInVoxelSizeUnit, sizeof(float));
    outFile.write((char *)(&m_voxelGridTSDF[0]), m_totalNumberOfVoxels * sizeof(float));
    outFile.close();

    float min = m_voxelGridTSDF[0];
    float max = m_voxelGridTSDF[0];
    for (size_t i = 0; i < m_totalNumberOfVoxels; i++)
    {
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

bool SDF::indexInGridBounds(int x, int y, int z) const
{
    return x >= 0 && x < m_gridSize(0) &&
           y >= 0 && y < m_gridSize(1) &&
           z >= 0 && z < m_gridSize(2);
}

bool SDF::indexInGridBounds(const Eigen::Vector3i &gridSpatialIndex) const
{
    return ((gridSpatialIndex.array() >= 0).all() &&
            (gridSpatialIndex.array() < m_gridSize.array()).all());
}

float SDF::getDistanceAtIndex(const Eigen::Vector3i &gridSpatialIndex) const
{
    if (!indexInGridBounds(gridSpatialIndex))
    {
        return -1;
    }
    return m_voxelGridTSDF.at(gridSpatialIndex.dot(m_gridSpacingPerAxis));
}

float SDF::getDistanceAtIndex(int x, int y, int z) const
{
    if (!indexInGridBounds(x, y, z))
    {
        return -1;
    }
    return m_voxelGridTSDF.at(z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x);
}

long SDF::getWeightAtIndex(const Eigen::Vector3i &gridSpatialIndex) const
{
    if (!indexInGridBounds(gridSpatialIndex))
    {
        return 0;
    }
    return m_voxelGridWeight.at(gridSpatialIndex.dot(m_gridSpacingPerAxis));
}

long SDF::getWeightAtIndex(int x, int y, int z) const
{
    if (!indexInGridBounds(x, y, z))
    {
        return 0;
    }
    return m_voxelGridWeight.at(z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x);
}

float SDF::getDistance(const Eigen::Vector3f &gridLocation) const
{
    // Substract 0.5f since, grid index (x,y,z) stores distance value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3f trueGridLocation = gridLocation.array() - 0.5f;

    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = trueGridLocation.cast<int>();

    // ToDo - Compute indices yourself to make faster. Done as below to get implementation correct first.
    float vertex_000 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    float vertex_001 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    float vertex_010 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    float vertex_011 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    float vertex_100 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    float vertex_101 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    float vertex_110 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    float vertex_111 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3f interpolationWeights = trueGridLocation - bottomLeftFrontIndex.cast<float>();
    return interpolate3D(vertex_000, vertex_001, vertex_010, vertex_011,
                         vertex_100, vertex_101, vertex_110, vertex_111,
                         interpolationWeights(0), interpolationWeights(1), interpolationWeights(2));
}

float SDF::getWeight(const Eigen::Vector3f &gridLocation) const
{
    // Substract 0.5f since, grid index (x,y,z) stores weight value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3f trueGridLocation = gridLocation.array() - 0.5f;

    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = trueGridLocation.cast<int>();
    // ToDo - Compute indices yourself to make faster. Done as below to get implementation correct first.
    float vertex_000 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    float vertex_001 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    float vertex_010 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    float vertex_011 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    float vertex_100 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    float vertex_101 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    float vertex_110 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    float vertex_111 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3f interpolationWeights = trueGridLocation - bottomLeftFrontIndex.cast<float>();
    return interpolate3D(vertex_000, vertex_001, vertex_010, vertex_011,
                         vertex_100, vertex_101, vertex_110, vertex_111,
                         interpolationWeights(0), interpolationWeights(1), interpolationWeights(2));
}

Eigen::Vector3f SDF::computeDistanceGradient(const Eigen::Vector3f &gridLocation) const
{
    // Substract 0.5f since, grid index (x,y,z) stores distance value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3f trueGridLocation = gridLocation.array() - 0.5f;
    // Check if trueGridLocation is in the grid
    if ((gridLocation.array() < 0.0f).any() ||
        (gridLocation.array() >= m_gridSize.array().cast<float>()).any())
        return Eigen::Vector3f::Zero();

    const Eigen::Vector3f forwardDiffDelta[3] = {Eigen::Vector3f(1, 0, 0),
                                                 Eigen::Vector3f(0, 1, 0),
                                                 Eigen::Vector3f(0, 0, 1)};

    Eigen::Vector3f gradient(0, 0, 0);
    // getDistance will take care of substracting 0.5, so pass gridLocation below
    gradient(0) = getDistance(gridLocation + forwardDiffDelta[0]) -
                  getDistance(gridLocation - forwardDiffDelta[0]);
    gradient(1) = getDistance(gridLocation + forwardDiffDelta[1]) -
                  getDistance(gridLocation - forwardDiffDelta[1]);
    gradient(2) = getDistance(gridLocation + forwardDiffDelta[2]) -
                  getDistance(gridLocation - forwardDiffDelta[2]);

    return gradient / (2 * m_voxelSize);
}

Eigen::Matrix3f SDF::computeDistanceHessian(const Eigen::Vector3f &gridLocation) const
{
    Eigen::Vector3f trueGridLocation = gridLocation.array() - 0.5f;

    // Check if trueGridLocation is in the grid
    if ((gridLocation.array() < 0.0f).any() ||
        (gridLocation.array() >= m_gridSize.array().cast<float>()).any())
        return Eigen::Matrix3f::Zero();

    // https://en.wikipedia.org/wiki/Finite_difference#Finite_difference_in_several_variables
    // ToDo: Optimize - Ideally compute Gradient for whole SDF grid and the hessian of whole SDF grid. Reduce the repeat computation
    float fxyz = getDistance(gridLocation);
    float fxplus2yz = getDistance(gridLocation + Eigen::Vector3f(2, 0, 0));
    float fxminus2yz = getDistance(gridLocation + Eigen::Vector3f(-2, 0, 0));
    float fxyplus2z = getDistance(gridLocation + Eigen::Vector3f(0, 2, 0));
    float fxyminus2z = getDistance(gridLocation + Eigen::Vector3f(0, -2, 0));
    float fxyzplus2 = getDistance(gridLocation + Eigen::Vector3f(0, 0, 2));
    float fxyzminus2 = getDistance(gridLocation + Eigen::Vector3f(0, 0, -2));
    float fxplus1yzplus1 = getDistance(gridLocation + Eigen::Vector3f(1, 0, 1));
    float fxminus1yzplus1 = getDistance(gridLocation + Eigen::Vector3f(-1, 0, 1));
    float fxyplus1zplus1 = getDistance(gridLocation + Eigen::Vector3f(0, 1, 1));
    float fxyminus1zplus1 = getDistance(gridLocation + Eigen::Vector3f(0, -1, 1));
    float fxplus1yplus1z = getDistance(gridLocation + Eigen::Vector3f(1, 1, 0));
    float fxminus1yplus1z = getDistance(gridLocation + Eigen::Vector3f(-1, 1, 0));
    float fxminus1yzminus1 = getDistance(gridLocation + Eigen::Vector3f(-1, 0, -1));
    float fxplus1yzminus1 = getDistance(gridLocation + Eigen::Vector3f(1, 0, -1));
    float fxyminus1zminus1 = getDistance(gridLocation + Eigen::Vector3f(0, -1, -1));
    float fxyplus1zminus1 = getDistance(gridLocation + Eigen::Vector3f(0, 1, -1));
    float fxminus1yminus1z = getDistance(gridLocation + Eigen::Vector3f(-1, -1, 0));
    float fxplus1yminus1z = getDistance(gridLocation + Eigen::Vector3f(1, -1, 0));

    float fxx, fyy, fzz, fxy, fxz, fyz;
    float denominator = (4 * m_voxelSize * m_voxelSize); // 4h^2, where h is step size.
    fxx = (fxplus2yz - 2 * fxyz + fxminus2yz);
    fyy = (fxyplus2z - 2 * fxyz + fxyminus2z);
    fyy = (fxyzplus2 - 2 * fxyz + fxyzminus2);
    fxz = (fxplus1yzplus1 + fxminus1yzminus1 - fxplus1yzminus1 - fxminus1yzplus1);
    fyz = (fxyplus1zplus1 + fxyminus1zminus1 - fxyplus1zminus1 - fxyminus1zplus1);
    fxy = (fxplus1yplus1z + fxminus1yminus1z - fxplus1yminus1z - fxminus1yplus1z);

    Eigen::Matrix3f hessian;
    hessian(0, 0) = fxx;
    hessian(1, 1) = fyy;
    hessian(2, 2) = fzz;
    hessian(0, 1) = hessian(1, 0) = fxy;
    hessian(0, 2) = hessian(2, 0) = fxz;
    hessian(1, 2) = hessian(2, 1) = fyz;
    return hessian / denominator;
}
