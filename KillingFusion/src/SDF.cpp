//
// Created by Saurabh Khanduja on 26.10.18.
//

#include <sstream>
#include <iomanip>
#include "SDF.h"
#include "config.h"
#include "SimpleMesh.h"
#include "MarchingCubes.h"
#include "utils.h"
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
    Eigen::Vector3f canSdfSphereCenter = (canSdf.m_gridSize / 2).cast<float>() + Eigen::Vector3f(0.5, 0.5, -0.5);
    cout << "Center for First Sphere is " << canSdfSphereCenter.transpose() << endl;
    Eigen::Vector3f nextSdfSphereCenter = (canSdf.m_gridSize / 2).cast<float>() + Eigen::Vector3f(0.5, 0.5, 0.5);
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

                // If depth image is valid at the pixel
                float voxelCenterDepth = voxelCenterInCamera(2);
                float depth = depthFrame.at<float>(row, col);
                if (depth < minDepth || depth > maxDepth)
                    continue;

                // Distance from surface
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
            continue;

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
#ifndef MY_DEBUG
#pragma omp parallel for
#endif
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
                    continue;         // Make no change to this voxel.

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

void SDF::update(const DisplacementField *displacementField)
{
    std::vector<float> newVoxelGridTSDF;
    std::vector<long> newVoxelGridWeight;
    newVoxelGridTSDF.resize(m_totalNumberOfVoxels);
    newVoxelGridWeight.resize(m_totalNumberOfVoxels);
    std::fill(newVoxelGridTSDF.begin(), newVoxelGridTSDF.end(), 1.0f);
    std::fill(newVoxelGridWeight.begin(), newVoxelGridWeight.end(), 0);

    for (int z = 0; z < m_gridSize(2); z++)
    {
        for (int y = 0; y < m_gridSize(1); y++)
        {
            for (int x = 0; x < m_gridSize(0); x++)
            {
                Eigen::Vector3f sdfLocation = Eigen::Vector3f(x + 0.5f, y + 0.5f, z + 0.5f) + displacementField->getDisplacementAt(x, y, z);
                float w = getWeight(sdfLocation);
                float dist = getDistance(sdfLocation);
                int voxelIndex = z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x;
                newVoxelGridTSDF.at(voxelIndex) = dist;
                newVoxelGridWeight.at(voxelIndex) = w;
            }
        }
    }

    // Swap the current SDF and weights with the new computed SDF and weights
    m_voxelGridTSDF.swap(newVoxelGridTSDF);
    m_voxelGridWeight.swap(newVoxelGridWeight);
}

bool SDF::ProcessVolumeCell(int x, int y, int z, double iso, SimpleMesh *mesh) const
{
    MC_Gridcell cell;

    Eigen::Vector3d tmp;

    // cell corners
    Eigen::Vector3d halfGridSize = (m_gridSize.cast<double>() / 2);
    cell.p[0] = (Eigen::Vector3d(x + 1, y, z).array() / halfGridSize.array()) - 1;
    cell.p[1] = (Eigen::Vector3d(x, y, z).array() / halfGridSize.array()) - 1;
    cell.p[2] = (Eigen::Vector3d(x, y + 1, z).array() / halfGridSize.array()) - 1;
    cell.p[3] = (Eigen::Vector3d(x + 1, y + 1, z).array() / halfGridSize.array()) - 1;
    cell.p[4] = (Eigen::Vector3d(x + 1, y, z + 1).array() / halfGridSize.array()) - 1;
    cell.p[5] = (Eigen::Vector3d(x, y, z + 1).array() / halfGridSize.array()) - 1;
    cell.p[6] = (Eigen::Vector3d(x, y + 1, z + 1).array() / halfGridSize.array()) - 1;
    cell.p[7] = (Eigen::Vector3d(x + 1, y + 1, z + 1).array() / halfGridSize.array()) - 1;

    // cell corner values
    cell.val[0] = (double)getDistanceAtIndex(x + 1, y, z);
    cell.val[1] = (double)getDistanceAtIndex(x, y, z);
    cell.val[2] = (double)getDistanceAtIndex(x, y + 1, z);
    cell.val[3] = (double)getDistanceAtIndex(x + 1, y + 1, z);
    cell.val[4] = (double)getDistanceAtIndex(x + 1, y, z + 1);
    cell.val[5] = (double)getDistanceAtIndex(x, y, z + 1);
    cell.val[6] = (double)getDistanceAtIndex(x, y + 1, z + 1);
    cell.val[7] = (double)getDistanceAtIndex(x + 1, y + 1, z + 1);

    MC_Triangle tris[6];
    int numTris = Polygonise(cell, iso, tris);

    if (numTris == 0)
        return false;

    for (int i1 = 0; i1 < numTris; i1++)
    {
        Vertex v0((float)tris[i1].p[0][0], (float)tris[i1].p[0][1], (float)tris[i1].p[0][2]);
        Vertex v1((float)tris[i1].p[1][0], (float)tris[i1].p[1][1], (float)tris[i1].p[1][2]);
        Vertex v2((float)tris[i1].p[2][0], (float)tris[i1].p[2][1], (float)tris[i1].p[2][2]);

        unsigned int vhandle[3];
        vhandle[0] = mesh->AddVertex(v0);
        vhandle[1] = mesh->AddVertex(v1);
        vhandle[2] = mesh->AddVertex(v2);

        mesh->AddFace(vhandle[0], vhandle[1], vhandle[2]);
    }

    return true;
}

void SDF::save_mesh(std::string mesh_name_prefix,
                    int fileCounter) const
{
    std::ostringstream filenameOutStream;
    filenameOutStream << OUTPUT_DIR << outputDir[datasetType] << mesh_name_prefix << std::setw(3) << std::setfill('0') << fileCounter << ".off";
    std::string filenameOut = filenameOutStream.str();

    SimpleMesh *mesh = getMesh();
    // write mesh to file
    if (!mesh->WriteMesh(filenameOut))
    {
        std::cout << "ERROR: unable to write output file at" << filenameOut << "!" << std::endl;
    }
    delete mesh;
}

SimpleMesh *SDF::getMesh() const
{
    SimpleMesh *mesh = new SimpleMesh();
    for (int z = 0; z < m_gridSize(2); z++)
    {
        for (int y = 0; y < m_gridSize(1); y++)
        {
            for (int x = 0; x < m_gridSize(0); x++)
            {
                float distance = getDistanceAtIndex(x, y, z);
                // if (distance > MaxSurfaceVoxelDistance || distance < -MaxSurfaceVoxelDistance)
                if (distance > 3 * m_voxelSize || distance < -3 * m_voxelSize)
                    continue;
                ProcessVolumeCell(x, y, z, 0.00f, mesh);
            }
        }
    }
    return mesh;
}

SimpleMesh *SDF::getMesh(const DisplacementField &displacementField) const
{
    SDF deformedSdf(m_voxelSize, m_min3dLoc, m_max3dLoc, m_unknownClipDistance);
    deformedSdf.fuse(this, &displacementField);
    return deformedSdf.getMesh();
}

void SDF::save_mesh(std::string mesh_name_prefix,
                    int fileCounter,
                    const DisplacementField &displacementField) const
{
    SDF deformedSdf(m_voxelSize, m_min3dLoc, m_max3dLoc, m_unknownClipDistance);
    deformedSdf.fuse(this, &displacementField);
    deformedSdf.save_mesh(mesh_name_prefix, fileCounter);
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

float SDF::getDistance(const Eigen::Vector3i &spatialIndex,
                       const DisplacementField *displacementField) const
{
    Eigen::Vector3f displacedGridLocation = spatialIndex.cast<float>() + displacementField->getDisplacementAt(spatialIndex) + Eigen::Vector3f(0.5, 0.5, 0.5);
    return getDistance(displacedGridLocation);
}

float SDF::getDistancef(const Eigen::Vector3f &gridLocation,
                        const DisplacementField *displacementField) const
{
    Eigen::Vector3f displacedGridLocation = gridLocation + displacementField->getDisplacementAtf(gridLocation) + Eigen::Vector3f(0.5, 0.5, 0.5);
    return getDistance(displacedGridLocation);
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

float SDF::getWeight(const Eigen::Vector3i &spatialIndex,
                     const DisplacementField *displacementField) const
{
    Eigen::Vector3f displacedGridLocation = spatialIndex.cast<float>() + displacementField->getDisplacementAt(spatialIndex) + Eigen::Vector3f(0.5, 0.5, 0.5);
    return getWeight(displacedGridLocation);
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

Eigen::Vector3f SDF::computeDistanceGradient(const Eigen::Vector3i &spatialIndex,
                                             const DisplacementField *displacementField) const
{
    Eigen::Vector3f gridLocation = spatialIndex.cast<float>() + Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    const Eigen::Vector3f forwardDiffDelta[3] = {deltaSize * Eigen::Vector3f(1, 0, 0),
                                                 deltaSize * Eigen::Vector3f(0, 1, 0),
                                                 deltaSize * Eigen::Vector3f(0, 0, 1)};

    Eigen::Vector3f gradient(0, 0, 0);
    // getDistance will take care of substracting 0.5, so pass gridLocation below
    gradient(0) = getDistancef(gridLocation + forwardDiffDelta[0], displacementField) -
                  getDistancef(gridLocation - forwardDiffDelta[0], displacementField);
    gradient(1) = getDistancef(gridLocation + forwardDiffDelta[1], displacementField) -
                  getDistancef(gridLocation - forwardDiffDelta[1], displacementField);
    gradient(2) = getDistancef(gridLocation + forwardDiffDelta[2], displacementField) -
                  getDistancef(gridLocation - forwardDiffDelta[2], displacementField);

    return gradient / (2 * deltaSize * m_voxelSize);
}

Eigen::Matrix3f SDF::computeDistanceHessian(const Eigen::Vector3f &gridLocation) const
{
    Eigen::Vector3f trueGridLocation = gridLocation.array() - 0.5f;

    // Check if trueGridLocation is in the grid
    if ((gridLocation.array() < 0.0f).any() ||
        (gridLocation.array() >= m_gridSize.array().cast<float>()).any())
        return Eigen::Matrix3f::Zero();

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

Eigen::Matrix3f SDF::computeDistanceHessian(const Eigen::Vector3i &spatialIndex,
                                            const DisplacementField *displacementField) const
{
    Eigen::Vector3f gridLocation = spatialIndex.cast<float>() + Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    // ToDo: Optimize
    float fxyz = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, 0, 0), displacementField);
    float fxplus2yz = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(2, 0, 0), displacementField);
    float fxminus2yz = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(-2, 0, 0), displacementField);
    float fxyplus2z = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, 2, 0), displacementField);
    float fxyminus2z = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, -2, 0), displacementField);
    float fxyzplus2 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, 0, 2), displacementField);
    float fxyzminus2 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, 0, -2), displacementField);
    float fxplus1yzplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(1, 0, 1), displacementField);
    float fxminus1yzplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(-1, 0, 1), displacementField);
    float fxyplus1zplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, 1, 1), displacementField);
    float fxyminus1zplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, -1, 1), displacementField);
    float fxplus1yplus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(1, 1, 0), displacementField);
    float fxminus1yplus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(-1, 1, 0), displacementField);
    float fxminus1yzminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(-1, 0, -1), displacementField);
    float fxplus1yzminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(1, 0, -1), displacementField);
    float fxyminus1zminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, -1, -1), displacementField);
    float fxyplus1zminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(0, 1, -1), displacementField);
    float fxminus1yminus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(-1, -1, 0), displacementField);
    float fxplus1yminus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3f(1, -1, 0), displacementField);

    float fxx, fyy, fzz, fxy, fxz, fyz;
    float denominator = (4 * deltaSize * deltaSize * m_voxelSize * m_voxelSize); // 4h^2, where h is step size.
    fxx = (fxplus2yz - 2 * fxyz + fxminus2yz);
    fyy = (fxyplus2z - 2 * fxyz + fxyminus2z);
    fzz = (fxyzplus2 - 2 * fxyz + fxyzminus2);
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
