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

SDF::SDF(double _voxelSize,
         Eigen::Vector3d _min3dLoc,
         Eigen::Vector3d _max3dLoc,
         double unknownClipDistance)
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
vector<SDF> SDF::getDataEnergyTestSample(double _voxelSize,
                                         double unknownClipDistance)
{
    Eigen::Vector3d _min3dLoc = Eigen::Vector3d::Zero();
    Eigen::Vector3d _max3dLoc = Eigen::Vector3d::Zero().array() + _voxelSize * 9;
    SDF canSdf(_voxelSize, _min3dLoc, _max3dLoc, unknownClipDistance);
    SDF nextSdf(_voxelSize, _min3dLoc, _max3dLoc, unknownClipDistance);

    // Build Sphere with different center
    Eigen::Vector3d canSdfSphereCenter = (canSdf.m_gridSize / 2).cast<double>() + Eigen::Vector3d(0.5, 0.5, -0.5);
    cout << "Center for First Sphere is " << canSdfSphereCenter.transpose() << endl;
    Eigen::Vector3d nextSdfSphereCenter = (canSdf.m_gridSize / 2).cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5);
    cout << "Center for Second Sphere is " << nextSdfSphereCenter.transpose() << endl;
    double radius = 3 * canSdf.m_voxelSize;
    for (int z = 0; z < canSdf.m_gridSize(2); z++)
    {
        for (int y = 0; y < canSdf.m_gridSize(1); y++)
        {
            for (int x = 0; x < canSdf.m_gridSize(0); x++)
            {
                Eigen::Vector3d voxelLocation = Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5);
                int voxelIndex = z * canSdf.m_gridSpacingPerAxis(2) + y * canSdf.m_gridSpacingPerAxis(1) + x;
                double canSdfDistance = (canSdfSphereCenter - voxelLocation).norm() * _voxelSize - radius;
                double nextSdfDistance = (nextSdfSphereCenter - voxelLocation).norm() * _voxelSize - radius;
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
    std::fill(m_voxelGridTSDF.begin(), m_voxelGridTSDF.end(), MaxSurfaceVoxelDistance+epsilon);
    std::fill(m_voxelGridWeight.begin(), m_voxelGridWeight.end(), 0);
}

void SDF::integrateDepthFrame(cv::Mat depthFrame,
                              Eigen::Matrix4d depthFrameC2WPose,
                              Eigen::Matrix3d depthIntrinsicMatrix,
                              double minDepth,
                              double maxDepth)
{
    int w = depthFrame.cols;
    int h = depthFrame.rows;

    Eigen::Matrix3d depthIntrinsicMatrixInv = depthIntrinsicMatrix.inverse();
    Eigen::Matrix4d world_to_camera_pose = depthFrameC2WPose.inverse();

    for (int z = 0; z < m_gridSize(2); z++)
    {
        double Z = m_voxelSize * (z + 0.5) + m_min3dLoc(2);
        for (int y = 0; y < m_gridSize(1); y++)
        {
            double Y = m_voxelSize * (y + 0.5) + m_min3dLoc(1);
            for (int x = 0; x < m_gridSize(0); x++)
            {
                double X = m_voxelSize * (x + 0.5) + m_min3dLoc(0);
                // Compute 3d location of center of voxel
                // Backproject it to the depth image
                Eigen::Vector4d voxelCenter(X, Y, Z, 1);
                Eigen::Vector4d voxelCenterInCamera = world_to_camera_pose * voxelCenter;
                Eigen::Vector3d voxelPixelLocation(voxelCenterInCamera(0) / voxelCenterInCamera(2),
                                                   voxelCenterInCamera(1) / voxelCenterInCamera(2), 1);
                voxelPixelLocation = depthIntrinsicMatrix * voxelPixelLocation;

                // If pixel is outside the image, check next voxel
                int col = roundf(voxelPixelLocation(0) + 0.5);
                if (col < 0 || col >= depthFrame.cols)
                    continue;

                int row = roundf(voxelPixelLocation(1) + 0.5);
                if (row < 0 || row >= depthFrame.rows)
                    continue;

                // If depth image is valid at the pixel
                double voxelCenterDepth = voxelCenterInCamera(2);
                double depth = depthFrame.at<double>(row, col);
                if (depth < minDepth || depth > maxDepth)
                    continue;

                // Distance from surface
                double dist = depth - voxelCenterDepth;

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
        double dist2 = otherSdf->m_voxelGridTSDF.at(voxelIndex);
        if (w1 == 0)
        {
            m_voxelGridTSDF.at(voxelIndex) = dist2;
            m_voxelGridWeight.at(voxelIndex) = w2;
        }
        else
        {
            double dist1 = m_voxelGridTSDF.at(voxelIndex);
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

// Ignore this, not used in processNextFrame method.
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
                Eigen::Vector3d otherSdfIndex = Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5) +
                                                otherDisplacementField->getDisplacementAt(x, y, z);
                // Tricky Part: We need to first get weight at otherSdfIndex.
                double w2 = otherSdf->getWeight(otherSdfIndex);
                // Ignore voxels that are at distance -1 behind the surface in otherSDF. No change needed.
                // http://realtimecollisiondetection.net/blog/?p=89
                if (fabs(w2) < 1e-5f) // Nearly Zero
                    continue;         // Make no change to this voxel.

                long w1 = getWeightAtIndex(x, y, z);
                double dist2 = otherSdf->getDistance(otherSdfIndex);
                int voxelIndex = z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x;
                if (w1 == 0)
                {
                    m_voxelGridTSDF.at(voxelIndex) = dist2;
                    m_voxelGridWeight.at(voxelIndex) = w2;
                }
                else
                {
                    double dist1 = m_voxelGridTSDF.at(voxelIndex);
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

// Tested by using displacementField of deltaX, 0, 0
void SDF::update(const DisplacementField *displacementField)
{
    std::vector<double> newVoxelGridTSDF;
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
                Eigen::Vector3d sdfLocation = Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5) + displacementField->getDisplacementAt(x, y, z);
                double w = getWeight(sdfLocation);
                double dist = getDistance(sdfLocation);
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
        Vertex v0((double)tris[i1].p[0][0], (double)tris[i1].p[0][1], (double)tris[i1].p[0][2]);
        Vertex v1((double)tris[i1].p[1][0], (double)tris[i1].p[1][1], (double)tris[i1].p[1][2]);
        Vertex v2((double)tris[i1].p[2][0], (double)tris[i1].p[2][1], (double)tris[i1].p[2][2]);

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
                double distance = getDistanceAtIndex(x, y, z);
                // if (distance > MaxSurfaceVoxelDistance || distance < -MaxSurfaceVoxelDistance)
                if (distance > MaxSurfaceVoxelDistance || distance < -UnknownClipDistance)
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
                        double truncationDistanceInVoxelSizeUnit,
                        double minimumWeightThreshold) const
{
    // Save TSDF voxel grid and its parameters to disk as binary file (double array)
    cout << "============================================================================\n";
    cout << "Saving TSDF voxel grid values at " << outputFilePath << "\n";
    ofstream outFile(outputFilePath, ios::binary | ios::out);
    outFile.write((char *)m_gridSize.data(), 3 * sizeof(int));
    outFile.write((char *)m_min3dLoc.data(), 3 * sizeof(double));
    outFile.write((char *)&m_voxelSize, sizeof(double));
    outFile.write((char *)&truncationDistanceInVoxelSizeUnit, sizeof(double));
    outFile.write((char *)(&m_voxelGridTSDF[0]), m_totalNumberOfVoxels * sizeof(double));
    outFile.close();

    double min = m_voxelGridTSDF[0];
    double max = m_voxelGridTSDF[0];
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

double SDF::getDistanceAtIndex(const Eigen::Vector3i &gridSpatialIndex) const
{
    if (!indexInGridBounds(gridSpatialIndex))
        return MaxSurfaceVoxelDistance+epsilon;
    return m_voxelGridTSDF.at(gridSpatialIndex.dot(m_gridSpacingPerAxis));
}

double SDF::getDistanceAtIndex(int x, int y, int z) const
{
    if (!indexInGridBounds(x, y, z))
        return MaxSurfaceVoxelDistance+epsilon;
    return m_voxelGridTSDF.at(z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x);
}

long SDF::getWeightAtIndex(const Eigen::Vector3i &gridSpatialIndex) const
{
    if (!indexInGridBounds(gridSpatialIndex))
        return 0;
    return m_voxelGridWeight.at(gridSpatialIndex.dot(m_gridSpacingPerAxis));
}

long SDF::getWeightAtIndex(int x, int y, int z) const
{
    if (!indexInGridBounds(x, y, z))
        return 0;
    return m_voxelGridWeight.at(z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x);
}

double SDF::getDistance(const Eigen::Vector3d &gridLocation) const
{
    // Substract 0.5 since, grid index (x,y,z) stores distance value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3d trueGridLocation = gridLocation.array() - 0.5;

    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = trueGridLocation.cast<int>();

    // ToDo - Compute indices yourself to make faster. Done as below to get implementation correct first.
    double vertex_000 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    double vertex_001 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    double vertex_010 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    double vertex_011 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    double vertex_100 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    double vertex_101 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    double vertex_110 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    double vertex_111 = getDistanceAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3d interpolationWeights = trueGridLocation - bottomLeftFrontIndex.cast<double>();
    return interpolate3D(vertex_000, vertex_001, vertex_010, vertex_011,
                         vertex_100, vertex_101, vertex_110, vertex_111,
                         interpolationWeights(0), interpolationWeights(1), interpolationWeights(2));
}

void SDF::testGetDistance()
{
    // Since interpolation is used, analytic function f to test should be linear
    double voxelSize = 0.5;
    Eigen::Vector3d min3dLoc(0, 0, 0);
    Eigen::Vector3d max3dLoc(8, 8, 8);
    double unknownClipDistance = 10;
    SDF testSdf(voxelSize, min3dLoc, max3dLoc, UnknownClipDistance);
    cout << testSdf.m_voxelGridTSDF.size() << endl;
    for (int z = 0; z < testSdf.m_gridSize(2); z++)
    {
        for (int y = 0; y < testSdf.m_gridSize(1); y++)
        {
            for (int x = 0; x < testSdf.m_gridSize(0); x++)
            {
                int index = z * testSdf.m_gridSpacingPerAxis(2) + y * testSdf.m_gridSpacingPerAxis(1) + x;
                // if(x == testSdf.m_gridSize(0)-1 && y == testSdf.m_gridSize(1)-1)
                //     cout << "Hi";
                testSdf.m_voxelGridTSDF[index] = x + y - z;
            }
        }
    }

    // ToDo: Add Bounday conditions
    for (int z = 2; z < testSdf.m_gridSize(2) - 2; z++)
    {
        for (int y = 2; y < testSdf.m_gridSize(1) - 2; y++)
        {
            for (int x = 2; x < testSdf.m_gridSize(0) - 2; x++)
            {
                for (int i = 0; i < 10; i++)
                {
                    double delta = i / 10.0;
                    double f1 = x + delta + y - delta - z + delta;
                    Eigen::Vector3d gridLocation(x + 0.5 + delta, y + 0.5 - delta, z + 0.5 - delta);
                    double f2 = testSdf.getDistance(gridLocation);
                    // cout << x << ',' << y << ',' << z << ": Expected" << f1 << ", Actual" << f2 << endl;
                    assert(fabs(f1 - f2) < epsilon && "Whoops, check testGetDistance");
                }
            }
        }
    }
}

double SDF::getDistance(const Eigen::Vector3i &spatialIndex,
                       const DisplacementField *displacementField) const
{
    Eigen::Vector3d displacedGridLocation = spatialIndex.cast<double>() + displacementField->getDisplacementAt(spatialIndex) + Eigen::Vector3d(0.5, 0.5, 0.5);
    return getDistance(displacedGridLocation);
}

double SDF::getDistancef(const Eigen::Vector3d &gridLocation,
                        const DisplacementField *displacementField) const
{
    Eigen::Vector3d displacedGridLocation = gridLocation + displacementField->getDisplacementAtf(gridLocation) + Eigen::Vector3d(0.5, 0.5, 0.5);
    return getDistance(displacedGridLocation);
}

double SDF::getWeight(const Eigen::Vector3d &gridLocation) const
{
    // Substract 0.5 since, grid index (x,y,z) stores weight value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3d trueGridLocation = gridLocation.array() - 0.5;

    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = trueGridLocation.cast<int>();
    // ToDo - Compute indices yourself to make faster. Done as below to get implementation correct first.
    double vertex_000 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    double vertex_001 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    double vertex_010 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    double vertex_011 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    double vertex_100 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    double vertex_101 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    double vertex_110 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    double vertex_111 = getWeightAtIndex(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3d interpolationWeights = trueGridLocation - bottomLeftFrontIndex.cast<double>();
    return interpolate3D(vertex_000, vertex_001, vertex_010, vertex_011,
                         vertex_100, vertex_101, vertex_110, vertex_111,
                         interpolationWeights(0), interpolationWeights(1), interpolationWeights(2));
}

void SDF::testGetWeight()
{
    // Since interpolation is used, analytic function f to test should be linear
    double voxelSize = 0.5;
    Eigen::Vector3d min3dLoc(0, 0, 0);
    Eigen::Vector3d max3dLoc(8, 8, 8);
    double unknownClipDistance = 10;
    SDF testSdf(voxelSize, min3dLoc, max3dLoc, UnknownClipDistance);
    // cout << testSdf.m_voxelGridTSDF.size() << endl;
    for (int z = 0; z < testSdf.m_gridSize(2); z++)
    {
        for (int y = 0; y < testSdf.m_gridSize(1); y++)
        {
            for (int x = 0; x < testSdf.m_gridSize(0); x++)
            {
                int index = z * testSdf.m_gridSpacingPerAxis(2) + y * testSdf.m_gridSpacingPerAxis(1) + x;
                // if(x == testSdf.m_gridSize(0)-1 && y == testSdf.m_gridSize(1)-1)
                //     cout << "Hi";
                testSdf.m_voxelGridWeight[index] = x + y - z;
            }
        }
    }

    // ToDo: Add Bounday conditions
    for (int z = 2; z < testSdf.m_gridSize(2) - 2; z++)
    {
        for (int y = 2; y < testSdf.m_gridSize(1) - 2; y++)
        {
            for (int x = 2; x < testSdf.m_gridSize(0) - 2; x++)
            {
                for (int i = 0; i < 10; i++)
                {
                    double delta = i / 10.0f;
                    double f1 = x + delta + y - delta - z + delta;
                    Eigen::Vector3d gridLocation(x + 0.5 + delta, y + 0.5 - delta, z + 0.5 - delta);
                    double f2 = testSdf.getWeight(gridLocation);
                    // cout << x << ',' << y << ',' << z << ": Expected" << f1 << ", Actual" << f2 << endl;
                    assert(fabs(f1 - f2) < epsilon && "Whoops, check testGetWeights");
                }
            }
        }
    }
}

double SDF::getWeight(const Eigen::Vector3i &spatialIndex,
                     const DisplacementField *displacementField) const
{
    Eigen::Vector3d displacedGridLocation = spatialIndex.cast<double>() + displacementField->getDisplacementAt(spatialIndex) + Eigen::Vector3d(0.5, 0.5, 0.5);
    return getWeight(displacedGridLocation);
}

Eigen::Vector3d SDF::computeDistanceGradient(const Eigen::Vector3d &gridLocation) const
{
    // Substract 0.5 since, grid index (x,y,z) stores distance value for center (x,y,z)+(0.5,0.5,0.5)
    Eigen::Vector3d trueGridLocation = gridLocation.array() - 0.5;
    // Check if trueGridLocation is in the grid
    if ((gridLocation.array() < 0.0f).any() ||
        (gridLocation.array() >= m_gridSize.array().cast<double>()).any())
        return Eigen::Vector3d::Zero();

    const Eigen::Vector3d forwardDiffDelta[3] = {Eigen::Vector3d(1, 0, 0),
                                                 Eigen::Vector3d(0, 1, 0),
                                                 Eigen::Vector3d(0, 0, 1)};

    Eigen::Vector3d gradient(0, 0, 0);
    // getDistance will take care of substracting 0.5, so pass gridLocation below
    for (size_t i = 0; i < 3; i++)
    {
        gradient(i) = getDistance(gridLocation + forwardDiffDelta[i]) -
                      getDistance(gridLocation - forwardDiffDelta[i]);
    }

    return gradient / 2;
}

void SDF::testComputeDistanceGradient()
{
    // Since interpolation is used, analytic function f to test should be linear
    double voxelSize = 0.5;
    Eigen::Vector3d min3dLoc(0, 0, 0);
    Eigen::Vector3d max3dLoc(8, 8, 8);
    double unknownClipDistance = 10;
    SDF testSdf(voxelSize, min3dLoc, max3dLoc, UnknownClipDistance);
    // cout << testSdf.m_voxelGridTSDF.size() << endl;
    for (int z = 0; z < testSdf.m_gridSize(2); z++)
    {
        for (int y = 0; y < testSdf.m_gridSize(1); y++)
        {
            for (int x = 0; x < testSdf.m_gridSize(0); x++)
            {
                int index = z * testSdf.m_gridSpacingPerAxis(2) + y * testSdf.m_gridSpacingPerAxis(1) + x;
                // if(x == testSdf.m_gridSize(0)-1 && y == testSdf.m_gridSize(1)-1)
                //     cout << "Hi";
                testSdf.m_voxelGridTSDF[index] = x + y - z;
            }
        }
    }

    // ToDo: Add Bounday conditions
    for (int z = 2; z < testSdf.m_gridSize(2) - 2; z++)
    {
        for (int y = 2; y < testSdf.m_gridSize(1) - 2; y++)
        {
            for (int x = 2; x < testSdf.m_gridSize(0) - 2; x++)
            {
                for (int i = 0; i < 10; i++)
                {
                    double delta = i / 10.0f;
                    Eigen::Vector3d gridLocation(x + 0.5 + delta, y + 0.5 - delta, z + 0.5 - delta);
                    Eigen::Vector3d actualDistanceGradient = testSdf.computeDistanceGradient(gridLocation);
                    // cout << x << ',' << y << ',' << z << ": Expected: 2, 2, -2, Actual" << actualDistanceGradient.transpose() << endl;
                    assert(fabs(actualDistanceGradient(0) - 1) < epsilon &&
                           fabs(actualDistanceGradient(1) - 1) < epsilon &&
                           fabs(actualDistanceGradient(2) + 1) < epsilon && "Whoops, check SDF::testComputeDistanceGradient");
                }
            }
        }
    }
}

Eigen::Vector3d SDF::computeDistanceGradient(const Eigen::Vector3i &spatialIndex,
                                             const DisplacementField *displacementField) const
{
    Eigen::Vector3d gridLocation = spatialIndex.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5);
    const Eigen::Vector3d forwardDiffDelta[3] = {deltaSize * Eigen::Vector3d(1, 0, 0),
                                                 deltaSize * Eigen::Vector3d(0, 1, 0),
                                                 deltaSize * Eigen::Vector3d(0, 0, 1)};

    Eigen::Vector3d gradient(0, 0, 0);
    // getDistance will take care of substracting 0.5, so pass gridLocation below
    gradient(0) = getDistancef(gridLocation + forwardDiffDelta[0], displacementField) -
                  getDistancef(gridLocation - forwardDiffDelta[0], displacementField);
    gradient(1) = getDistancef(gridLocation + forwardDiffDelta[1], displacementField) -
                  getDistancef(gridLocation - forwardDiffDelta[1], displacementField);
    gradient(2) = getDistancef(gridLocation + forwardDiffDelta[2], displacementField) -
                  getDistancef(gridLocation - forwardDiffDelta[2], displacementField);

    return gradient / (2 * deltaSize);
}

Eigen::Matrix3d SDF::computeDistanceHessian(const Eigen::Vector3d &gridLocation) const
{
    Eigen::Vector3d trueGridLocation = gridLocation.array() - 0.5;

    // Check if trueGridLocation is in the grid
    if ((trueGridLocation.array() < 0.0f).any() ||
        (trueGridLocation.array() >= m_gridSize.array().cast<double>()).any())
        return Eigen::Matrix3d::Zero();

    // ToDo: Optimize - Ideally compute Gradient for whole SDF grid and the hessian of whole SDF grid. Reduce the repeat computation
    double fxyz = getDistance(gridLocation);
    double fxplus2yz = getDistance(gridLocation + deltaSize * Eigen::Vector3d(2, 0, 0));
    double fxminus2yz = getDistance(gridLocation + deltaSize * Eigen::Vector3d(-2, 0, 0));
    double fxyplus2z = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, 2, 0));
    double fxyminus2z = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, -2, 0));
    double fxyzplus2 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, 0, 2));
    double fxyzminus2 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, 0, -2));
    double fxplus1yzplus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(1, 0, 1));
    double fxminus1yzplus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(-1, 0, 1));
    double fxyplus1zplus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, 1, 1));
    double fxyminus1zplus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, -1, 1));
    double fxplus1yplus1z = getDistance(gridLocation + deltaSize * Eigen::Vector3d(1, 1, 0));
    double fxminus1yplus1z = getDistance(gridLocation + deltaSize * Eigen::Vector3d(-1, 1, 0));
    double fxminus1yzminus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(-1, 0, -1));
    double fxplus1yzminus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(1, 0, -1));
    double fxyminus1zminus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, -1, -1));
    double fxyplus1zminus1 = getDistance(gridLocation + deltaSize * Eigen::Vector3d(0, 1, -1));
    double fxminus1yminus1z = getDistance(gridLocation + deltaSize * Eigen::Vector3d(-1, -1, 0));
    double fxplus1yminus1z = getDistance(gridLocation + deltaSize * Eigen::Vector3d(1, -1, 0));

    double fxx, fyy, fzz, fxy, fxz, fyz;
    fxx = (fxplus2yz - 2 * fxyz + fxminus2yz);
    fyy = (fxyplus2z - 2 * fxyz + fxyminus2z);
    fzz = (fxyzplus2 - 2 * fxyz + fxyzminus2);
    fxz = (fxplus1yzplus1 + fxminus1yzminus1 - fxplus1yzminus1 - fxminus1yzplus1);
    fyz = (fxyplus1zplus1 + fxyminus1zminus1 - fxyplus1zminus1 - fxyminus1zplus1);
    fxy = (fxplus1yplus1z + fxminus1yminus1z - fxplus1yminus1z - fxminus1yplus1z);

    Eigen::Matrix3d hessian;
    hessian(0, 0) = fxx;
    hessian(1, 1) = fyy;
    hessian(2, 2) = fzz;
    hessian(0, 1) = hessian(1, 0) = fxy;
    hessian(0, 2) = hessian(2, 0) = fxz;
    hessian(1, 2) = hessian(2, 1) = fyz;

    // Dividing by denominator in real distance causes floating precision errors. Keep unit in voxel size.
    double denominator = (4 * deltaSize * deltaSize); // 4h^2, where h is step size.
    return hessian / denominator;
}

void SDF::testComputeDistanceHessian()
{
    // Since interpolation is used, analytic function f to test should be linear
    double voxelSize = 0.5;
    Eigen::Vector3d min3dLoc(0, 0, 0);
    Eigen::Vector3d max3dLoc(8, 8, 8);
    double unknownClipDistance = 10;
    SDF testSdf(voxelSize, min3dLoc, max3dLoc, UnknownClipDistance);
    // cout << testSdf.m_voxelGridTSDF.size() << endl;
    for (int z = 0; z < testSdf.m_gridSize(2); z++)
    {
        for (int y = 0; y < testSdf.m_gridSize(1); y++)
        {
            for (int x = 0; x < testSdf.m_gridSize(0); x++)
            {
                int index = z * testSdf.m_gridSpacingPerAxis(2) + y * testSdf.m_gridSpacingPerAxis(1) + x;
                // if(x == testSdf.m_gridSize(0)-1 && y == testSdf.m_gridSize(1)-1)
                //     cout << "Hi";
                testSdf.m_voxelGridTSDF[index] = x + y - z;
            }
        }
    }

    // ToDo: Add Bounday conditions
    for (int z = 2; z < testSdf.m_gridSize(2) - 2; z++)
    {
        for (int y = 2; y < testSdf.m_gridSize(1) - 2; y++)
        {
            for (int x = 2; x < testSdf.m_gridSize(0) - 2; x++)
            {
                for (int i = 0; i < 10; i++) // Hessian looks 2 steps away, and thus we will cross the boundary for
                {
                    double delta = i * voxelSize / 10.0f;
                    Eigen::Vector3d gridLocation(x + 0.5 + delta, y + 0.5 - delta, z + 0.5 - delta);
                    Eigen::Matrix3d actualDistanceHessian = testSdf.computeDistanceHessian(gridLocation);
                    // cout << x << ',' << y << ',' << z << "," << i << ": Expected: Zero Matrix, Actually isZero " << actualDistanceHessian << endl;
                    if (!actualDistanceHessian.isZero(1e-4))
                        actualDistanceHessian = testSdf.computeDistanceHessian(gridLocation);
                    assert(actualDistanceHessian.isZero(1e-4) && "Whoops, check SDF::testComputeDistanceGradient");
                }
            }
        }
    }
}

Eigen::Matrix3d SDF::computeDistanceHessian(const Eigen::Vector3i &spatialIndex,
                                            const DisplacementField *displacementField) const
{
    Eigen::Vector3d gridLocation = spatialIndex.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5);
    // ToDo: Optimize
    double fxyz = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, 0, 0), displacementField);
    double fxplus2yz = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(2, 0, 0), displacementField);
    double fxminus2yz = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(-2, 0, 0), displacementField);
    double fxyplus2z = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, 2, 0), displacementField);
    double fxyminus2z = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, -2, 0), displacementField);
    double fxyzplus2 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, 0, 2), displacementField);
    double fxyzminus2 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, 0, -2), displacementField);
    double fxplus1yzplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(1, 0, 1), displacementField);
    double fxminus1yzplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(-1, 0, 1), displacementField);
    double fxyplus1zplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, 1, 1), displacementField);
    double fxyminus1zplus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, -1, 1), displacementField);
    double fxplus1yplus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(1, 1, 0), displacementField);
    double fxminus1yplus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(-1, 1, 0), displacementField);
    double fxminus1yzminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(-1, 0, -1), displacementField);
    double fxplus1yzminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(1, 0, -1), displacementField);
    double fxyminus1zminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, -1, -1), displacementField);
    double fxyplus1zminus1 = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(0, 1, -1), displacementField);
    double fxminus1yminus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(-1, -1, 0), displacementField);
    double fxplus1yminus1z = getDistancef(gridLocation + deltaSize * Eigen::Vector3d(1, -1, 0), displacementField);

    double fxx, fyy, fzz, fxy, fxz, fyz;
    fxx = (fxplus2yz - 2 * fxyz + fxminus2yz);
    fyy = (fxyplus2z - 2 * fxyz + fxyminus2z);
    fzz = (fxyzplus2 - 2 * fxyz + fxyzminus2);
    fxz = (fxplus1yzplus1 + fxminus1yzminus1 - fxplus1yzminus1 - fxminus1yzplus1);
    fyz = (fxyplus1zplus1 + fxyminus1zminus1 - fxyplus1zminus1 - fxyminus1zplus1);
    fxy = (fxplus1yplus1z + fxminus1yminus1z - fxplus1yminus1z - fxminus1yplus1z);

    Eigen::Matrix3d hessian;
    hessian(0, 0) = fxx;
    hessian(1, 1) = fyy;
    hessian(2, 2) = fzz;
    hessian(0, 1) = hessian(1, 0) = fxy;
    hessian(0, 2) = hessian(2, 0) = fxz;
    hessian(1, 2) = hessian(2, 1) = fyz;

    // Dividing by denominator in real distance causes floating precision errors. Keep unit in voxel size.
    double denominator = (4 * deltaSize * deltaSize); // 4h^2, where h is step size.
    return hessian / denominator;
}
