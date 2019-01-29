#include <DisplacementField.h>
#include <iostream>
#include "config.h"
#include "utils.h"
using namespace std;

DisplacementField::DisplacementField(Eigen::Vector3i _gridSize,
                                     float _voxelSize)
    : m_gridSize{_gridSize},
      m_voxelSize(_voxelSize)
{
    // Auto Initialized to Zero vector.
    m_gridDisplacementValue.resize(m_gridSize.prod(), Eigen::Vector3f::Zero());
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
}

DisplacementField::~DisplacementField()
{
}

Eigen::Vector3f DisplacementField::getDisplacementAt(const Eigen::Vector3i &spatialIndex) const
{
    int index = m_gridSpacingPerAxis.dot(spatialIndex);
    if (index < 0 || index > m_gridSize.prod())
        return Eigen::Vector3f::Zero();
    return m_gridDisplacementValue.at(index);
}

Eigen::Vector3f DisplacementField::getDisplacementAt(int x, int y, int z) const
{
    int index = z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x;
    if (index < 0 || index > m_gridSize.prod())
        return Eigen::Vector3f::Zero();

    Eigen::Vector3f tmp = m_gridDisplacementValue.at(index);
    return tmp;
}

Eigen::Vector3f DisplacementField::getDisplacementAtf(const Eigen::Vector3f &gridLocation) const
{
    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = gridLocation.cast<int>();

    // ToDo - Compute indices yourself to make faster. Done as below to get implementation correct first.
    Eigen::Vector3f vertex_000 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    Eigen::Vector3f vertex_001 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    Eigen::Vector3f vertex_010 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    Eigen::Vector3f vertex_011 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    Eigen::Vector3f vertex_100 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    Eigen::Vector3f vertex_101 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    Eigen::Vector3f vertex_110 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    Eigen::Vector3f vertex_111 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3f interpolationWeights = gridLocation - bottomLeftFrontIndex.cast<float>();
    return interpolate3DVectors(vertex_000, vertex_001, vertex_010, vertex_011,
                                vertex_100, vertex_101, vertex_110, vertex_111,
                                interpolationWeights(0), interpolationWeights(1), interpolationWeights(2));
}

Eigen::Vector3f DisplacementField::getDisplacementAtf(float x, float y, float z) const
{
    return getDisplacementAtf(Eigen::Vector3f(x, y, z));
}

void DisplacementField::update(const Eigen::Vector3i &spatialIndex,
                               const Eigen::Vector3f &deltaUpdate)
{
    // Future Tasks - Implement boundary checking.
    int index = m_gridSpacingPerAxis.dot(spatialIndex);
    m_gridDisplacementValue.at(index) += deltaUpdate;
}

DisplacementField &DisplacementField::operator+(const DisplacementField &otherDisplacementField)
{
    for (size_t i = 0; i < m_gridSize.prod(); i++)
    {
        this->m_gridDisplacementValue[i] += otherDisplacementField.m_gridDisplacementValue[i];
    }
    return *this;
}

Eigen::Matrix3f DisplacementField::computeJacobian(float x, float y, float z) const
{
    // Future Tasks:- Add boundary checks.
    int deltaLoc[3][3] = {{1, 0, 0},
                          {0, 1, 0},
                          {0, 0, 1}};
    Eigen::Vector3f displacementUVW[3][2];
    Eigen::Matrix3f jacobian;
    // ux, uy, uz
    // vx, vy, vz
    // wx, wy, wz
    for (int i = 0; i < 3; i++) // Compute ux, vx,wx. Next, compute for y and last for z.
    {
        displacementUVW[i][0] = getDisplacementAt(x + deltaLoc[i][0] * deltaSize,
                                                  y + deltaLoc[i][1] * deltaSize,
                                                  z + deltaLoc[i][2] * deltaSize);
        displacementUVW[i][1] = getDisplacementAt(x - deltaLoc[i][0] * deltaSize,
                                                  y - deltaLoc[i][1] * deltaSize,
                                                  z - deltaLoc[i][2] * deltaSize);
        // For i=0
        // displacementUVW[i][0] - displacementUVW[i][1] gives how DisplacementField changes with 2 * deltaX
        // Thus, (displacementUVW[i][0] - displacementUVW[i][1])/2*deltaX, gives ux, vx, wx.
        jacobian.col(i) = displacementUVW[i][0] - displacementUVW[i][1];
    }

    jacobian /= 2 * deltaSize * m_voxelSize;
    return jacobian;
}

float DisplacementField::computeKillingEnergy(float x, float y, float z) const
{
    // Future Tasks:- Add boundary checks. Not needed as of now.

    // Compute Jacobian Matrix
    Eigen::Matrix3f jacobian = computeJacobian(x, y, z);
    if (!jacobian.array().isFinite().all())
        cout << "Jacobian is infinite\n";

    // Stack Matrix and its transpose columnwise
    Eigen::VectorXf jacobianVec(Eigen::Map<Eigen::VectorXf>(jacobian.data(), jacobian.cols() * jacobian.rows()));
    jacobian.transposeInPlace();
    Eigen::VectorXf jacobianTransposeVec(Eigen::Map<Eigen::VectorXf>(jacobian.data(), jacobian.cols() * jacobian.rows()));

    // Compute Damped Approximate Killing Vector Field
    float avkf = jacobianVec.dot(jacobianVec) + gammaKilling * jacobianTransposeVec.dot(jacobianVec);
    return std::min(avkf, 2.0f * m_voxelSize);
}

Eigen::Vector3f DisplacementField::computeKillingEnergyGradient(const Eigen::Vector3i &spatialIndex) const
{
    // ToDO: Correct boundary check.
    if ((spatialIndex.array() < 2 * deltaSize).any() || (spatialIndex.array() >= (m_gridSize.array() - 2 * deltaSize)).any())
        return Eigen::Vector3f::Zero();

    int x = spatialIndex(0), y = spatialIndex(1), z = spatialIndex(2);
    Eigen::Vector3f killingEnergyGrad;
    int deltaLoc[3][3] = {{1, 0, 0},
                          {0, 1, 0},
                          {0, 0, 1}};
    for (int i = 0; i < 3; i++) // x or y or z
    {
        killingEnergyGrad(i) = computeKillingEnergy(x + deltaLoc[i][0] * deltaSize,
                                                    y + deltaLoc[i][1] * deltaSize,
                                                    z + deltaLoc[i][2] * deltaSize) -
                               computeKillingEnergy(x - deltaLoc[i][0] * deltaSize,
                                                    y - deltaLoc[i][1] * deltaSize,
                                                    z - deltaLoc[i][2] * deltaSize);
    }

    killingEnergyGrad /= 2 * deltaSize * m_voxelSize;
    return killingEnergyGrad;
}
