#include <DisplacementField.h>
#include <iostream>
#include "config.h"
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
    // Future Tasks - Implement boundary checking.
    return m_gridDisplacementValue.at(m_gridSpacingPerAxis.dot(spatialIndex));
}

Eigen::Vector3f DisplacementField::getDisplacementAt(int x, int y, int z) const
{
    // Future Tasks - Implement boundary checking.
    return m_gridDisplacementValue.at(z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x);
}

void DisplacementField::update(const Eigen::Vector3i &spatialIndex,
                               const Eigen::Vector3f &deltaUpdate)
{
    // Future Tasks - Implement boundary checking.
    int index = m_gridSpacingPerAxis.dot(spatialIndex);
    m_gridDisplacementValue.at(index) += deltaUpdate;
}

Eigen::Matrix3f DisplacementField::computeJacobian(int x, int y, int z) const 
{
    // Future Tasks:- Add boundary checks.
    int deltaLoc[3][2][3] = {{{1, 0, 0}, {-1, 0, 0}},
                             {{0, 1, 0}, {0, -1, 0}},
                             {{0, 0, 1}, {0, 0, -1}}};
    Eigen::Vector3f displacementUVW[3][2];
    for (int i = 0; i < 3; i++) // Compute ux, vx,wx. Next, compute for y and last for z.
    {
        displacementUVW[i][0] = getDisplacementAt(x + deltaLoc[i][0][0], y + deltaLoc[i][0][1], z + deltaLoc[i][0][2]);
        displacementUVW[i][1] = getDisplacementAt(x + deltaLoc[i][1][0], y + deltaLoc[i][1][1], z + deltaLoc[i][1][2]);
    }

    Eigen::Matrix3f jacobian;
    // ux, uy, uz
    // vx, vy, vz
    // wx, wy, wz
    for(int i = 0; i < 3; i++) // First diff wrt x, then y, and last z
    {
            // For i=0
            // displacementUVW[i][0] - displacementUVW[i][1] gives how DisplacementField changes with 2 * deltaX
            // Thus, (displacementUVW[i][0] - displacementUVW[i][1])/2*deltaX, gives ux, vx, wx.
            jacobian.col(i) = displacementUVW[i][0] - displacementUVW[i][1];
    }

    jacobian /= 2 * m_voxelSize;
    return jacobian;
}

float DisplacementField::computeKillingEnergy(int x, int y, int z) const
{
    // Future Tasks:- Add boundary checks.
    // Compute Jacobian Matrix
    Eigen::Matrix3f jacobian = computeJacobian(x,y,z);

    // Stack Matrix and its transpose columnwise
    Eigen::VectorXf jacobianVec(Eigen::Map<Eigen::VectorXf>(jacobian.data(), jacobian.cols()*jacobian.rows()));
    jacobian.transposeInPlace();
    Eigen::VectorXf jacobianTransposeVec(Eigen::Map<Eigen::VectorXf>(jacobian.data(), jacobian.cols()*jacobian.rows()));

    // Compute Damped Approximate Killing Vector Field
    float avkf = jacobianVec.dot(jacobianVec) + gammaKilling *  jacobianTransposeVec.dot(jacobianVec);

    return avkf;
}

Eigen::Vector3f DisplacementField::computeKillingEnergyGradient(const Eigen::Vector3i &spatialIndex) const
{
    if((spatialIndex.array() <= 1).any() || (spatialIndex.array() >= (m_gridSize.array()-2)).any())
        return Eigen::Vector3f::Zero();

    int x = spatialIndex(0), y = spatialIndex(1), z = spatialIndex(2);
    Eigen::Vector3f killingEnergyGrad;
    int deltaLoc[3][2][3] = {{{1, 0, 0}, {-1, 0, 0}},
                             {{0, 1, 0}, {0, -1, 0}},
                             {{0, 0, 1}, {0, 0, -1}}};
    for (int i = 0; i < 3; i++) // x or y or z
    {
        killingEnergyGrad(i) = computeKillingEnergy(x + deltaLoc[i][0][0], y + deltaLoc[i][0][1], z + deltaLoc[i][0][2]) -
                               computeKillingEnergy(x + deltaLoc[i][1][0], y + deltaLoc[i][1][1], z + deltaLoc[i][1][2]);
    }

    killingEnergyGrad /= 2 * m_voxelSize;
    return killingEnergyGrad;
}
