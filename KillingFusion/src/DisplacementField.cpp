#include <DisplacementField.h>
#include <iostream>
#include "config.h"
#include "utils.h"
using namespace std;

DisplacementField::DisplacementField(Eigen::Vector3i _gridSize,
                                     double _voxelSize)
    : m_gridSize{_gridSize},
      m_voxelSize(_voxelSize)
{
    // Auto Initialized to Zero vector.
    m_gridDisplacementValue.resize(m_gridSize.prod(), Eigen::Vector3d::Zero());
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
}

DisplacementField::~DisplacementField()
{
}

Eigen::Vector3d DisplacementField::getDisplacementAt(const Eigen::Vector3i &spatialIndex) const
{
    int index = m_gridSpacingPerAxis.dot(spatialIndex);
    if (index < 0 || index >= m_gridSize.prod())
        return Eigen::Vector3d::Zero();
    return m_gridDisplacementValue.at(index);
}

Eigen::Vector3d DisplacementField::getDisplacementAt(int x, int y, int z) const
{
    int index = z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x;
    if (index < 0 || index >= m_gridSize.prod())
        return Eigen::Vector3d::Zero();

    Eigen::Vector3d tmp = m_gridDisplacementValue.at(index);
    return tmp;
}

Eigen::Vector3d DisplacementField::getDisplacementAtf(const Eigen::Vector3d &gridLocation) const
{
    // Interpolate in 3D array - https://stackoverflow.com/questions/19271568/trilinear-interpolation
    Eigen::Vector3i bottomLeftFrontIndex = gridLocation.cast<int>();

    // ToDo - Compute indices yourself to make faster. Done as below to get implementation correct first.
    Eigen::Vector3d vertex_000 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 0));
    Eigen::Vector3d vertex_001 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 0));
    Eigen::Vector3d vertex_010 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 0));
    Eigen::Vector3d vertex_011 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 0));
    Eigen::Vector3d vertex_100 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 0, 1));
    Eigen::Vector3d vertex_101 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 0, 1));
    Eigen::Vector3d vertex_110 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(0, 1, 1));
    Eigen::Vector3d vertex_111 = getDisplacementAt(bottomLeftFrontIndex + Eigen::Vector3i(1, 1, 1));
    Eigen::Vector3d interpolationWeights = gridLocation - bottomLeftFrontIndex.cast<double>();
    return interpolate3DVectors(vertex_000, vertex_001, vertex_010, vertex_011,
                                vertex_100, vertex_101, vertex_110, vertex_111,
                                interpolationWeights(0), interpolationWeights(1), interpolationWeights(2));
}

Eigen::Vector3d DisplacementField::getDisplacementAtf(double x, double y, double z) const
{
    return getDisplacementAtf(Eigen::Vector3d(x, y, z));
}

void DisplacementField::update(const Eigen::Vector3i &spatialIndex,
                               const Eigen::Vector3d &deltaUpdate)
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

void DisplacementField::initializeAllVoxels(Eigen::Vector3d displacement)
{
    for (size_t i = 0; i < m_gridSize.prod(); i++)
    {
        this->m_gridDisplacementValue[i] = displacement;
    }
}

Eigen::Matrix3d DisplacementField::computeJacobian(double x, double y, double z) const
{
    // Future Tasks:- Add boundary checks.
    int deltaLoc[3][3] = {{1, 0, 0},
                          {0, 1, 0},
                          {0, 0, 1}};
    Eigen::Vector3d displacementUVW[3][2];
    Eigen::Matrix3d jacobian;
    // ux, uy, uz
    // vx, vy, vz
    // wx, wy, wz
    for (int i = 0; i < 3; i++) // Compute ux, vx,wx. Next, compute for y and last for z.
    {
        displacementUVW[i][0] = getDisplacementAtf(Eigen::Vector3d(x + deltaLoc[i][0] * deltaSize,
                                                                   y + deltaLoc[i][1] * deltaSize,
                                                                   z + deltaLoc[i][2] * deltaSize));
        displacementUVW[i][1] = getDisplacementAtf(Eigen::Vector3d(x - deltaLoc[i][0] * deltaSize,
                                                                   y - deltaLoc[i][1] * deltaSize,
                                                                   z - deltaLoc[i][2] * deltaSize));
        // For i=0
        // displacementUVW[i][0] - displacementUVW[i][1] gives how DisplacementField changes with 2 * deltaX
        // Thus, (displacementUVW[i][0] - displacementUVW[i][1])/2*deltaX, gives ux, vx, wx.
        jacobian.col(i) = displacementUVW[i][0] - displacementUVW[i][1];
    }

    jacobian /= 2 * deltaSize;
    return jacobian;
}

void DisplacementField::testJacobian()
{
    // Since interpolation is used in numerical differentiation, analytic function f to test should be linear
    double voxelSize = 0.5;
    DisplacementField testField(Eigen::Vector3i(8, 8, 8), voxelSize);
    for (size_t x = 0; x < 5; x++)
    {
        for (size_t y = 0; y < 5; y++)
        {
            for (size_t z = 0; z < 5; z++)
            {
                // F(x,y,z) = (x+y, x+z, y+z)
                double f1 = x + y + z;
                double f2 = x + z;
                double f3 = y + z;
                testField.update(Eigen::Vector3i(x, y, z), Eigen::Vector3d(f1, f2, f3));
            }
        }
    }
    double x = 2, y = 2, z = 2;
    Eigen::Matrix3d numericalJacobian = testField.computeJacobian(x, y, z);
    cout << "numericalJacobian at 2,2,2 is \n"
         << numericalJacobian << endl;
    cout << "analyticalJacobian at 2,2,2 is \n"
         << "1, 1, 1\n"
         << "1, 0, 1\n"
         << "0, 1, 1" << endl;
}

double DisplacementField::computeKillingEnergy(double x, double y, double z) const
{
    // Future Tasks:- Add boundary checks. Not needed as of now.

    // Compute Jacobian Matrix
    Eigen::Matrix3d jacobian = computeJacobian(x, y, z);
    if (!jacobian.array().isFinite().all())
    {
        cout << "Jacobian is infinite\n";
        jacobian = computeJacobian(x, y, z); // To Debug
        return 0.0;
    }

    // Stack Matrix and its transpose columnwise
    Eigen::Matrix3d jacobianTranspose = jacobian.transpose();
    Eigen::VectorXd jacobianVec(Eigen::Map<Eigen::VectorXd>(jacobian.data(), jacobian.cols() * jacobian.rows()));
    Eigen::VectorXd jacobianTransposeVec(Eigen::Map<Eigen::VectorXd>(jacobianTranspose.data(), jacobianTranspose.cols() * jacobianTranspose.rows()));

    // Compute Damped Approximate Killing Vector Field
    double avkf = jacobianVec.dot(jacobianVec) + gammaKilling * jacobianTransposeVec.dot(jacobianVec);
    // return avkf;
    return std::min(avkf, 25.0); // threshold to cutoff too high killing values, causes Nan in grad. Put this in config.cpp
}

void DisplacementField::testKillingEnergy()
{
    // Since interpolation is used in numerical differentiation, analytic function f to test should be linear
    double voxelSize = 0.5;
    DisplacementField testField(Eigen::Vector3i(8, 8, 8), voxelSize);
    for (size_t x = 0; x < 8; x++)
    {
        for (size_t y = 0; y < 8; y++)
        {
            for (size_t z = 0; z < 8; z++)
            {
                // F(x,y,z) = (x+y, x+z, y+z)
                double f1 = x + y + z;
                double f2 = x + z;
                double f3 = y;
                testField.update(Eigen::Vector3i(x, y, z), Eigen::Vector3d(f1, f2, f3));
            }
        }
    }

    for (size_t x = 0; x < 7; x++)
    {
        for (size_t y = 1; y < 7; y++)
        {
            for (size_t z = 1; z < 7; z++)
            {
                double avkf = 0.0;
                Eigen::Matrix3d analyticalJacobian = testField.computeJacobian(x, y, z);
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        avkf += analyticalJacobian(i, j) * analyticalJacobian(i, j) + gammaKilling * analyticalJacobian(i, j) * analyticalJacobian(j, i);
                    }
                }
                if(fabs(avkf - testField.computeKillingEnergy(x, y, z)) > epsilon)
                {
                    cout << analyticalJacobian << endl;
                    float expectedAvkf = testField.computeKillingEnergy(x, y, z);
                }
                // remove truncation of killing energy if test fails.
                assert(fabs(avkf - testField.computeKillingEnergy(x, y, z)) < epsilon && "Whoops! Check DisplacementField::testKillingEnergy");
            }
        }
    }
}

Eigen::Vector3d DisplacementField::computeKillingEnergyGradient(const Eigen::Vector3i &spatialIndex) const
{
    if ((spatialIndex.array() < 2 * deltaSize).any() || (spatialIndex.array() >= (m_gridSize.array() - 2 * deltaSize)).any())
        return Eigen::Vector3d::Zero();

    int x = spatialIndex(0), y = spatialIndex(1), z = spatialIndex(2);
    Eigen::Vector3d killingEnergyGrad;
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

    killingEnergyGrad /= 2 * deltaSize;
    return killingEnergyGrad;
}
