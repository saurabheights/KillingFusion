#include <DisplacementField.h>
#include <iostream>
using namespace std;

DisplacementField::DisplacementField(Eigen::Vector3i _gridSize,
                                     float _voxelSize,
                                     Eigen::Vector3f _min3dLoc,
                                     Eigen::Vector3f _max3dLoc)
    : m_gridSize{_gridSize},
      m_voxelSize(_voxelSize),
      m_min3dLoc(_min3dLoc),
      m_max3dLoc(_max3dLoc)
{
    // Auto Initialized to Zero vector.
    m_gridDisplacementValue.resize(m_gridSize.prod());
    m_gridSpacingPerAxis = Eigen::Vector3i(1, m_gridSize(0), m_gridSize(0) * m_gridSize(1));
}

DisplacementField::~DisplacementField()
{
}

Eigen::Vector3f DisplacementField::getDisplacementAt(const Eigen::Vector3i &spatialIndex) const
{
    // ToDo - Implement boundary checking.
    return m_gridDisplacementValue.at(m_gridSpacingPerAxis.dot(spatialIndex));
}

Eigen::Vector3f DisplacementField::getDisplacementAt(int x, int y, int z) const
{
    // ToDo - Implement boundary checking.
    return m_gridDisplacementValue.at(z * m_gridSpacingPerAxis(2) + y * m_gridSpacingPerAxis(1) + x);
}

void DisplacementField::update(Eigen::Vector3i spatialIndex, Eigen::Vector3f deltaUpdate)
{
    // ToDo - Implement boundary checking.
    int index = m_gridSpacingPerAxis.dot(spatialIndex);
    m_gridDisplacementValue.at(index) += deltaUpdate;
}