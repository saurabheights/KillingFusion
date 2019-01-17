#if !defined(DISPLACEMENT_FIELD_H)
#define DISPLACEMENT_FIELD_H

#include <vector>
#include <Eigen/Eigen>

// This field stores 3D Vector location, any given voxel should be moved to.
// Thus distance value at (x,y,z) should be given by SDF->grid([(x,y,z) + deformation(x,y,z)])
class DisplacementField
{
private:
  std::vector<Eigen::Vector3f> m_gridDisplacementValue;
  float m_voxelSize;
  Eigen::Vector3i m_gridSize;
  Eigen::Vector3i m_gridSpacingPerAxis; // See update function to see how this is used.
  Eigen::Vector3f m_bound;

public:
  DisplacementField() = delete;
  DisplacementField(Eigen::Vector3i gridSize,
                    float _voxelSize);
  ~DisplacementField();

  Eigen::Vector3f getDisplacementAt(const Eigen::Vector3i &spatialIndex) const;
  Eigen::Vector3f getDisplacementAt(int x, int y, int z) const;

  void update(Eigen::Vector3i spatialIndex, const Eigen::Vector3f deltaUpdate);
};

#endif // DISPLACEMENT_FIELD_H
