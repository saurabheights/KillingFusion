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

  /**
   * Update(adds) the displacement value at location spatialIndex by deltaUpdate.
   */
  void update(const Eigen::Vector3i& spatialIndex, const Eigen::Vector3f &deltaUpdate);

  /**
   * Computes Jacobian of Displacement Field(3d Vector Field) with respect to x,y,z.
   **/
  Eigen::Matrix3f computeJacobian(int x, int y, int z) const;

  /**
   * Computes Killing energy at any given point of the displacement field.
   */
  float computeKillingEnergy(int x, int y, int z) const;

  /**
   * Computes Killing energy gradient at any given point of the displacement field.
   */
  Eigen::Vector3f computeKillingEnergyGradient(const Eigen::Vector3i &spatialIndex) const;
};

#endif // DISPLACEMENT_FIELD_H
