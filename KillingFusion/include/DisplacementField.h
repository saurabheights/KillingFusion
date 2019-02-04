#if !defined(DISPLACEMENT_FIELD_H)
#define DISPLACEMENT_FIELD_H

#include <vector>
#include <Eigen/Eigen>

// This field stores 3D Vector location, any given voxel should be moved to.
// Thus distance value at (x,y,z) should be given by SDF->grid([(x,y,z) + deformation(x,y,z)])
class DisplacementField
{
private:
  std::vector<Eigen::Vector3d> m_gridDisplacementValue;
  double m_voxelSize;
  Eigen::Vector3i m_gridSize;
  Eigen::Vector3i m_gridSpacingPerAxis; // See update function to see how this is used.
  Eigen::Vector3d m_bound;

public:
  DisplacementField() = delete;
  DisplacementField(Eigen::Vector3i gridSize,
                    double _voxelSize);
  ~DisplacementField();

  Eigen::Vector3d getDisplacementAt(const Eigen::Vector3i &spatialIndex) const;

  Eigen::Vector3d getDisplacementAt(int x, int y, int z) const;

  Eigen::Vector3d getDisplacementAtf(const Eigen::Vector3d &gridLocation) const;

  Eigen::Vector3d getDisplacementAtf(double x, double y, double z) const;

  /**
   * Update(adds) the displacement value at location spatialIndex by deltaUpdate.
   */
  void update(const Eigen::Vector3i &spatialIndex, const Eigen::Vector3d &deltaUpdate);

  DisplacementField &operator+(const DisplacementField &otherDisplacementField);

  void initializeAllVoxels(Eigen::Vector3d displacement);

  /**
   * Computes Jacobian of Displacement Field(3d Vector Field) with respect to x,y,z.
   **/
  Eigen::Matrix3d computeJacobian(double x, double y, double z) const;

  static void testJacobian();

  /**
   * Computes Killing energy at any given point of the displacement field.
   */
  double computeKillingEnergy(double x, double y, double z) const;

  static void testKillingEnergy();

  /**
   * Computes Killing energy gradient at any given point of the displacement field.
   */
  Eigen::Vector3d computeKillingEnergyGradient(const Eigen::Vector3i &spatialIndex) const;
  Eigen::Vector3d computeKillingEnergyGradient2(const Eigen::Vector3i &spatialIndex) const;

  void dumpToBinFile(std::string outputFilePath) const;

  Eigen::Vector3i getGridSize() const {
    return m_gridSize;
  }
};

#endif // DISPLACEMENT_FIELD_H
