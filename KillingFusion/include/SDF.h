//
// Created by Saurabh Khanduja on 26.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H

class SDF {
  float * m_grid_TSDF;
  float * m_grid_weight;
  int m_grid_dim_x, m_grid_dim_y, m_grid_dim_z;
 public:
  SDF(int grid_dim_x, int grid_dim_y, int grid_dim_z);
  ~SDF();
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_SDF_H
