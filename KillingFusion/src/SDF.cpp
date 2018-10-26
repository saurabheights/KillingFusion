//
// Created by Saurabh Khanduja on 26.10.18.
//

#include "SDF.h"
#include <string.h> // for memset

SDF::SDF(int grid_dim_x, int grid_dim_y, int grid_dim_z) :
    m_grid_dim_x(grid_dim_x),
    m_grid_dim_y(grid_dim_y),
    m_grid_dim_z(grid_dim_z) {
  // ToDo - Use smart pointer to avoid deallocation
  float *m_grid_TSDF = new float[m_grid_dim_x * m_grid_dim_y * m_grid_dim_z];
  float *m_grid_weight = new float[m_grid_dim_x * m_grid_dim_y * m_grid_dim_z];

  // Set weight to 1.0
  for (int i = 0; i < m_grid_dim_x * m_grid_dim_y * m_grid_dim_z; ++i)
    m_grid_TSDF[i] = 1.0f;
  memset(m_grid_weight, 0, sizeof(float) * m_grid_dim_x * m_grid_dim_y * m_grid_dim_z);
}

SDF::~SDF() {
  delete[] m_grid_TSDF;
  delete[] m_grid_weight;
}