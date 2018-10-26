//
// Created by Saurabh Khanduja on 18.10.18.
//

#include "DatasetReader.h"

DatasetReader::DatasetReader(std::string imageDir,
                             std::string depthFilePattern,
                             std::string colorFilePattern,
                             std::string omaskFilePattern,
                             std::string intrinsicParamsFile,
                             int numImageFiles) : m_imageDir(imageDir),
                                                  m_depthFilePattern(depthFilePattern),
                                                  m_colorFilePattern(colorFilePattern),
                                                  m_omaskFilePattern(omaskFilePattern),
                                                  m_numImageFiles(numImageFiles) {
  m_intrinsicParams = LoadMatrixFromFile(m_intrinsicParamsFile, 3 * 3);
}

/**
 * This method reads the value from intrinsic file into a vector.
 * ToDo: Check C++14 standard which allows to return by reference instead of value.
 *
 * @param filename The file to be read
 * @param M The number of elements in matrix
 * @return A vector of all the values read as float.
 */
std::vector<float> DatasetReader::LoadMatrixFromFile(std::string filename, int M) {
  std::vector<float> matrix;
  FILE *fp = fopen(filename.c_str(), "r");
  for (int i = 0; i < M; i++) {
    float tmp;
    int iret = fscanf(fp, "%f", &tmp);
    matrix.push_back(tmp);
  }
  fclose(fp);
  return matrix;
}
