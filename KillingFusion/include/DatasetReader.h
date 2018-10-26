//
// Created by Saurabh Khanduja on 18.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_DATASETREADER_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_DATASETREADER_H

#include <string>
#include <vector>

class DatasetReader {
  std::string m_imageDir;
  std::string m_depthFilePattern;
  std::string m_colorFilePattern;
  std::string m_omaskFilePattern;
  std::vector<float>  m_intrinsicParams;
  int m_numImageFiles;

  std::vector<float> LoadMatrixFromFile(std::string filename, int M);

 public:

  DatasetReader() = delete;

  DatasetReader(std::string imageDir,
                std::string depthFilePattern,
                std::string colorFilePattern,
                std::string omaskFilePattern,
                std::string intrinsicParamsFile,
                int numImageFiles);
};

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_DATASETREADER_H
