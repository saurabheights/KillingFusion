//
// Created by Saurabh Khanduja on 18.10.18.
//
#include <string>

#include "KillingFusion.h"
#include "DatasetReader.h"
#include "config.h"

int main(int argc, char *argv[]) {
  std::string dataDir = DATA_DIR;
  std::string imageDir = dataDir + "Snoopy_Loop/";
  std::string depthFilePattern = "depth_%6d.png";
  std::string colorFilePattern = "color_%6d.png";
  std::string omaskFilePattern = "omask_%6d.png";
  int numImageFiles = 630;
  std::string intrinsicParamsFile = dataDir + "intrinsics_kinect1.txt";

  DatasetReader datasetReader(imageDir, depthFilePattern, colorFilePattern,
                              omaskFilePattern, intrinsicParamsFile, numImageFiles);
  KillingFusion fusion(datasetReader);
  fusion.process();

}

