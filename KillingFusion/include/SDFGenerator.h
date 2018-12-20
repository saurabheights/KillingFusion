//
// Created by Saurabh Khanduja on 25.11.18.
//

#ifndef SHAPECOMPLETIONDATASETCREATION_SDFGENERATOR_H
#define SHAPECOMPLETIONDATASETCREATION_SDFGENERATOR_H

#include <string>
#include <vector>
#include <memory>
#include "SDF.h"
#include "ScannetDataLoader.h"

class SDFGenerator {
  public:
    SDFGenerator(std::string sensFolderPath);
    std::shared_ptr<SDF> createSDF();
  private:
    bool alignAxis;
    std::string rootDir;
    std::unique_ptr<ScannetDataLoader> scannetDataLoader;
    std::pair<Eigen::Vector3f, Eigen::Vector3f> computeBounds();
};

#endif //SHAPECOMPLETIONDATASETCREATION_SDFGENERATOR_H
