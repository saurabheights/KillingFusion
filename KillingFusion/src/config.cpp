#include <config.h>

const FusionTechnique fusionTechnique = KILLING_FUSION;

int numImageFiles[2] = {447, 630};
std::string imageDir[2] = {std::string("Duck/"), std::string("Snoopy/")};
std::string intrinsicParamsFile = "intrinsics_kinect1.txt";
float datasetDepthMinMaxValues[2][2] = {
    {0.0497470f, 3.66115f},
    {0.0495164f, 3.40335f}
};

/**
 * Killing Fusion Pipeline configuration
 */
float omegaKilling = 0.5f;
float omegaLevelSet = 0.2f;
