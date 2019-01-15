//
// Created by Saurabh Khanduja on 18.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H

#include <string>

// ToDo: later add another layer of VariationalFusion between main and KillingFusion class and further
// add the Solobev Fusion.
enum FusionTechnique
{
  KILLING_FUSION
};

const extern FusionTechnique fusionTechnique;

/**
 * Mira Slavcheva Deformable Dataset Parameters
 * See: campar.in.tum.de/personal/slavcheva/deformable-dataset/index.html
 */
enum DEFORMABLE_DATASET
{
  DUCK = 0,
  SNOOPY = 1
};
extern int numImageFiles[2];
extern std::string imageDir[2];
extern std::string intrinsicParamsFile;
extern float datasetDepthMinMaxValues[2][2];

/**
 * Killing Fusion Pipeline configuration
 */

// learning rates for gradient descent
extern float alpha;

// Convergence criterion - Stop when gradient < threshold  
extern float threshold;

// Killing condition weights 
extern float omegaKilling;

// Level-set condition weights
extern float omegaLevelSet;
#endif //INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H
