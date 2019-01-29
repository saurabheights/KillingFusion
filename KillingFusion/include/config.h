//
// Created by Saurabh Khanduja on 18.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H

#include <string>

// SDF Generation Parameters
const extern float VoxelSize;
const extern float UnknownClipDistance;
const extern float MaxSurfaceVoxelDistance;
const extern bool FUSE_BY_MERGE;
// Dataset and Pipeline to Use

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

const extern DEFORMABLE_DATASET datasetType;
const extern int numImageFiles[2];
const extern std::string imageDir[2];
const extern std::string intrinsicParamsFile;
const extern float datasetDepthMinMaxValues[2][2];
const extern std::string outputDir[2];

/**
 * Killing Fusion Pipeline configuration
 */

// The below four params change how optimization is done over each voxel for next frame.
const extern bool EnergyTypeUsed[3]; // To enable or disable - Data, LevelSet, Killing
const extern bool UseZeroDisplacementFieldForNextFrame; // Use Zero Displacement Field.
const extern bool UpdateAllVoxelsInEachIter; // Update is performed on all voxels for each iterations. If false, all iteration updates are performed on one voxel and then on next. Ideadlly, One should make one update on all voxels, and then perform next iter, thus keey this true. But runs very fast if false. :)
const extern bool UsePreviousIterationDeformationField; // If true, previous iteration displacement field is used for computing LevelSet Energy and KillingEnergy. 

const extern int KILLING_MAX_ITERATIONS;

const extern float deltaSize; // Step Size in Voxel unit for central difference.

// learning rates for gradient descent
const extern float alpha;

// Convergence criterion - Stop when gradient < threshold - Used only when UpdateAllVoxelsInEachIter is false.
const extern float threshold;

// Killing condition weights
const extern float omegaKilling;

// Killing - Rigidity factor
const extern float gammaKilling;

// Level-set condition weights
const extern float omegaLevelSet;

// Level-set - Prevents division by zero
const extern float epsilon;

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H
