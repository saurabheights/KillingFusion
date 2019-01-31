//
// Created by Saurabh Khanduja on 18.10.18.
//

#ifndef INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H
#define INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H

#include <string>

// SDF Generation Parameters
const extern double VoxelSize;
const extern double UnknownClipDistance;
const extern double MaxSurfaceVoxelDistance;
const extern bool FUSE_BY_MERGE; // Always set to true. False is not required.
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
const extern double datasetDepthMinMaxValues[2][2];
const extern std::string outputDir[2];

/**
 * Killing Fusion Pipeline configuration
 */

// The below params change how optimization is done over each voxel for next frame.
const extern bool EnergyTypeUsed[3]; // To enable or disable - Data, LevelSet, Killing
const extern bool UseZeroDisplacementFieldForNextFrame; // Use Zero Displacement Field for next frame. Only useful when using Data Energy.
const extern bool UpdateAllVoxelsInEachIter; // Update is performed on all voxels for each iterations. If false, all iteration updates are performed on one voxel and then on next. Ideadlly, One should make one update on all voxels, and then perform next iter, thus keey this true. But runs very fast if false. :)
const extern bool UsePreviousIterationDeformationField; // If true, previous iteration displacement field is used for computing LevelSet Energy and KillingEnergy. 
const extern bool UseTrustStrategy; // Only used when working only with data energy. Helps in finding which alpha to use for voxel data energy gradient.

const extern int KILLING_MAX_ITERATIONS;

const extern double deltaSize; // Step Size in Voxel unit for central difference.

// learning rates for gradient descent
const extern double alpha;

// Convergence criterion - Stop when gradient < threshold - Used only when UpdateAllVoxelsInEachIter is false.
const extern double threshold;

// Killing condition weights
const extern double omegaKilling;

// Killing - Rigidity factor
const extern double gammaKilling;

// Level-set condition weights
const extern double omegaLevelSet;

// Level-set - Prevents division by zero
const extern double epsilon;

#endif //INC_3DSCANNINGANDMOTIONCAPTURE_CONFIG_H
