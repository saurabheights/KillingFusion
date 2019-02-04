#include <config.h>

// SDF Generation Parameters
const double VoxelSize = 0.012;
const double UnknownClipDistance = VoxelSize * 4;
const double MaxSurfaceVoxelDistance = VoxelSize * 4;
const bool FUSE_BY_MERGE = true;

// Dataset and Pipeline to Use
const FusionTechnique fusionTechnique = KILLING_FUSION;
const DEFORMABLE_DATASET datasetType = SNOOPY;

const int numImageFiles[2] = {447, 630};
const std::string imageDir[2] = {std::string("Duck/"), std::string("Snoopy/")};
const std::string intrinsicParamsFile = "intrinsics_kinect1.txt";
const double datasetDepthMinMaxValues[2][2] = {
    {0.0497470, 1.2}, // 3.66115f
    {0.0495164, 1.2},     // 3.40335f - True value changed to remove pixels of wall in background. This reduces grid size substantially and makes KF fast.
};

const std::string outputDir[2] = {"Duck/", "Snoopy/"};

const bool EnergyTypeUsed[3] = {true, true, true}; // Data, LevelSet, Killing
const bool UseZeroDisplacementFieldForNextFrame = true;
const bool UpdateAllVoxelsInEachIter = false;
const bool UsePreviousIterationDeformationField = true; // If true, previous iteration displacement field is used for computing LevelSet Energy and KillingEnergy. 
const bool UseTrustStrategy = false; // Only used when working only with data energy. Helps in finding which alpha to use for voxel data energy gradient.
// Do not reduce, causes floating point precision errors in SDF::computeDistanceHessian
const double deltaSize = 0.05; // Step Size in Voxel unit for central difference.


/**
 * Killing Fusion Pipeline configuration
 * The parameters were fixed as follows: gradient descent step α = 0.1, 
 * damping factor for the Killing energy γ = 0.1, 
 * weights for the motion and level set regularization respectively ωk = 0.5, ωs = 0.2.
 * The choice of values for ωs and ωk not only balances their influence, but also acts as 
 * normalization since signed distances are truncated to the interval [−1; 1], while the 
 * deformation field contains vectors spanning up to several voxels. We used a voxel size 
 * of 8 mm for human-sized subjects and 4 mm for smaller-scale ones.
 */
const int KILLING_MAX_ITERATIONS = 1024;
const double threshold = 0.000001;

const double alpha = 0.025;

// Killing weights
const double omegaKilling = 0.04;

// Killing - Purity
const double gammaKilling = 0;

// Level-set condition weights
const double omegaLevelSet = 0.1;

// Level-set - Prevents division by zero
const double epsilon = 0.00001;
