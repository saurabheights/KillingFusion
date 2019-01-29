#include <config.h>

// SDF Generation Parameters
const float VoxelSize = 0.02f;
const float UnknownClipDistance = VoxelSize * 4;
const float MaxSurfaceVoxelDistance = VoxelSize * 3;
const bool FUSE_BY_MERGE = true;

// Dataset and Pipeline to Use
const FusionTechnique fusionTechnique = KILLING_FUSION;
const DEFORMABLE_DATASET datasetType = SNOOPY;

const int numImageFiles[2] = {447, 630};
const std::string imageDir[2] = {std::string("Duck/"), std::string("Snoopy/")};
const std::string intrinsicParamsFile = "intrinsics_kinect1.txt";
const float datasetDepthMinMaxValues[2][2] = {
    {0.0497470f, 3.66115f}, // 3.66115f
    {0.0495164f, 1.2f},     // 3.40335f - True value changed, since to remove pixels of wall in background. This reduces grid size substantially and makes KF fast.
};

const std::string outputDir[2] = {"Duck/", "Snoopy/"};

const bool EnergyTypeUsed[3] = {true, false, true}; // Data, LevelSet, Killing
const bool UseZeroDisplacementFieldForNextFrame = false;
const bool UpdateAllVoxelsInEachIter = false;
const bool UsePreviousIterationDeformationField = false; // If true, previous iteration displacement field is used for computing LevelSet Energy and KillingEnergy. 

const float deltaSize = 0.1; // Step Size in Voxel unit for central difference.


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
const int KILLING_MAX_ITERATIONS = 128;
const float threshold = 0.001f;

const float alpha = 0.1f;

// Killing weights
const float omegaKilling = 0.5f;

// Killing - Purity
const float gammaKilling = 0;

// Level-set condition weights
const float omegaLevelSet = 0.05f;

// Level-set - Prevents division by zero
const float epsilon = 0.00001f;
