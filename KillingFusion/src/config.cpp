#include <config.h>

// SDF Generation Parameters
float VoxelSize = 0.01f;
float UnknownClipDistance = VoxelSize * 2;
float MaxSurfaceVoxelDistance = VoxelSize * 4;
bool FUSE_BY_MERGE=true;

// Dataset and Pipeline to Use
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
 * The parameters were fixed as follows: gradient descent step α = 0.1, 
 * damping factor for the Killing energy γ = 0.1, 
 * weights for the motion and level set regularization respectively ωk = 0.5, ωs = 0.2.
 * The choice of values for ωs and ωk not only balances their influence, but also acts as 
 * normalization since signed distances are truncated to the interval [−1; 1], while the 
 * deformation field contains vectors spanning up to several voxels. We used a voxel size 
 * of 8 mm for human-sized subjects and 4 mm for smaller-scale ones.

 */
int KILLING_MAX_ITERATIONS = 32;
float threshold = 0.1f;

float alpha = 0.1f;
float omegaKilling = 0.5f;
float omegaLevelSet = 0.2f;

// Level-set - Prevents division by zero
float epsilon = 0.00001f;
