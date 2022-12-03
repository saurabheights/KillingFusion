# KillingFusion

This is my implementation of [KillingFusion](http://campar.in.tum.de/Chair/PublicationDetail?pub=slavcheva2017cvpr). What works well:-
  - Full GUI to view displacement field, current RGBD frame and SDF's mesh using Marching Cubes.
  - Data Energy Term - [Video](https://youtu.be/DAhcmZfHk_4)
  - Data Energy and Motion Regularizer Term - [Video](https://www.youtube.com/watch?v=3NYu2poyIhk)
  - LevelSet Energy Term

What needs work:-
 - Implement **Rigid Registration**. This is must, since number of iterations becomes very high with slightest displacement of object from its initial position and the data term of a voxel gets no proper gradient if voxel ends up too away from canonical SDF voxels.
 - Balancing of Rigidity constraint energy with Data and Motion Regularizer term. Some notes on this issue are [here](https://github.com/Algomorph/InfiniTAM/issues/20)

### Installation

Fix path to compile FreeImage library in CmakeLists.txt and then run CMake. 

### ToDo
 - Move all code to CUDA.
 - Fix CMakeList - FreeImage library hard coded path
 - Cleanup of the documentation
