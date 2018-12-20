//
// Created by Saurabh Khanduja on 25.11.18.
//

#ifndef SHAPECOMPLETIONDATASETCREATION_SCANNETDATALOADER_H
#define SHAPECOMPLETIONDATASETCREATION_SCANNETDATALOADER_H

#include <string>
#include <opencv2/core/mat.hpp>
#include <Eigen/Eigen>

class ScannetDataLoader {
  public:
    ScannetDataLoader(std::string scanDir);

    float getDepthShift();
    const Eigen::Matrix4f getAxisAlignment() const;
    const float *getColorToDepthExtrinsics() const;
    int getColorWidth() const;
    int getColorHeight() const;
    int getDepthWidth() const;
    int getDepthHeight() const;
    double getFxColor() const;
    double getFyColor() const;
    double getFxDepth() const;
    double getFyDepth() const;
    double getMxColor() const;
    double getMyColor() const;
    double getMxDepth() const;
    double getMyDepth() const;
    int getNumColorFrames() const;
    int getNumDepthFrames() const;

    /**
     * Hard Coded Values.
     */
    static float getVoxelSize() { return 0.048; }
    static float getMinimumDepthThreshold() { return 0.4; }
    static float getMaximumDepthThreshold() { return 4.0; }
    static float getTruncationDistanceInVoxelSize() { return 2.0; }
    static float getMinimumSdfWeight() { return 0.0; } // ToDo - Remove this or not?

    cv::Mat getDepthFrame(int index);
    Eigen::Matrix4f getCameraToWorldPose(int index, bool axisAlignment);
    Eigen::Matrix3f getDepthIntrinsicMatrix();

  private:

    void readScanSummaryTextFile(std::string summaryTextFile);
    std::string scanDir;

    // a 4x4 matrix encoding the rigid transform to axis alignment for the scan
    // See http://www.scan-net.org/changelog: under "New axis alignments"
    Eigen::Matrix4f axisAlignment;
    float colorToDepthExtrinsics[16]; // ToDo - Change to Matrix4f
    int colorWidth, colorHeight;
    int depthWidth, depthHeight;
    double fxColor, fyColor;
    double fxDepth, fyDepth;
    double mxColor, myColor;
    double mxDepth, myDepth;

    float depthShift;

    int numColorFrames;
    int numDepthFrames;
    int numImuMeasurements;
    std::string sceneType; // Change to enum
    std::vector<Eigen::Matrix4f> camera_to_world_poses; // Not used yet. Reading from files for now
};

#endif //SHAPECOMPLETIONDATASETCREATION_SCANNETDATALOADER_H
