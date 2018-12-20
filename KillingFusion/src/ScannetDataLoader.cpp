//
// Loads data from directory of a scan. with extracted sens data in sens_data folder
// Created by Saurabh Khanduja on 25.11.18.
//

#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ScannetDataLoader.h>

using namespace std;

ScannetDataLoader::ScannetDataLoader(string scanDir) {
    this->scanDir = scanDir;
    this->depthShift = -1; // Read always from getDepthShift()
    string basename = scanDir.substr(scanDir.find_last_of('/') + 1);
    string summaryTextFile = scanDir + '/' + basename + ".txt";
    readScanSummaryTextFile(summaryTextFile);
}

void ScannetDataLoader::readScanSummaryTextFile(string summaryTextFile) {
    // Read summary text file
    ifstream infile;
    infile.open(summaryTextFile.c_str());
    string line, tmp;

    if (!infile.is_open()) {
        printf("Text file %s\n failed to open.", summaryTextFile.c_str());
        exit(-1);
    }

    // Line 1
    // axisAlignment = 0.945519 0.325568 0.000000 -5.384390 -0.325568 0.945519 0.000000 -2.871780 0.000000 0.000000 1.000000 -0.064350 0.000000 0.000000 0.000000 1.000000
    getline(infile, line);
    istringstream ss(line);
    ss >> tmp >> tmp;
    float axisAlignmentArr[16];
    for (int i = 0; i < 16; ++i) {
        ss >> axisAlignmentArr[i];
        axisAlignment(i/4, i%4) =  axisAlignmentArr[i];
    }
    
    // colorHeight = 968
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> colorHeight;

    //colorToDepthExtrinsics = 0.999973 0.006791 0.002776 -0.037886 -0.006767 0.999942 -0.008366 -0.003410 -0.002833 0.008347 0.999961 -0.021924 -0.000000 0.000000 -0.000000 1.000000
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    for (int i = 0; i < 16; ++i) {
        ss >> colorToDepthExtrinsics[i];
    }

    //colorWidth = 1296
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> colorWidth;

    //depthHeight = 480
    //depthWidth = 640
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> depthHeight;

    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> depthWidth;

    //fx_color = 1170.187988
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> fxColor;

    //fx_depth = 571.623718
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> fxDepth;

    //fy_color = 1170.187988
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> fyColor;

    //fy_depth = 571.623718
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> fyDepth;

    //mx_color = 647.750000
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> mxColor;

    //mx_depth = 319.500000
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> mxDepth;

    //my_color = 483.750000
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> myColor;

    //my_depth = 239.500000
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> myDepth;

    //numColorFrames = 5578
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> numColorFrames;

    //numDepthFrames = 5578
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> numDepthFrames;

    //numIMUmeasurements = 11834
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> numImuMeasurements;

    //sceneType = Apartment
    getline(infile, line);
    ss.str(line);
    ss.clear();
    ss >> tmp >> tmp;
    ss >> sceneType;

    infile.close();
}

float ScannetDataLoader::getDepthShift() {
    if (depthShift < 0) {
        stringstream depthShiftPathStream;
        depthShiftPathStream << scanDir << "/sens_data/depthShift.txt";
        string depthShiftFile = depthShiftPathStream.str();

        printf("Reading depth value from %s\n", depthShiftFile.c_str());
        ifstream infile;
        infile.open(depthShiftFile.c_str());
        string line, tmp;

        if (!infile.is_open()) {
            printf("Text file %s\n failed to open.", depthShiftFile.c_str());
            exit(-1);
        }

        getline(infile, line);
        istringstream ss(line);
        ss >> depthShift;
        infile.close();
    }
    return depthShift;
}

const Eigen::Matrix4f ScannetDataLoader::getAxisAlignment() const {
    return axisAlignment;
}

const float *ScannetDataLoader::getColorToDepthExtrinsics() const {
    return colorToDepthExtrinsics;
}

int ScannetDataLoader::getColorWidth() const {
    return colorWidth;
}

int ScannetDataLoader::getColorHeight() const {
    return colorHeight;
}

int ScannetDataLoader::getDepthWidth() const {
    return depthWidth;
}

int ScannetDataLoader::getDepthHeight() const {
    return depthHeight;
}

double ScannetDataLoader::getFxColor() const {
    return fxColor;
}

double ScannetDataLoader::getFyColor() const {
    return fyColor;
}

double ScannetDataLoader::getFxDepth() const {
    return fxDepth;
}

double ScannetDataLoader::getFyDepth() const {
    return fyDepth;
}

double ScannetDataLoader::getMxColor() const {
    return mxColor;
}

double ScannetDataLoader::getMyColor() const {
    return myColor;
}

double ScannetDataLoader::getMxDepth() const {
    return mxDepth;
}

double ScannetDataLoader::getMyDepth() const {
    return myDepth;
}

int ScannetDataLoader::getNumColorFrames() const {
    return numColorFrames;
}

int ScannetDataLoader::getNumDepthFrames() const {
    return numDepthFrames;
}

cv::Mat ScannetDataLoader::getDepthFrame(int index) {
    stringstream depthPath;
    depthPath << scanDir << "/sens_data/depth/" << index << ".png";
    cv::Mat depthImage = cv::imread(depthPath.str(), cv::IMREAD_UNCHANGED); // Because pngs are 16-bit.
    if (depthImage.empty()) {
        cout << "Error: depth image file not read!" << endl;
        cv::waitKey(0);
        exit(-1);
    }

    cv::Mat depthFloatImage(depthImage.size(), CV_32FC1);
    depthImage.convertTo(depthFloatImage, CV_32FC1, 1.0 / getDepthShift());
    double minVal, maxVal;
    cv::minMaxLoc(depthFloatImage, &minVal, &maxVal);
    cout << "Depth Frame " << index << " has minimum and maximum value as : " << minVal << ", " << maxVal << endl;
    return depthFloatImage;
}

// Concatenate all poses to a single txt file and then read from it in one go.
Eigen::Matrix4f ScannetDataLoader::getCameraToWorldPose(int index, bool alignAxis) {
    stringstream depthPathStream;
    depthPathStream << scanDir << "/sens_data/pose/" << index << ".txt";
    string depthPath = depthPathStream.str();
    ifstream infile;
    infile.open(depthPath.c_str());
    if (!infile.is_open()) {
        printf("Text file %s\n failed to open.", depthPath.c_str());
        exit(-1);
    }

    string line;
    Eigen::Matrix4f pose;
    for (int i = 0; i < 4; ++i) {
        getline(infile, line);
        istringstream ss(line);
        ss >> pose(i, 0) >> pose(i, 1) >> pose(i, 2) >> pose(i, 3);
    }
    
    if (alignAxis) {
        // AxisAlignment will move the room center to origin and rotates till +x-axis and +y-axis are perpendicular to the walls.
        pose =  axisAlignment * pose;
    }

    return pose;
}

Eigen::Matrix3f ScannetDataLoader::getDepthIntrinsicMatrix() {
    Eigen::Matrix3f depthIntrinsic;
    depthIntrinsic << fxDepth, 0, mxDepth,
        0, fyDepth, myDepth,
        0, 0, 1;
    return depthIntrinsic;
}