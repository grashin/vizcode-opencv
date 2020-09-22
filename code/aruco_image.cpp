// #include <iostream>
// #include <opencv2/aruco.hpp>
// cv::Mat markerImage;
// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
// cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include < opencv2/aruco.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
// static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
//     FileStorage fs(filename, FileStorage::READ);
//     if(!fs.isOpened())
//         return false;
//     fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
//     fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
//     fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
//     fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
//     fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
//     fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
//     fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
//     fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
//     fs["minDistanceToBorder"] >> params->minDistanceToBorder;
//     fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
//     fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
//     fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
//     fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
//     fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
//     fs["markerBorderBits"] >> params->markerBorderBits;
//     fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
//     fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
//     fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
//     fs["minOtsuStdDev"] >> params->minOtsuStdDev;
//     fs["errorCorrectionRate"] >> params->errorCorrectionRate;
//     return true;
// }


/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int markersX = 4;
    int markersY = 5;
    float markerLength = 0.04;
    float markerSeparation = 0.005;
    bool showRejected = parser.has("r");
    bool refindStrategy = parser.has("rs");
    int camId = 0;


    Mat camMatrix, distCoeffs;
    
    bool readOk = readCameraParameters("/Users/grashin/video_detection/calibration_3.yaml", camMatrix, distCoeffs);
    if(!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }
    

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    // if(parser.has("dp")) {
    //     bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
    //     if(!readOk) {
    //         cerr << "Invalid detector parameters file" << endl;
    //         return 0;
    //     }
    // }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    String video;
    if(True) {
        video = parser.get<String>("/Users/grashin/video_detection/aruco_video_4.mp4");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICT_6X6_250));

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);

    // create board object
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if(ids.size() > 0)
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        if(markersOfBoardDetected > 0)
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

    return 0;
}