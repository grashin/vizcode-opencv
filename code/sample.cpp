#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {


    int markersX = 4;
    int markersY = 5;
    float markerLength = 0.04;
    float markerSeparation = 0.005;



    Mat camMatrix, distCoeffs;
//    String filename = "/Users/grashin/video_detection/calibration_3.yaml";
//    FileStorage fs(filename, FileStorage::READ);
//    fs["camera_matrix"] >> camMatrix;
//    fs["dist_coeff"] >> distCoeffs;
//    cout<<(camMatrix);
//    fs.release();
    
    camMatrix = (Mat_<double>(3, 3) << 3585.1832013704093, 0.0, 1578.2004147693144, 0.0, 3523.75914878101, 938.3293602038799, 0.0, 0.0, 1.0);
    distCoeffs = (Mat_<double>(1, 5) << 0.48703167372875666, 0.44308067917207833, 0.01028402178186137, 0.00673989396472976, 0.3018965883819692);
    
    
    
    
//        if(!readOk) {
//            cerr << "Invalid camera file" << endl;
//            return 0;
//        }
//    camMatrix = [3585.1832013704093, 0.0, 1578.2004147693144; 0.0, 3523.75914878101, 938.3293602038799; 0.0, 0.0, 1.0];
//    double data_1[9] = { 3585.1832013704093, 0.0,1578.2004147693144, 0.0, 3523.75914878101, 938.3293602038799, 0.0, 0.0, 1.0  }
////    camMatrix = vector(- - 3585.1832013704093
////      - 0.0
////      - 1578.2004147693144
////    - - 0.0
////      - 3523.75914878101
////      - 938.3293602038799
////    - - 0.0
////      - 0.0
////                       - 1.0);
//    Mat camMatrix = Mat(3, 3, CV_, data_1)
//
//    Mat distCoeffs = vector(- - -0.48703167372875666
//    - 0.44308067917207833
//    - 0.01028402178186137
//    - 0.00673989396472976
//    - 0.3018965883819692)

    

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    // if(parser.has("dp")) {
    //     bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
    //     if(!readOk) {
    //         cerr << "Invalid detector parameters file" << endl;
    //         return 0;
    //     }
    // }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    String video = "/Users/grashin/video_detection/aruco_video_8.mp4";



    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_250));

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    }

    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);


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

        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);


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

        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        if(markersOfBoardDetected > 0)
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }
    return 0;

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
