#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <math.h>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

//#include <chrono>

using namespace std;
using namespace cv;

int main(){
    
    
    
    
    
//     phone_1 :
   const double    fx = 3585.1832013704093,
                    fy = 3523.75914878101,
                    cx = 1578.2004147693144,
                    cy = 938.3293602038799;
//    web :
//    const double    fx = 971.09050398,
//                         fy = 968.24582919,
//                         cx = 643.07484029,
//                         cy = 356.36661208;
//    samsung:
   // const double    fx = 1553.341974179234,
   //                           fy = 1595.8012466794,
   //                           cx = 931.5687101310722,
   //                           cy = 580.6287476101777;

     Matx33d K = Matx33d(fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1);
//    cameraMatrix = (Mat_<double>(3, 3) << 3585.1832013704093, 0.0, 1578.2004147693144, 0.0, 3523.75914878101, 938.3293602038799, 0.0, 0.0, 1.0); // intrinsic matrix
    
   Mat distCoeffs = (Mat_<double>(1, 5) << 0.48703167372875666, 0.44308067917207833, 0.01028402178186137, 0.00673989396472976, 0.3018965883819692);
//    Mat distCoeffs = (Mat_<double>(1, 5) <<-0.08330321124565016, 0.5264065138107696, -0.0009070331602434745, 0.004016078243366027, -0.9275764560623708);
    // Mat distCoeffs = (Mat_<double>(1, 5) <<-0.09444649530336692, -0.6649654808745757, -0.012299296986908024, -0.013865770040306846, 0.8044275793419697);

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_6X6_250));

    Mat image;
    VideoCapture cap;
    const char* video_1 = "/Users/grashin/video_detection/red_marker_1.mp4";
    cap.open(video_1);
    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
    cap >> image;

   int c = 0;
    VideoWriter video("/Users/grashin/video_detection/examples/outcpp.avi",VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height), true);
    while (cap.grab()) {
        Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);
        Mat rot_vec = Mat::zeros(1,3,CV_64F);


            int k_sum = 0;
            vector<int> ids;
            vector<vector<Point2f>> corners;
            aruco::detectMarkers(image, dictionary, corners, ids);
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            
            vector<Vec3d> rvecs, tvecs;
            
            // MARKER LENGTH NOT 0.04 AT ALL  !!!!!!
            float length = 0.0275;
            aruco::estimatePoseSingleMarkers(corners, length, K, distCoeffs, rvecs, tvecs);
    //        aruco::estimatePoseBoard(corners, ids, board, K, distCoeffs, rvec, tvec);
            vector<Point3f> axisPoints;
            axisPoints.push_back(Point3f(0, 0, 0));
            axisPoints.push_back(Point3f(length, 0, 0));
            axisPoints.push_back(Point3f(0, length, 0));
            axisPoints.push_back(Point3f(0, 0, length));
            vector< Point2f > imagePoints;
        if (ids.size()>0){
            projectPoints(axisPoints, rvecs, tvecs, K, distCoeffs, imagePoints);

            // draw axis lines
//            line(imageCopy, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
            line(imageCopy, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
//            line(imageCopy, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
        }
           
            video.write(imageCopy);
            c++;

            
        }
         
          video.release();
    return 0;
}





