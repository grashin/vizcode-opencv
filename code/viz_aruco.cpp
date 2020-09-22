// Camera calibration outputs
#include <opencv2/viz.hpp>
#include <iostream>
using namespace cv;
using namespace std;
cv::Mat cameraMatrix, distCoeffs;
loadIntrinsicCameraParameters(cameraMatrix, distCoeffs);

// Marker dictionary
Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

viz::Viz3d myWindow("Coordinate Frame");

cv::Mat image;

// Webcam frame pose, without this frame is upside-down
Affine3f imagePose(Vec3f(3.14159,0,0), Vec3f(0,0,0));

// Viz viewer pose to see whole webcam frame
Vec3f cam_pos( 0.0f,0.0f,900.0f), cam_focal_point(0.0f,0.0f,0.0f), cam_y_dir(0.0f,0.0f,0.0f);
Affine3f viewerPose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

// Video capture from source
VideoCapture cap(camID);
int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
cap >> image;

// Load mash data
viz::WMesh batman(viz::Mesh::load("../data/bat.ply"));
viz::WImage3D img(image, Size2d(frame_width, frame_height));

// Show camera frame, mesh and a coordinate widget (for debugging)
myWindow.showWidget("Image", img);
myWindow.showWidget("Batman", batman);
myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem(5.0));

myWindow.setFullScreen(true);
myWindow.setViewerPose(viewerPose);

// Rotation vector of 3D model
Mat rot_vec = Mat::zeros(1,3,CV_32F);
cv::Vec3d rvec, tvec;

// ARUCO outputs
float roll, pitch, yaw;
float x, y, z;

while (!myWindow.wasStopped()) {

    if (cap.read(image)) {
        cv::Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);

        // Marker detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.size() > 0){

            // Draw a green line around markers
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            vector<Vec3d> rvecs, tvecs;

            // Get rotation and translation vectors of each markers
            cv::aruco::estimatePoseSingleMarkers(corners, 0.0365, cameraMatrix, distCoeffs, rvecs, tvecs);

            for(int i=0; i<ids.size(); i++){

                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

                // Take only the first marker rotation and translation to visualize 3D model on this marker
                rvec = rvecs[0];
                tvec = tvecs[0];

                roll = rvec[0];
                pitch = rvec[1];
                yaw = rvec[2];

                x = tvec[0];
                y = tvec[1];
                z = tvec[2];

                qDebug() << rvec[0] << "," << rvec[1] << "," << rvec[2] << "---" << tvec[0] << "," << tvec[1] << "," << tvec[2];
            }
        }
        // Show camera frame in Viz window
        img.setImage(imageCopy);
        img.setPose(imagePose);
    }

    // Create affine pose from rotation and translation vectors
    // rot_vec.at<float>(0,0) = roll;
    // rot_vec.at<float>(0,1) = pitch;
    // rot_vec.at<float>(0,2) = yaw;
    // Mat rot_mat;

    // Rodrigues(rot_vec, rot_mat);

    // Affine3f pose(rot_mat, Vec3f(x, y, z));

    // // Set the pose of 3D model
    // batman.setPose(pose);

    myWindow.spinOnce(1, true);
}