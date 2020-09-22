#include <opencv2/viz.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

using namespace cv;
using namespace std;


Mat euler2rot(double x, double y, double z)
{
    Mat rotationMatrix;

//    double x = roll;
//    double y = pitch;
//    double z = yaw;
  // Assuming the angles are in radians.
  double ch = cos(z);
  double sh = sin(z);
  double ca = cos(y);
  double sa = sin(y);
  double cb = cos(x);
  double sb = sin(x);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

rotationMatrix = (Mat_<double>(3, 3) << m00, m01, m02, m10, m11, m12, m20, m21, m22);

  return rotationMatrix;
}


vector<Point3f> getCornersInCameraWorldBoard(double side_1, double side_2, Vec3d rvec, Vec3d tvec){
     double width = side_2;
    double height = side_1;
     Mat rot_mat;
     Rodrigues(rvec, rot_mat);
//    cout<<rot_mat<<endl;
     Mat rot_mat_t = rot_mat.t();

     double * tmp = rot_mat_t.ptr<double>(0);
//    cout<<*tmp<<endl;

     Point3f camWorldE(tmp[0]*width, tmp[1]*width, tmp[2]*width);
//    cout<<camWorldE<<endl;
     tmp = rot_mat_t.ptr<double>(1);
//    cout<<*tmp<<endl;
     Point3f camWorldF(tmp[0]*height, tmp[1]*height, tmp[2]*height);
//    cout<<camWorldF<<endl;
     Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

     vector<Point3f> ret(4,tvec_3f);
//    cout<<ret<<endl;
     ret[1] += camWorldE;
     ret[2] += camWorldF+camWorldE ;
     ret[3] += camWorldF;

     return ret;
}


int main(){
     const double    fx = 3585.1832013704093,
                     fy = 3523.75914878101,
                     cx = 1578.2004147693144,
                     cy = 938.3293602038799;

     Matx33d K = Matx33d(fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1);
//    cameraMatrix = (Mat_<double>(3, 3) << 3585.1832013704093, 0.0, 1578.2004147693144, 0.0, 3523.75914878101, 938.3293602038799, 0.0, 0.0, 1.0); // intrinsic matrix
    Mat distCoeffs = (Mat_<double>(1, 5) << 0.48703167372875666, 0.44308067917207833, 0.01028402178186137, 0.00673989396472976, 0.3018965883819692);

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_6X6_250));

//    viz::Viz3D::convertToWindowCoordinates();

//    viz::Viz3d::getViewerPose();
    // getCamera()
//    viz::Viz3d::getWidgetPose();

//    myWindow.resetCameraViewpoint());

//    viz::Viz3d::setCamera();

//    myWindow.setWidgetPose(id, pose)

//    myWindow.showWidget("Coordinate marker", viz::WCoordinateSystem(0.5), pose);
//    viz::Camera(cameraMatrix, Size());


    Affine3f imagePose(Vec3f(3.14,0,0), Vec3f(0,0,0));

    Affine3f cam_pose_1 = viz::makeCameraPose(Vec3f(3585.1832013704093, 0.0, 1578.2004147693144), Vec3f( 0.0, 3523.75914878101, 938.3293602038799), Vec3f(0.0, 0.0, 1.0));

    Vec3f cam_pos( 5.0f,5.0f,5.0f), cam_focal_point(0.0f,0.0f,0.0f), cam_y_dir(0.0f,0.0f,0.0f);

    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

//    Affine3f camerapose = viz::WCameraPosition(cameraMatrix);

    viz::Viz3d myWindow("Coordinate Frame");
    Mat mesh_1 = imread("/Users/grashin/video_detection/mesh.png");
    myWindow.setBackgroundTexture(mesh_1);
    viz::Viz3d secondWindow("Video");
    secondWindow.setBackgroundColor();
    myWindow.setWindowPosition(Point2f(0, 1100));
    secondWindow.setWindowPosition(Point2f(1500, 1100));

    Mat image;
    VideoCapture cap;
    const char* video = "/Users/grashin/video_detection/video_10.mp4";
    cap.open(video);
    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
    cap >> image;
    viz::WImage3D img(image, Size2d(frame_width, frame_height));
    secondWindow.showWidget("Image", img);

    Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,-1.0f,0.0f), Vec3f(-1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), cam_pos);

    viz::WCameraPosition cpw(0.5);
    Affine3f cam_pose_global = transform * cam_pose;
    myWindow.showWidget("CPW", cpw, cam_pose_global);
//    viz::WCameraPosition cpw_frustum(Vec2f(1.0, 0.56));
    viz::WCameraPosition cpw_frustum(K, 1.0, viz::Color::yellow()) ;
    myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose_global);

    myWindow.setViewerPose(cam_pose);
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem(0.5));

//    myWindow.showWidget("Camera Widget", viz::WCameraPosition::WCameraPosition(cameraMatrix, 1.0, viz::Color::white()));

    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(4, 5, 0.04, 0.005, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();
    Vec3d rvec, tvec;
    int k = 0;
    float roll, pitch, yaw;
    float x, y, z;
//    Point3d Pt3D_Camera(0.0,0.0,0.0),
//                Pt3D_Origin(0.0,0.0,0.0);
    while (cap.grab()) {
        Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);
        Mat rot_vec = Mat::zeros(1,3,CV_64F);

        vector<int> ids;
        vector<vector<Point2f> > corners;
        aruco::detectMarkers(image, dictionary, corners, ids);
//      draw corner widget
//        vector<Vec3d> rvecs, tvecs;
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
//        aruco::estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, distCoeffs, rvecs, tvecs);
        aruco::estimatePoseBoard(corners, ids, board, K, distCoeffs, rvec, tvec);
        if(ids.size()>0){
            for(int i=0; i<1; i++){
                aruco::drawAxis(imageCopy, K, distCoeffs, rvec, tvec, 0.1);

//                rvec = rvecs[0];
//                tvec = tvecs[0];

                roll = rvec[0];
                pitch = rvec[1];
                yaw = rvec[2];

                x = tvec[0];
                y = tvec[1];
                z = tvec[2];

                rot_vec.at<float>(0,0) = roll;
                rot_vec.at<float>(0,1) = pitch;
                rot_vec.at<float>(0,2) = yaw;
//                rot_vec.at<float>(0,0) = yaw;
//                rot_vec.at<float>(0,1) = pitch;
//                rot_vec.at<float>(0,2) = roll;

            }

            img.setImage(imageCopy);
            img.setPose(imagePose);

            Mat rot_mat;
            Rodrigues(rot_vec, rot_mat);
            Mat rot_mat_1 = euler2rot(roll, pitch, yaw);
//            cout<<rot_mat<<endl<<euler2rot(rvec)<<endl;
            Mat h, g;
//            RQDecomp3x3( rot_mat_1, h, g);

            Affine3d pose(rot_mat_1, Vec3f(x, y, z));
//            cout<<pose.matrix;
//            Affine3f pose(rot_mat, tvec);
//            pose = transform*pose;

            myWindow.setWidgetPose("CPW", pose);
            myWindow.setWidgetPose("CPW_FRUSTUM", pose);
            Vec3d Pt3d_Object(0.10,0.10,1.0);
            Affine3d point_pose(Mat::eye(3, 3, CV_64F), Pt3d_Object);
            if (k<100000){
    //        cube_widget.setPose(pose);
                vector<Point3f> point_in_cw = getCornersInCameraWorldBoard(0.22,0.175, rvec, tvec);

//                point_in_cw = transform * point_in_cw;
//                cout<<point_in_cw;
                viz::WLine line_1(point_in_cw[0], point_in_cw[1], viz::Color::red());
                viz::WLine line_2(point_in_cw[1], point_in_cw[2], viz::Color::red());
                viz::WLine line_3(point_in_cw[2], point_in_cw[3], viz::Color::red());
                viz::WLine line_4(point_in_cw[3], point_in_cw[0], viz::Color::red());
                myWindow.showWidget("Line_1", line_1, pose);
                myWindow.showWidget("Line_2", line_2, pose);
                myWindow.showWidget("Line_3", line_3, pose);
                myWindow.showWidget("Line_4", line_4, pose);

                line_1.setRenderingProperty(viz::LINE_WIDTH, 4.0);
                line_2.setRenderingProperty(viz::LINE_WIDTH, 4.0);
                line_3.setRenderingProperty(viz::LINE_WIDTH, 4.0);
                line_4.setRenderingProperty(viz::LINE_WIDTH, 4.0);
                k+=1;
//                viz::WCube cube_widget(point_in_cw[0], Point3f(x+0.219, y+0.178, z+0.1), true, viz::Color::purple());
//                cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
//                myWindow.showWidget("Cube Widget", cube_widget, point_pose);
            }
            Affine3d AffineTransform= Affine3d(rvec,tvec);
//            cout<<AffineTransform.matrix<<pose.matrix<<endl<<Pt3D_Camera<<endl<<Pt3D_Origin<<endl;
//            Pt3D_Camera= AffineTransform * Pt3D_Origin;
//            cout<<Pt3D_Camera<<endl;
            myWindow.spinOnce(50, true);
            secondWindow.spinOnce(50, true);
        }
    }
    return 0;
}











//#include <opencv2/opencv.hpp>
//#include <opencv2/viz.hpp>
//
//#include <iostream>
//#include <fstream>
//#include <string>
//
//using namespace std;
//using namespace cv;
//
//bool bCamView = false;
//
//
//void keyboard_callback(const viz::KeyboardEvent &event, void* cookie)
//{
//    if (event.action == 0 && !event.symbol.compare("s"))
//        bCamView = !bCamView;
//}
//
//int main(int argc, char* argv[])
//{
//    /// Create 3D windows
//    viz::Viz3d Window_3D("Estimation Coordinate Frame");
//    Window_3D.setBackgroundColor(); // black by default
//    Window_3D.registerKeyboardCallback(&keyboard_callback);
//
//    // Create the pointcloud
//    cout << "Reading the 3D points  ... ";
//
//    const int NoOfCamera = 10;
//    const double    fx = 1239.911,
//                    fy = 1239.911,
//                    cx = 519.909,
//                    cy = 246.656;
//
//    Matx33d K = Matx33d(fx, 0, cx,
//                        0, fy, cy,
//                        0, 0, 1);
//
//    vector<Affine3d> Cam_Trajectory;
//    Mat RotationVector(1, 3, CV_64FC1, Scalar(0.0));
//    Mat TranslationVector(3, 1, CV_64FC1, Scalar(1.0));
//    for (size_t i = 0; i < NoOfCamera; ++i)
//    {
//        Cam_Trajectory.push_back(Affine3d(RotationVector, TranslationVector));
//        TranslationVector.at<double>(2, 0) += 1.0;
//    }
//
//    Vec3d Pt3d_Object(0.10,0.10,15.0);
//    /// Wait for key 'q' to close the window
//    cout << endl << "Press:                       " << endl;
//    cout << " 's' to switch the camera view" << endl;
//    cout << " 'q' to close the windows    " << endl;
//
//
//    if (Cam_Trajectory.size() > 0)
//    {
//        // animated trajectory
//        int idx = 0, forw = -1, n = static_cast<int>(Cam_Trajectory.size());
//
//        while (!Window_3D.wasStopped())
//        {
//            /// Render a 3D cube
//            Affine3d point_pose(Mat::eye(3, 3, CV_64F), Pt3d_Object);
//            viz::WCube cube_widget(Point3f(0.1, 0.1, 0.0), Point3f(0.0, 0.0, -0.1), true, viz::Color::blue());
//            cube_widget.setRenderingProperty(viz::LINE_WIDTH, 2.0);
//            Window_3D.showWidget("Cube1", cube_widget, point_pose);
//
//            Affine3d cam_pose = Cam_Trajectory[idx];
//
//            viz::WCameraPosition cpw(0.25); // Coordinate axes
//            viz::WCameraPosition cpw_frustum(K, 0.3, viz::Color::yellow()); // Camera frustum
//            if (bCamView)
//            {
//                Window_3D.setViewerPose(cam_pose);
//            }
//            else
//            {
//                // render complete trajectory
//                Window_3D.showWidget("cameras_frames_and_lines_est", viz::WTrajectory(Cam_Trajectory, viz::WTrajectory::PATH, 1.0, viz::Color::green()));
//
//                Window_3D.showWidget("CPW", cpw, cam_pose);
//                Window_3D.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
//            }
//
//            // update trajectory index (spring effect)
//            forw *= (idx == n || idx == 0) ? -1 : 1;
//            idx += forw;
//
//            // frame rate 1s
//            Window_3D.spinOnce(100, true);
//            Window_3D.removeAllWidgets();
//
//        }
//
//    }
//    Window_3D.close();
//    return 0;
//}
//
//















//vector<Point3d> GetWorldPoint(Mat input, Vec3d rvec, Vec3d tvec)
//{
//    Mat rot_mat;
//    double x = input.at<double>(0);
//    double y = input.at<double>(1);
//    double z = input.at<double>(2);
//    Rodrigues(Mat(rvec[0], rvec[1], rvec[2]), rot_mat);
//
//    Mat pointProject;
//    pointProject = (rot_mat * Mat(x,y,z));
//    return tvec + new Vec3d(pointProject.at<double>(0, 0), pointProject.at<double>(0, 1), pointProject.at<double>(0, 2));
//}
//vector<Point3f> getRealWorldCoordinates(vector<int> indices){
//
//    vector<Point3f> pts_vec;
//    for(int i = 0 ; i < indices.size(); i++){
//
//        int id = indices.at(i);
//        double x = tags_matrix(id, 0);
//        double y = tags_matrix(id, 1);
//        double z = tags_matrix(id, 2); // 0 (all on the same plane)
//        double dist = 0.0445 / 2;
//
//        Point3f p;
//        p.z = 0;
//
//        // counter clockwise from top left corners and ending with center
//        p.x = x + dist;
//        p.y = y + dist;
//        pts_vec.push_back(p);
//
//        p.x = x - dist;
//        p.y = y + dist;
//        pts_vec.push_back(p);
//
//        p.x = x - dist;
//        p.y = y - dist;
//        pts_vec.push_back(p);
//
//        p.x = x + dist;
//        p.y = y - dist;
//        pts_vec.push_back(p);
//
//        //        p.x = x;
//        //        p.y = y;
//        //        pts_vec.push_back(p);
//    }
//
//
//    return pts_vec;
//
//}
//vector<Vec3d> getPoint(Vec3d rotation, Vec3d translation){
    //    Mat translation_1;
    //    Mat rotation_1;
    //    Mat R;
    //    Rodrigues(rotation, R);
    //    R = R.t();
    //    translation_1 = -R * translation;
    //    Rodrigues(R, rotation_1);
    //    return rotation_1;
    //}
//         Create affine pose from rotation and translation vectors

//        vector<cv::Point3f> pts = {Point3f(0, 0, 0)};
//        vector<cv::Point2f> imagePoints;
//        projectPoints(pts, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
//        if (has_x && has_y && has_z) {
//            worldCoordinates.emplace_back(x, y, z);
//            auto marker = mapMarkerCorners[i];
//
//            // This is probably not so accurate
//            imageCoordinates.emplace_back(imagePoints[0]);
//        }


//            solvePnP(worldCoordinates, imageCoordinates, cameraMatrix, distCoeffs,rvec, tvec, false, SOLVEPNP_ITERATIVE);
//            roll = rvec[0];
//            pitch = rvec[1];
//            yaw = rvec[2];
//
//            x = tvec[0];
//            y = tvec[1];
//            z = tvec[2];
//            Mat rotation_matrix;
//        cout<<rot_vec;
//        rotation_matrix = euler2rot(rot_vec);
//        cout<<rotation_matrix;







//vector<Point3f> getCornersInCameraWorld(double side, Vec3d rvec, Vec3d tvec){
//     double half_side = side/2;
//     Mat rot_mat;
//     Rodrigues(rvec, rot_mat);
//    cout<<rot_mat<<endl;
//     Mat rot_mat_t = rot_mat.t();
//
//     double * tmp = rot_mat_t.ptr<double>(0);
//    cout<<*tmp<<endl;
//
//     Point3f camWorldE(tmp[0]*half_side, tmp[1], tmp[2]);
//    cout<<camWorldE<<endl;
//     tmp = rot_mat_t.ptr<double>(1);
//    cout<<*tmp<<endl;
//     Point3f camWorldF(tmp[0], tmp[1]*half_side, tmp[2]);
//    cout<<camWorldF<<endl;
//     Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);
//
//     vector<Point3f> ret(4,tvec_3f);
//    cout<<ret<<endl;
//     ret[0] +=  camWorldE + camWorldF;
//     ret[1] += -camWorldE + camWorldF;
//     ret[2] += -camWorldE - camWorldF;
//     ret[3] +=  camWorldE - camWorldF;
//
//     return ret;
//}
