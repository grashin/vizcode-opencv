
#include <opencv2/viz.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <math.h>
#include <string>
#include <chrono>

using namespace cv;
using namespace std;
using namespace cv::viz;


vector<Point3f> getCornersInCameraWorld(double side, Vec3d rvec, Vec3d tvec){
     double half_side = side/2;
     Mat rot_mat;
     Rodrigues(rvec, rot_mat);
//    cout<<rot_mat<<endl;
     Mat rot_mat_t = rot_mat.t();

     double * tmp = rot_mat_t.ptr<double>(0);
//    cout<<*tmp<<endl;

     Point3f camWorldE(tmp[0]*half_side, tmp[1]*half_side, tmp[2]*half_side);
//    cout<<camWorldE<<endl;
     tmp = rot_mat_t.ptr<double>(1);
//    cout<<*tmp<<endl;
     Point3f camWorldF(tmp[0]*half_side, tmp[1]*half_side, tmp[2]*half_side);
//    cout<<camWorldF<<endl;
     Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

     vector<Point3f> ret(4,tvec_3f);
//    cout<<ret<<endl;
     ret[0] +=  camWorldE + camWorldF;
     ret[1] += -camWorldE + camWorldF;
     ret[2] += -camWorldE - camWorldF;
     ret[3] +=  camWorldE - camWorldF;

     return ret;
}


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
//     const double    fx = 3585.1832013704093,
//                     fy = 3523.75914878101,
//                     cx = 1578.2004147693144,
//                     cy = 938.3293602038799;
    const double    fx = 971.09050398,
                         fy = 968.24582919,
                         cx = 643.07484029,
                         cy = 356.36661208;

     Matx33d K = Matx33d(fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1);
//    cameraMatrix = (Mat_<double>(3, 3) << 3585.1832013704093, 0.0, 1578.2004147693144, 0.0, 3523.75914878101, 938.3293602038799, 0.0, 0.0, 1.0); // intrinsic matrix
    
//    Mat distCoeffs = (Mat_<double>(1, 5) << 0.48703167372875666, 0.44308067917207833, 0.01028402178186137, 0.00673989396472976, 0.3018965883819692);
    Mat distCoeffs = (Mat_<double>(1, 5) <<-0.08330321124565016, 0.5264065138107696, -0.0009070331602434745, 0.004016078243366027, -0.9275764560623708);

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

//    Affine3f cam_pose_1 = viz::makeCameraPose(Vec3f(3585.1832013704093, 0.0, 1578.2004147693144), Vec3f( 0.0, 3523.75914878101, 938.3293602038799), Vec3f(0.0, 0.0, 1.0));

    Vec3f cam_pos( 2.0f,3.0f,2.0f), cam_focal_point(0.0f,0.0f,0.0f), cam_y_dir(0.0f,0.0f,0.0f);

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
    const char* video = "/Users/grashin/video_detection/video_11.mov";
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
    viz::WCameraPosition cpw_frustum(K, 0.4, viz::Color::yellow()) ;
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
    float e , b , w;
    float length_cylinder = 0.22;
    double dist;
    ostringstream ids_to_print;
    vector<int> all_ids;
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
        vector<Vec3d> rvecs, tvecs;
        aruco::drawDetectedMarkers(imageCopy, corners, ids);
        aruco::estimatePoseSingleMarkers(corners, 0.04, K, distCoeffs, rvecs, tvecs);
//        aruco::estimatePoseBoard(corners, ids, board, K, distCoeffs, rvec, tvec);
        if(ids.size()>0){
            for(int i=0; i<ids.size(); i++){
                aruco::drawAxis(imageCopy, K, distCoeffs, rvecs[i], tvecs[i], 0.1);
//                drawFrameAxes(imageCopy, K, distCoeffs, rvec, tvec, 0.3);

            }
            rvec = rvecs[0];
            tvec = tvecs[0];
            
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

            Mat RotX(3, 3, cv::DataType<double>::type);
            setIdentity(RotX);
            RotX.at<double>(4) = -1;
            RotX.at<double>(8) = -1;

            Mat H;
            Rodrigues(rvec, H);

            H = H.t();
            Mat rvecConverted;
            Rodrigues(H, rvecConverted);
            rvecConverted = RotX * rvecConverted;

            Mat tvecConverted = -H * tvec;
            tvecConverted = RotX * tvecConverted;

    //        payload.rotationVector = SCNVector4Make(rvecConverted.at<double>(0), rvecConverted.at<double>(1), rvecConverted.at<double>(2), norm(rvecConverted));
    //        payload.translationVector = SCNVector3Make(tvecConverted.at<double>(0), tvecConverted.at<double>(1), tvecConverted.at<double>(2));

            Mat rot_mat;
            Rodrigues(rot_vec, rot_mat);
            Mat rot_mat_1 = euler2rot(roll, pitch, yaw);
//            Affine3d AffineTransform = Affine3d(rvec,tvec).inv();
//            cout<< AffineTransform.matrix<<endl;
//            cout<<rot_mat<<endl<<euler2rot(rvec)<<endl;

            Mat h, g;
            RQDecomp3x3( rot_mat, h, g);
            Mat r = h*g*rot_mat;

            if (k<1){
                e = x;
                b = y;
                w = z;
            }
            Affine3d pose(RotX, Vec3f(x, y, z));
            Affine3d pose_10(-h
                             , Vec3f(e, b, w));
//            Affine3d pose(H, Vec3f(e, b, w));
//            cout<<pose.matrix;
//            Affine3d pose(rot_mat, Vec3f(x,y,z));
//            pose = transform*pose;
//            pose = AffineTransform.inv();

            myWindow.setWidgetPose("CPW", pose_10);
//            Affine3d pose_1_1 = myWindow.getWidgetPose("CPW");
//            cout<<pose_1_1.matrix;

            myWindow.setWidgetPose("CPW_FRUSTUM", pose_10);
            Vec3d Pt3d_Object(0.10,0.10,1.0);
            Affine3d point_pose(Mat::eye(3, 3, CV_64F), Pt3d_Object);
            if (k<100000){
//                point_in_cw = transform * point_in_cw;
//                cout<<point_in_cw;
                vector<Point3f> point_in_cw;
        
                for (int i =0; i<ids.size(); i++){
                    point_in_cw = getCornersInCameraWorld(0.04, rvecs[i], tvecs[i]);
                    myWindow.showWidget(to_string(4*i), WLine(point_in_cw[0], point_in_cw[1], viz::Color::red()), pose);
                    myWindow.showWidget(to_string(4*i+1), WLine(point_in_cw[1], point_in_cw[2], viz::Color::red()), pose);
                    myWindow.showWidget(to_string(4*i+2), WLine(point_in_cw[2], point_in_cw[3], viz::Color::red()), pose);
                    myWindow.showWidget(to_string(4*i+3), WLine(point_in_cw[3], point_in_cw[0], viz::Color::red()), pose);
                    
                    myWindow.getWidget(to_string(4*i)).setRenderingProperty(viz::LINE_WIDTH, 4.0);
                    myWindow.getWidget(to_string(4*i+1)).setRenderingProperty(viz::LINE_WIDTH, 4.0);
                    myWindow.getWidget(to_string(4*i+2)).setRenderingProperty(viz::LINE_WIDTH, 4.0);
                    myWindow.getWidget(to_string(4*i+3)).setRenderingProperty(viz::LINE_WIDTH, 4.0);

                }
                
                if (k == 0){
                     all_ids = ids;
                     copy(all_ids.begin(), all_ids.end()-1, ostream_iterator<int>(ids_to_print, ","));
                     ids_to_print << all_ids.back();
                 }
                 for (int i = 0; i<ids.size(); i++){
                     if (!(find(all_ids.begin(), all_ids.end(), ids[i]) != all_ids.end())){
                         all_ids.push_back(ids[i]);
                         ids_to_print<<","<<all_ids.back();
                     }
                 }
                dist = sqrt(x*x+y*y+z*z);
                viz::WCylinder babina(Point3d(x,y,z),Point3d(x,y,z+length_cylinder), 0.08, 20, viz::Color::purple());
                myWindow.showWidget("Cylinder", babina, pose);
            }
//            cout<<AffineTransform.matrix<<pose.matrix<<endl<<Pt3D_Camera<<endl<<Pt3D_Origin<<endl;
//            Pt3D_Camera= AffineTransform * Pt3D_Origin;
//            cout<<Pt3D_Camera<<endl;
//            cout<<x<<y<<z<<endl;

        }
        img.setImage(imageCopy);
        img.setPose(imagePose);
        string s_1 = "Found " + to_string(ids.size()) + " markers";
        char const *pchar_1 = s_1.c_str();
        viz::WText text_1(pchar_1, Point2f(100.0,100.0), 30.0, viz::Color::white());
        secondWindow.showWidget("Text_1", text_1);

        string s_2 = "Dist to axes " + to_string(dist);
        char const *pchar_2 = s_2.c_str();
        viz::WText text_2(pchar_2, Point2f(400.0,100.0), 30.0, viz::Color::bluberry());
        secondWindow.showWidget("Text_2", text_2);


        string s_3 = "IDs: " + ids_to_print.str();
        char const *pchar_3 = s_3.c_str();
        viz::WText text_3(pchar_3, Point2f(100.0,0.0), 30.0, viz::Color::white());
        secondWindow.showWidget("Text_3", text_3);
    
        k+=1;
        myWindow.spinOnce(30, true);
        secondWindow.spinOnce(30, true);
    }
    
//    waitKey(100);
    return 0;
}
