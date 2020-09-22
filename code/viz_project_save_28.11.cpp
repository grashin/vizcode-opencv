#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <math.h>
#include <string>
//#include <chrono>

using namespace std;
using namespace cv;
using namespace cv::viz;

ostringstream ids_to_print_1;
ostringstream ids_to_print_2;
ostringstream ids_to_print_3;
ostringstream ids_to_print_4;
vector<int> all_ids_1;
vector<int> all_ids_2;
vector<int> all_ids_3;
vector<int> all_ids_4;


//сделать не айди маркеров а айди_словарь, идея: сделать не 4 вида маркеров а записывать их в общую но айдимаркера_словарь
bool find_motion(int j, int k, int id, float *** dist_1){
    if(k>10){
        if((abs((dist_1[j][id][k-10]-dist_1[j][id][k]))>0.2)&&(dist_1[j][id][k-10] != 0)&&(dist_1[j][id][k] != 0)){
            cout<< "vzyali!!!"<<endl<<"id: "<<id<<endl;
            return true;
        }
    }
    return false;
}

void print_vector(vector<float> const &input)
{
    for (int i = 0; i < input.size(); i++) {
        std::cout << input.at(i) <<" ";
    }
}


void ids_print(int k ,int j, vector<int> ids,Viz3d secondWindow){
    switch (j){
        case 0:{
            if (k == 0){
                all_ids_1 = ids;
                copy(all_ids_1.begin(), all_ids_1.end()-1, ostream_iterator<int>(ids_to_print_1, ","));
                ids_to_print_1 << all_ids_1.back();
            }
            for (int i = 0; i<ids.size(); i++){
                 if (!(find(all_ids_1.begin(), all_ids_1.end(), ids[i]) != all_ids_1.end())){
                     all_ids_1.push_back(ids[i]);
                     ids_to_print_1<<","<<to_string(ids[i]);
                 }
             }
             
             string s_3 = "IDs_from_dict"+to_string(j)+": " + ids_to_print_1.str();
             char const *pchar_3 = s_3.c_str();
             secondWindow.showWidget("Text_ids_to_print_"+to_string(j), WText(pchar_3, Point2f(100.0,20.0+20*j), 30.0, viz::Color::white()));
            
            break;
        }
        case 1:{
            if (k == 0){
                all_ids_2 = ids;
                copy(all_ids_2.begin(), all_ids_2.end()-1, ostream_iterator<int>(ids_to_print_2, ","));
                ids_to_print_2 << all_ids_2.back();
            }
            for (int i = 0; i<ids.size(); i++){
                 if (!(find(all_ids_2.begin(), all_ids_2.end(), ids[i]) != all_ids_2.end())){
                     all_ids_2.push_back(ids[i]);
                     ids_to_print_2<<","<<to_string(ids[i]);
                 }
             }
             
             string s_3 = "IDs_from_dict"+to_string(j)+": " + ids_to_print_2.str();
             char const *pchar_3 = s_3.c_str();
             secondWindow.showWidget("Text_ids_to_print_"+to_string(j), WText(pchar_3, Point2f(100.0,20.0+20*j), 30.0, viz::Color::white()));
            
            
            break;
        }
        case 2:{
            if (k == 0){
                all_ids_3 = ids;
                copy(all_ids_3.begin(), all_ids_3.end()-1, ostream_iterator<int>(ids_to_print_3, ","));
                ids_to_print_3 << all_ids_3.back();
            }
            for (int i = 0; i<ids.size(); i++){
                 if (!(find(all_ids_3.begin(), all_ids_3.end(), ids[i]) != all_ids_3.end())){
                     all_ids_3.push_back(ids[i]);
                     ids_to_print_3<<","<<to_string(ids[i]);
                 }
             }
             
             string s_3 = "IDs_from_dict"+to_string(j)+": " + ids_to_print_3.str();
             char const *pchar_3 = s_3.c_str();
             secondWindow.showWidget("Text_ids_to_print_"+to_string(j), WText(pchar_3, Point2f(100.0,20.0+20*j), 30.0, viz::Color::white()));
            
            
            break;
        }
        case 3:{
            if (k == 0){
                all_ids_4 = ids;
                copy(all_ids_4.begin(), all_ids_4.end()-1, ostream_iterator<int>(ids_to_print_4, ","));
                ids_to_print_4 << all_ids_1.back();
            }
            for (int i = 0; i<ids.size(); i++){
                 if (!(find(all_ids_4.begin(), all_ids_4.end(), ids[i]) != all_ids_4.end())){
                     all_ids_4.push_back(ids[i]);
                     ids_to_print_4<<","<<to_string(ids[i]);
                 }
             }
             
             string s_3 = "IDs_from_dict"+to_string(j)+": " + ids_to_print_4.str();
             char const *pchar_3 = s_3.c_str();
             secondWindow.showWidget("Text_ids_to_print_"+to_string(j), WText(pchar_3, Point2f(100.0,20.0+20*j), 30.0, viz::Color::white()));
            
            
            break;
        }
    }
}

void draw_cylinder(vector<int> ids_cylinder, int j, vector<Vec3d> tvecs, Affine3d pose,Viz3d myWindow, Viz3d secondWindow){
    switch (j){
        case 0:{
            float length_cylinder = 0.22;
            float radius = 0.08;
            for (int i =0; i<ids_cylinder.size(); i++){
                Vec3d tvec = tvecs[i];
                float x = tvec[0], y = tvec[1], z = tvec[2];
            myWindow.showWidget("cylinder_"+to_string(ids_cylinder[i])+"from_dict"+to_string(j), WCylinder(Point3d(x-0.01,y-0.05,z),Point3d(x-0.01,y-0.05,z+length_cylinder), radius, 20, viz::Color::purple()), pose);
            }
            break;
        }
        case 1:{
           float length_cylinder = 0.2;
            float radius = 0.1;
            for (int i =0; i<ids_cylinder.size(); i++){
                Vec3d tvec = tvecs[i];
                float x = tvec[0], y = tvec[1], z = tvec[2];
            myWindow.showWidget("cylinder_"+std::to_string(ids_cylinder[i])+"from_dict"+to_string(j), WCylinder(Point3d(x-0.01,y-0.05,z),Point3d(x-0.01,y-0.05,z+length_cylinder), radius, 20, viz::Color::purple()), pose);
            }
            break;
        }
        case 2:{
          float length_cylinder = 0.15;
            float radius = 0.05;
            for (int i =0; i<ids_cylinder.size(); i++){
                Vec3d tvec = tvecs[i];
                float x = tvec[0], y = tvec[1], z = tvec[2];
            myWindow.showWidget("cylinder_"+std::to_string(ids_cylinder[i])+"from_dict"+to_string(j), WCylinder(Point3d(x-0.01,y-0.05,z),Point3d(x-0.01,y-0.05,z+length_cylinder), radius, 20, viz::Color::purple()), pose);
            }
            break;
        }
        case 3:{
           float length_cylinder = 0.1;
           float radius = 0.02;
            for (int i =0; i<ids_cylinder.size(); i++){
                Vec3d tvec = tvecs[i];
                float x = tvec[0], y = tvec[1], z = tvec[2];
            myWindow.showWidget("cylinder_"+std::to_string(ids_cylinder[i])+"from_dict"+to_string(j), WCylinder(Point3d(x-0.01,y-0.05,z),Point3d(x-0.01,y-0.05,z+length_cylinder), radius, 20, viz::Color::purple()), pose);
            }
            break;
        }
    }
}


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
    
    int dim1 = 4, dim2 = 50, dim3 = 450;
    float ***dist_1=new float **[dim1];
    for (int i=0;i<dim1;i++)
    {
        dist_1[i]=new float *[dim2];
        for (int j=0;j<dim2;j++)
            dist_1[i][j]=new float [dim3];
    }
    for ( int i = 0; i <dim1;i++){
        for ( int j = 0; j <dim2;j++){
            for ( int z = 0; z <dim3;z++){
                dist_1[i][j][z] = 0;
                
            }
        }
        
    }
    
    
    
    
//     phone_1 :
//    const double    fx = 3585.1832013704093,
//                     fy = 3523.75914878101,
//                     cx = 1578.2004147693144,
//                     cy = 938.3293602038799;
//    web :
//    const double    fx = 971.09050398,
//                         fy = 968.24582919,
//                         cx = 643.07484029,
//                         cy = 356.36661208;
//    samsung:
   const double    fx = 1553.341974179234,
                             fy = 1595.8012466794,
                             cx = 931.5687101310722,
                             cy = 580.6287476101777;

     Matx33d K = Matx33d(fx, 0, cx,
                         0, fy, cy,
                         0, 0, 1);
//    cameraMatrix = (Mat_<double>(3, 3) << 3585.1832013704093, 0.0, 1578.2004147693144, 0.0, 3523.75914878101, 938.3293602038799, 0.0, 0.0, 1.0); // intrinsic matrix
    
//    Mat distCoeffs = (Mat_<double>(1, 5) << 0.48703167372875666, 0.44308067917207833, 0.01028402178186137, 0.00673989396472976, 0.3018965883819692);
//    Mat distCoeffs = (Mat_<double>(1, 5) <<-0.08330321124565016, 0.5264065138107696, -0.0009070331602434745, 0.004016078243366027, -0.9275764560623708);
    Mat distCoeffs = (Mat_<double>(1, 5) <<-0.09444649530336692, -0.6649654808745757, -0.012299296986908024, -0.013865770040306846, 0.8044275793419697);

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    vector<Ptr<aruco::Dictionary>> dictionary;
    dictionary.push_back(aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_4X4_50)));
    dictionary.push_back(aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_5X5_50)));
    dictionary.push_back(aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_6X6_50)));
    dictionary.push_back(aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(aruco::DICT_7X7_50)));

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
    const char* video = "/Users/grashin/video_detection/samsung_videos/samsung_24.mp4";
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
        aruco::GridBoard::create(4, 5, 0.04, 0.005, dictionary[0]);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();
    Vec3d rvec, tvec;
    vector<int> k = {0, 0, 0 ,0};
    float roll, pitch, yaw;
    float x, y, z;
    float e , b , w;
    vector<float> dist;
    int trf = 0;
//    Point3d Pt3D_Camera(0.0,0.0,0.0),
//                Pt3D_Origin(0.0,0.0,0.0);
    while (cap.grab()) {
        Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);
        Mat rot_vec = Mat::zeros(1,3,CV_64F);
        int found_markers = 0;
        for (int j = 0; j < 4; j++){
            vector<int> ids;
            vector<vector<Point2f>> corners;
            aruco::detectMarkers(image, dictionary[j], corners, ids);
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, 0.04, K, distCoeffs, rvecs, tvecs);
    //        aruco::estimatePoseBoard(corners, ids, board, K, distCoeffs, rvec, tvec);
            found_markers += ids.size();
            if(ids.size()>0){

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

                if (k[j]<1){
                    e = x;
                    b = y;
                    w = z;
                }
                Affine3d pose(RotX, Vec3f(x, y, z));
    //            myWindow.showWidget("trajectory", viz::WTrajectory(pose));


                Affine3d pose_10(-h, Vec3f(e,b,w));
    //            Affine3d pose(H, Vec3f(e, b, w));
    //            cout<<pose.matrix;
    //            Affine3d pose(rot_mat, Vec3f(x,y,z));
    //            pose = transform*pose;
    //            pose = AffineTransform.inv();

                myWindow.setWidgetPose("CPW", pose_10);
    //            Affine3d pose_1_1 = myWindow.getWidgetPose("CPW");
    //            cout<<pose_1_1.matrix;

                myWindow.setWidgetPose("CPW_FRUSTUM", pose_10);
//                Vec3d Pt3d_Object(0.10,0.10,1.0);
//                Affine3d point_pose(Mat::eye(3, 3, CV_64F), Pt3d_Object);
    //                point_in_cw = transform * point_in_cw;
    //                cout<<point_in_cw;
                vector<Point3f> point_in_cw;


                ids_print(k[j], j, ids, secondWindow);
        
                
                for (int i = 0; i<ids.size(); i++){
                    aruco::drawAxis(imageCopy, K, distCoeffs, rvecs[i], tvecs[i], 0.1);
    //                drawFrameAxes(imageCopy, K, distCoeffs, rvec, tvec, 0.3);
                    tvec = tvecs[i];
                    x = tvec[0];
                    y = tvec[1];
                    z = tvec[2];
                    dist_1[j][ids[i]][k[j]] = sqrt(x*x+y*y+z*z);
                    dist.push_back(sqrt(x*x+y*y+z*z));
                    string str = to_string(floor(100*dist.back())/100);
//                    print_vector(dist);
                    bool fdf = find_motion(j, k[j], ids[i], dist_1);
                    string dist = "Dist" + to_string(ids[i]) +"_"+to_string(j)+ ": " + str.substr(0, str.find_last_not_of('0') + 1);
                    char const *pchar = dist.c_str();
                    secondWindow.showWidget("dist_"+to_string(ids[i])+"_from_dict_"+to_string(j), WText(pchar,Point2f(1100.0,0.0+ids[i]*20+40*j), 20.0, viz::Color::white() ));
                    
                    point_in_cw = getCornersInCameraWorld(0.04, rvecs[i], tvecs[i]);
                    myWindow.showWidget(to_string(4*i)+"line_"+to_string(j), WLine(point_in_cw[0], point_in_cw[1], viz::Color::red()), pose);
                    myWindow.showWidget(to_string(4*i+1)+"line_"+to_string(j), WLine(point_in_cw[1], point_in_cw[2], viz::Color::red()), pose);
                    myWindow.showWidget(to_string(4*i+2)+"line_"+to_string(j), WLine(point_in_cw[2], point_in_cw[3], viz::Color::red()), pose);
                    myWindow.showWidget(to_string(4*i+3)+"line_"+to_string(j), WLine(point_in_cw[3], point_in_cw[0], viz::Color::red()), pose);

                    myWindow.getWidget(to_string(4*i)+"line_"+to_string(j)).setRenderingProperty(viz::LINE_WIDTH, 4.0);
                    myWindow.getWidget(to_string(4*i+1)+"line_"+to_string(j)).setRenderingProperty(viz::LINE_WIDTH, 4.0);
                    myWindow.getWidget(to_string(4*i+2)+"line_"+to_string(j)).setRenderingProperty(viz::LINE_WIDTH, 4.0);
                    myWindow.getWidget(to_string(4*i+3)+"line_"+to_string(j)).setRenderingProperty(viz::LINE_WIDTH, 4.0);

                }

                draw_cylinder(ids, j, tvecs, pose, myWindow, secondWindow);
    //            cout<<AffineTransform.matrix<<pose.matrix<<endl<<Pt3D_Camera<<endl<<Pt3D_Origin<<endl;
    //            Pt3D_Camera= AffineTransform * Pt3D_Origin;
    //            cout<<Pt3D_Camera<<endl;
    //            cout<<x<<y<<z<<endl;
                if(trf == 1){
                    waitKey(10000);
                }
                if (ids[0] == 32){
                    trf+=1;
                }
            }

            k[j]+=1;
            
        }
        string s_1 = "Found " + to_string(found_markers) + " markers";
        char const *pchar_1 = s_1.c_str();
        secondWindow.showWidget("Text_1", WText(pchar_1, Point2f(100.0,100.0), 30.0, viz::Color::white()));
        
        img.setImage(imageCopy);
        img.setPose(imagePose);
        myWindow.spinOnce(1, true);
        secondWindow.spinOnce(1, true);
        

    }
//    for ( int i = 0; i <dim1;i++){
//        for ( int j = 0; j <dim2;j++){
//            for ( int z = 0; z <dim3;z++){
//                cout<<dist_1[i][j][z]<<" ";
//            }
//            cout<<endl;
//        }
//
//    }
//    waitKey(100);
    return 0;
}




                
//                viz::WCube cube_widget(point_in_cw[0], Point3f(x+0.219, y+0.178, z+0.1), true, viz::Color::purple());
//                cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
//                myWindow.showWidget("Cube Widget", cube_widget, point_pose);

//auto start = std::chrono::high_resolution_clock::now();
//auto finish = std::chrono::high_resolution_clock::now();
//   chrono::duration<double> elapsed = finish - start;
//   cout<<elapsed.count();



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
//    pointProject = rot_mat * Mat(x,y,z);
//    Vec3d g = Vec3d(pointProject.at<double>(0, 0), pointProject.at<double>(0, 1), pointProject.at<double>(0, 2))
//    Point3d h = tvec + g;
//    return h;
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




//#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/core.hpp>
//
//#include <iostream>
//#include <fstream>
//
//using namespace std;
//using namespace cv;
//using namespace cv::sfm;
//
//static void help() {
//    cout
//    << "\n------------------------------------------------------------------------------------\n"
//    << " This program shows the multiview reconstruction capabilities in the \n"
//    << " OpenCV Structure From Motion (SFM) module.\n"
//    << " It reconstruct a scene from a set of 2D images \n"
//    << " Usage:\n"
//    << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
//    << " where: path_to_file is the file absolute path into your system which contains\n"
//    << "        the list of images to use for reconstruction. \n"
//    << "        f  is the focal length in pixels. \n"
//    << "        cx is the image principal point x coordinates in pixels. \n"
//    << "        cy is the image principal point y coordinates in pixels. \n"
//    << "------------------------------------------------------------------------------------\n\n"
//    << endl;
//}
//
//
//static int getdir(const string _filename, vector<String> &files)
//{
//    ifstream myfile(_filename.c_str());
//    if (!myfile.is_open()) {
//        cout << "Unable to read file: " << _filename << endl;
//        exit(0);
//    } else {;
//        size_t found = _filename.find_last_of("/\\");
//        string line_str, path_to_file = _filename.substr(0, found);
//        while ( getline(myfile, line_str) )
//            files.push_back(path_to_file+string("/")+line_str);
//    }
//    return 1;
//}
//
//
//int main(int argc, char* argv[])
//{
//    // Read input parameters
//
//
//    // Parse the image paths
//
//    vector<String> images_paths;
//    getdir("/Users/grashin/video_detection/webcam_calibrate", images_paths);
//
//
//    // Build intrinsics
//
//    float fx = 971.09050398,
//    fy = 968.24582919,
//    cx = 643.07484029,
//    cy = 356.36661208;
//
//    Matx33d K = Matx33d( fx, 0, cx,
//                        0, fy, cy,
//                        0, 0,  1);
//
//
//    /// Reconstruct the scene using the 2d images
//
//    bool is_projective = true;
//    vector<Mat> Rs_est, ts_est, points3d_estimated;
//
//    reconstruct(images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);
//
//
//    // Print output
//
//    cout << "\n----------------------------\n" << endl;
//    cout << "Reconstruction: " << endl;
//    cout << "============================" << endl;
//    cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
//    cout << "Estimated cameras: " << Rs_est.size() << endl;
//    cout << "Refined intrinsics: " << endl << K << endl << endl;
//    cout << "3D Visualization: " << endl;
//    cout << "============================" << endl;
//
//
//    /// Create 3D windows
//
//    viz::Viz3d window("Coordinate Frame");
//    window.setWindowSize(Size(500,500));
//    window.setWindowPosition(Point(150,150));
//    window.setBackgroundColor(); // black by default
//
//    // Create the pointcloud
//    cout << "Recovering points  ... ";
//
//    // recover estimated points3d
//    vector<Vec3f> point_cloud_est;
//    for (int i = 0; i < points3d_estimated.size(); ++i)
//        point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
//
//    cout << "[DONE]" << endl;
//
//
//    /// Recovering cameras
//    cout << "Recovering cameras ... ";
//
//    vector<Affine3d> path;
//    for (size_t i = 0; i < Rs_est.size(); ++i)
//        path.push_back(Affine3d(Rs_est[i],ts_est[i]));
//
//    cout << "[DONE]" << endl;
//
//
//    /// Add the pointcloud
//    if ( point_cloud_est.size() > 0 )
//    {
//        cout << "Rendering points   ... ";
//
//        viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
//        window.showWidget("point_cloud", cloud_widget);
//
//        cout << "[DONE]" << endl;
//    }
//    else
//    {
//        cout << "Cannot render points: Empty pointcloud" << endl;
//    }
//
//
//    /// Add cameras
//    if ( path.size() > 0 )
//    {
//        cout << "Rendering Cameras  ... ";
//
//        window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
//        window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
//
//        window.setViewerPose(path[0]);
//
//        cout << "[DONE]" << endl;
//    }
//    else
//    {
//        cout << "Cannot render the cameras: Empty path" << endl;
//    }
//
//    /// Wait for key 'q' to close the window
//    cout << endl << "Press 'q' to close each windows ... " << endl;
//
//    window.spin();
//
//    return 0;
//}
