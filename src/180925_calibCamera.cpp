#include"../include/180925_calibCamera.h"

int CameraCalibrator::nCalibPictures=15;
void CameraCalibrator::takeCalibPicture_ZED()
{
    
    int counter=0;
    cap.open(1);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if(cap.isOpened())
    {
        Mat frame;
	String path="/home/nvidia/DISK/CODEing/OpenCV_codeSources/CameraCalibrate/"+cameraName+"/";
        cout<<"目录："<<path<<endl;
        while (true)
        {
            cap>>frame;
            if(frame.empty()) break;
            Mat leftImage,rightImage;
            leftImage = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            rightImage = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
            Mat tempImage_l,tempImage_r;
            tempImage_l=leftImage.clone();
            tempImage_r=rightImage.clone();
            vector<Point2f> imageCorners_l,imageCorners_r;
            bool found_l = findChessboardCorners(leftImage,boardSize,imageCorners_l);
            bool found_r = findChessboardCorners(rightImage,boardSize,imageCorners_r);
            drawChessboardCorners(leftImage,boardSize,imageCorners_l,found_l);
            drawChessboardCorners(rightImage,boardSize,imageCorners_r,found_r);
            imshow("发现的角点_left",leftImage);
            imshow("发现的角点_right",rightImage);
            char c=waitKey(1);
            if(c==27 || counter>=nCalibPictures) break;
            else if(c=='p' && found_l && found_r)
                if(waitKey(0)=='s')
                {
                    cout<<"正在纪录第"<<counter<<"张标定图"<<endl;
                    const String filename_l=path+cameraName+format("_left_calibPicture_number%d.jpg",counter);
                    const String filename_r=path+cameraName+format("_right_calibPicture_number%d.jpg",counter);
                    counter++;
                    cout<<"FIELNAEM_l："<<filename_l<<endl;
                    cout<<"FIELNAEM_r："<<filename_r<<endl;
                    imwrite(filename_l,tempImage_l);
                    imwrite(filename_r,tempImage_r);
                    cout<<"纪录完毕，图片存放在目录："<<path<<endl;
                }
        }
        cap.release();
    }
}
Mat CameraCalibrator::remapImage(Mat &srcImage)
{
    Mat undistorted;
    Mat map1,map2;
    initUndistortRectifyMap(cameraMatrix,   //计算映射关系
                            distCoeffs,
                            Mat(),//可选矫正项
                            Mat(),//生成无畸变图像的相机矩阵
                            srcImage.size(),//矫正后图片的尺寸
                            CV_32FC1,//输出图像的类型
                            map1,map2);//映射关系
    remap(srcImage,undistorted,map1,map2, //实施映射
          INTER_LINEAR);//差值类型
    return undistorted;
}
void CameraCalibrator::loadCalibratedata()
{
  String path="/home/nvidia/DISK/CODEing/OpenCV_codeSources/CameraCalibrate/"+cameraName+"/";
    String filename=path+cameraName+format("_calibMatrixs.xml");
    FileStorage file;
    file.open(filename,FileStorage::READ);
    file["CameraMatrix"]>>cameraMatrix;
    file["distCoeffs"]>>distCoeffs;
    file["calibrateImageSize"]>>imageSize;
    file["objectPTS"]>>objectPTS;
    file["imagePTS"]>>imagePTS;
    file.release();
    cout<<"读取完毕！"<<endl;
    cout<<"标定图像尺寸："<<imageSize;
    cout<<"内参矩阵："<<cameraMatrix<<endl;
    cout<<"畸变矩阵："<<distCoeffs<<endl;

}
void CameraCalibrator::showAndSaveCalibratedata()
{
    cout<<"内参矩阵："<<cameraMatrix<<endl;
    cout<<"畸变矩阵："<<distCoeffs<<endl;
    FileStorage file;
    String path="/home/nvidia/DISK/CODEing/OpenCV_codeSources/CameraCalibrate/"+cameraName+"/";
    String filename=path+cameraName+format("_calibMatrixs.xml");
    file.open(filename,FileStorage::WRITE);
    file<<"CameraMatrix"<<cameraMatrix;
    file<<"distCoeffs"<<distCoeffs;
    file<<"calibrateImageSize"<<imageSize;
    file<<"objectPTS"<<objectPTS;
    file<<"imagePTS"<<imagePTS;
    cout<<"xml文件："<<filename<<endl;
    file.release();
}
void CameraCalibrator::getcalibFilelist()
{
  String path="/home/nvidia/DISK/CODEing/OpenCV_codeSources/CameraCalibrate/"+cameraName+"/";
    for(int counter=0;counter<nCalibPictures;counter++)
    {
        const String filename=path+cameraName+format("_calibPicture_number%d.jpg",counter);
        cout<<"fetching picture:"<<filename<<endl;
        filelist.push_back(filename);
    }
    Mat imag=imread(filelist[0]);
    imageSize=Size(imag.cols,imag.rows);
}
int CameraCalibrator::addChessboardPoints()
{
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;
    for(int i=0;i<boardSize.height;i++)
        for(int j=0;j<boardSize.width;j++)
            objectCorners.push_back(Point3f(i*150,j*150,0.0f)); /*场景中的三维点，初始化棋盘中的角点
    这里我用的标定板每一个小矩形边长为15mm，故这里设置了现实世界的距离单位为mm*/
    Mat calibImage; //用于储存棋盘图像
    int counter=0;
    for(int i=0;i<filelist.size();i++)
    {
        calibImage=imread(filelist[i],0);
        bool found=findChessboardCorners(calibImage, //棋盘图像
                                         boardSize,  //棋盘尺寸，内角点数
                                         imageCorners);//检测到的角点列表
        if(found)
        {
            cornerSubPix(calibImage,imageCorners,
                         Size(5,5),
                         Size(-1,-1),
                         TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS,
                                      30, //最大迭代次数
                                      0.1));//最小精度
            if(imageCorners.size()==boardSize.area())
            {
                objectPTS.push_back(objectCorners);
                imagePTS.push_back(imageCorners);
                counter++;
            }
        }
    }
    return counter;
}
double CameraCalibrator::calibrate()
{
    getcalibFilelist();
    addChessboardPoints();
    vector<Mat> rvecs,tvecs; //每幅图像从自身坐标系变换到相机坐标系对应的旋转量和平移量
    return calibrateCamera(objectPTS,imagePTS,//对应的三维二维点
                           imageSize, //标定图像的尺寸
                           cameraMatrix,//输出相机内部参数
                           distCoeffs,//输出畸变矩阵
                           rvecs,//输出的旋转量矩阵nx3x3
                           tvecs,//输出的平移矩阵nx3x1
                           flag);//设置选项，默认为0
}
void CameraCalibrator::takeCalibPicture()
{
    int counter=0;
    cap.open(1);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if(cap.isOpened())
    {
        Mat frame;

	String path="/home/nvidia/DISK/CODEing/OpenCV_codeSources/CameraCalibrate/"+cameraName+"/";
        cout<<"目录："<<path<<endl;
        while (true)
        {
            cap>>frame;
            Mat tempImage=frame.clone();
            if(frame.empty()) break;
            vector<Point2f> imageCorners;

            bool found = findChessboardCorners(frame,boardSize,imageCorners);
            drawChessboardCorners(frame,boardSize,imageCorners,found);
            imshow("corners",frame);
            char c=waitKey(1);
            if(c==27 || counter>=nCalibPictures) break;
            else if(c=='p' && found)
                if(waitKey(0)=='s')
                {
                    cout<<"正在纪录第"<<counter<<"张标定图"<<endl;
                    const String filename=path+cameraName+format("_calibPicture_number%d.jpg",counter++);
                    cout<<"FIELNAEM："<<filename<<endl;
                    imwrite(filename,tempImage);
                    cout<<"纪录完毕，图片存放在目录："<<path<<endl;
                }
        }
        cap.release();
    }
}
void stereoCalibrator::initCalibdata()
{
    string camera_l=cameraName+"_left",camera_r=cameraName+"_right";
    CameraCalibrator brator_l(camera_l,boardSize),brator_r(camera_r,boardSize);
    brator_l.loadCalibratedata();
    brator_r.loadCalibratedata();
    calibrateImageSize=brator_l.getCalibrateImageSize();
    cameraMTX_l=brator_l.getCameraMatrix();
    cameraMTX_r=brator_r.getCameraMatrix();
    distCoeffs_l=brator_l.getCameraDistcoeffs();
    distCoeffs_r=brator_r.getCameraDistcoeffs();
    objectPTS=brator_l.getobjectPTS();
    leftImagePTS=brator_l.getImagePTS();
    rightImagePTS=brator_r.getImagePTS();
}
double stereoCalibrator::calibrate()
{
    cout<<"Now stereoCalibrating"<<endl;
    initCalibdata();
    stereoCalibrate(objectPTS,leftImagePTS,rightImagePTS,
                    cameraMTX_l,distCoeffs_l,
                    cameraMTX_r,distCoeffs_r,
                    calibrateImageSize,
                    R,T,
                    noArray(),noArray()//不计算E，F矩阵
                    );
    stereoRectify(cameraMTX_l,distCoeffs_l, //计算行对齐参数
                  cameraMTX_r,distCoeffs_r,
                  calibrateImageSize,
                  R,
                  T,
                  rectedRotion_left,
                  rectedRotion_right,
                  newProjectionMatrix_left,
                  newProjectionMatrix_right,
                  Q);
    cout<<"translation"<<T<<endl;
}
void stereoCalibrator::computeCalibMap()
{
    initUndistortRectifyMap(cameraMTX_l,
                            distCoeffs_l,
                            rectedRotion_left,
                            newProjectionMatrix_left,
                            calibrateImageSize,
                            CV_32FC1,                                                                                                        
                            leftMap_1,
                            leftMap_2);//计算左相机矫正
    initUndistortRectifyMap(cameraMTX_r,
                            distCoeffs_r,
                            rectedRotion_right,
                            newProjectionMatrix_right,
                            calibrateImageSize,
                            CV_32FC1,
                            rightMap_1,
                            rightMap_2);//计算左相机矫正
}
/*************************************测试****************************/
/*
int main()
{
    string cameraname="ZED";
    CameraCalibrator calibrator(cameraname,Size(11,8));
#if !HAVE_PICTURE
    calibrator.takeCalibPicture_ZED();
#endif
    CameraCalibrator calibrator_l("ZED_left",Size(11,8)),calibrator_r("ZED_right",Size(11,8));
    calibrator_l.calibrate();
    calibrator_r.calibrate();
#if !HAVE_XML
    calibrator_l.showAndSaveCalibratedata();
    calibrator_r.showAndSaveCalibratedata();
#endif
    calibrator_l.loadCalibratedata();
    calibrator_r.loadCalibratedata();
    stereoCalibrator zed("ZED");
    zed.calibrate();
    zed.computeCalibMap();
    Mat leftMap_1,leftMap_2;
    Mat rightMap_1,rightMap_2;
    zed.getLeftMap(leftMap_1,leftMap_2);
    zed.getRightMap(rightMap_1,rightMap_2);
    VideoCapture cap(0);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 2560);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    Ptr<StereoSGBM> stereo=StereoSGBM::create(
                0,128,11,0,0,
                0,0,0,0,0,
                StereoSGBM::MODE_SGBM_3WAY);
    if(cap.isOpened())
    {
        Mat frame;
        Mat disp;
        Mat _3DImage;
        viz::Viz3d myWindow("creating weidgt");
        while(1)
        {
            cap>>frame;
            if(frame.empty()) break;
            Mat leftImage,rightImage;
            leftImage = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            rightImage = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));
            imshow("没矫正前",frame);
            //leftImage=calibrator_l.remapImage(leftImage);
            //rightImage=calibrator_r.remapImage(rightImage);
          //  imshow("单目矫正后_l",leftImage);
            //imshow("单目矫正后_r",rightImage);
            remap(leftImage,leftImage,leftMap_1,leftMap_2,INTER_LINEAR);
            remap(rightImage,rightImage,rightMap_1,rightMap_2,INTER_LINEAR);
            imshow("立体矫正后—l",leftImage);
            imshow("立体矫正后—r",rightImage);
            stereo->compute(leftImage,rightImage,disp);
            cv::normalize(disp,disp,0,256,cv::NORM_MINMAX,CV_8U);
            reprojectImageTo3D(disp,_3DImage,zed.getQMatirx());
            imshow("dis",disp);
            cout<<"P(255.255)"<<_3DImage.at<Vec3f>(320,300)<<endl;
            if(waitKey(10)==27) break;
        }
    }
    return 0;
}
*/
