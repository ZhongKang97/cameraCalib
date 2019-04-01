#include"../include/180925_calibCamera.h"
#define HAVE_PICTURE 0
#define HAVE_XML 0
#define HAVE_CALIB 0
/**************************************测试单目*********************/
int main(int argc,char** argv)
{
  string cameraname="C930E_640x480";
  CameraCalibrator calibrator(cameraname,Size(11,8));
  string path = "D:/CODEing/OpenCV_codeSources/CameraCalibrate/C930E_640x480/";
  calibrator.setPath(path);
  #if !HAVE_PICTURE
  calibrator.takeCalibPicture();
  #endif
  #if !HAVE_CALIB
  calibrator.calibrate();
  #endif
  #if !HAVE_XML
  calibrator.showAndSaveCalibratedata();
  #endif
  calibrator.loadCalibratedata();
  
  VideoCapture cap(1);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  
  if(cap.isOpened())
  {
	  cout << "cap is opened" << endl;
    cv::Mat frame;
    while(1)
    {
      cap>>frame;
      if(frame.empty()) break;
      imshow("srcImage",frame);
      Mat frame_undistort = calibrator.remapImage( frame );
      imshow("undistort_img",frame_undistort);
      char c= waitKey(0);
      if(c==27) break;
    }
  }
  return system("pause");
}
 /*************************************测试双目****************************/
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