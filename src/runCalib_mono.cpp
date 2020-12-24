#include"../include/180925_calibCamera.h"
#define HAVE_PICTURE 1
#define HAVE_XML 1
#define HAVE_CALIB 1
/**************************************测试单目*********************/
int main(int argc,char** argv)
{
  string cameraname="MC500UC";
  CameraCalibrator calibrator(cameraname,Size(11,8));
  calibrator.setCamera(0, 800, 600);
  string path = "D:/CODEing/ComputerVision_codeSources/CameraCalibrate/MagiChanCamera MC500UC_2592x1944/";
  calibrator.setPath(path);
  #if !HAVE_PICTURE
  //calibrator.takeCalibPicture();
  calibrator.getCalibPictureFromDir();
  #endif
  #if !HAVE_CALIB
  calibrator.calibrate();
  #endif
  #if !HAVE_XML
  calibrator.showAndSaveCalibratedata();
  #endif
  calibrator.loadCalibratedata();
/*
VideoCapture cap(0);
cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
cap.set(CV_CAP_PROP_FRAME_HEIGHT,600);

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
*/


String filename = path + format("calib_%d.bmp", 1);
Mat frame = imread(filename);
Mat temp;
resize(frame, temp, Size(800, 600));
imshow("srcFrame", temp);
Mat frame_undistort = calibrator.remapImage(frame);
resize(frame_undistort, temp, Size(800, 600));
imshow("undistort_img", temp);
waitKey(0);

return system("pause");

}
