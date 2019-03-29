#include"opencv2/opencv.hpp"
#include"opencv2/calib3d.hpp"
#ifndef _180925_calib_H_
#define _180925_calib_H_
using namespace cv;
using namespace std;
class CameraCalibrator
{
public:
    CameraCalibrator(String cameraname,Size boardsize=Size(11,8)):flag(0),cameraName(cameraname),
        boardSize(boardsize)
    {


    }
    void takeCalibPicture();
    void takeCalibPicture_ZED();
    void getcalibFilelist();
    Mat getCameraMatrix(){return cameraMatrix;}
    Mat getCameraDistcoeffs(){return distCoeffs;}
    vector<vector<Point2f>> getImagePTS(){return imagePTS;}
    vector<vector<Point3f>> getobjectPTS(){return objectPTS;}
    Size getBoardSize(){return boardSize;}
    Size getCalibrateImageSize(){return imageSize;}
    int addChessboardPoints();
    virtual double calibrate(); //标定，返回重投影误差
    void setCalibrateFlag(int flag){this->flag=flag;}
    void showAndSaveCalibratedata();//以xml文件格式储存标定数据，内参与畸变矩阵
    void loadCalibratedata();//读取标定数据
    virtual Mat remapImage(Mat &srcImage);//应用标定数据消除畸变
protected:
    vector<vector<Point3f>> objectPTS; /*世界坐标系中的点，坐标系固结在标定板上，xy分别
与标定板的网格对齐,Z轴为标定板平面的法向量,以一个正方形为单位，第一点为（0,0,0）*/
    vector<vector<Point2f>> imagePTS;//图像坐标系中的点，以像素为单位
    Size boardSize; //标定板尺寸，内角点个数nXm
    Mat cameraMatrix;//相机内部参数矩阵
    Mat distCoeffs;//畸变矩阵
    Size imageSize;//标定图像的尺寸大小
    String cameraName;//标定相机名称
    vector<String> filelist;//标定相机拍下的图像文件列表
    int flag;//指定标定方式的标志
    static int nCalibPictures;
};
class stereoCalibrator:public CameraCalibrator
{
public:
    stereoCalibrator(String cameraname,Size boardsize=Size(11,8)):CameraCalibrator(cameraname,boardsize)
    {}
    void initCalibdata();//在calibrate内部调用
    virtual double calibrate();//多态函数,复用calibrate函数
    void computeCalibMap();//计算矫正映射
    Mat getQMatirx(){return Q;}//返回带深度信息的Q矩阵
    void getLeftMap(Mat &leftMap1,Mat &leftMap2){leftMap1=leftMap_1.clone();leftMap2=leftMap_2.clone();}
    void getRightMap(Mat &rightMap1,Mat &rightMap2){rightMap1=rightMap_1.clone();rightMap2=rightMap_2.clone();}
protected:
    vector<vector<Point2f>> leftImagePTS,rightImagePTS;
    Mat R,T;//保证光轴平行以及像平面共面的旋转矩阵和平移矩阵
    Mat cameraMTX_l,cameraMTX_r;
    Mat distCoeffs_l,distCoeffs_r;
    Size calibrateImageSize;
    Mat leftMap_1,leftMap_2;
    Mat rightMap_1,rightMap_2;
    Mat newProjectionMatrix_left,newProjectionMatrix_right;//立体标定后3x4的左右相机矩阵
    Mat rectedRotion_left,rectedRotion_right; //立体标定旋转矩阵，使得左右像素行对齐
    Mat Q;//含有深度信息的矩阵
};
#endif // 180925_相机标定_H

