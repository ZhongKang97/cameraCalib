#include"opencv2/opencv.hpp"
#include"opencv2/calib3d.hpp"
#ifndef _180925_calib_H_
#define _180925_calib_H_
using namespace cv;
using namespace std;
class CameraCalibrator
{
public:
	CameraCalibrator(String cameraname, Size boardsize = Size(11, 8)) :flag(0), cameraName(cameraname),
		boardSize(boardsize)
	{

	}
	void setCamera(int index=0,int img_width=640,int img_height=480) { 
		cap.open(index);
		cap.set(cv::CAP_PROP_FRAME_WIDTH, img_width);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, img_height);
	}
	void setPath(string path) { this->path = path; }
	void takeCalibPicture();
	void takeCalibPicture_ZED();
	void getcalibFilelist();
	void getCalibPictureFromDir();
	Mat getCameraMatrix() { return cameraMatrix; }
	Mat getCameraDistcoeffs() { return distCoeffs; }
	vector<vector<Point2f>> getImagePTS() { return imagePTS; }
	vector<vector<Point3f>> getobjectPTS() { return objectPTS; }
	Size getBoardSize() { return boardSize; }
	Size getCalibrateImageSize() { return imageSize; }
	int addChessboardPoints();
	virtual double calibrate(); //�궨��������ͶӰ���
	void setCalibrateFlag(int flag) { this->flag = flag; }
	void showAndSaveCalibratedata();//��xml�ļ���ʽ����궨���ݣ��ڲ���������
	void loadCalibratedata();//��ȡ�궨����
	Mat remapImage(Mat &srcImage);//Ӧ�ñ궨������������
protected:
	vector<vector<Point3f>> objectPTS; /*��������ϵ�еĵ㣬����ϵ�̽��ڱ궨���ϣ�xy�ֱ�
									   ��궨����������,Z��Ϊ�궨��ƽ��ķ�����,��һ��������Ϊ��λ����һ��Ϊ��0,0,0��*/
	vector<vector<Point2f>> imagePTS;//ͼ������ϵ�еĵ㣬������Ϊ��λ
	Size boardSize; //�궨��ߴ磬�ڽǵ����nXm
	Mat cameraMatrix;//����ڲ���������
	Mat distCoeffs;//�������
	Size imageSize;//�궨ͼ��ĳߴ��С
	String cameraName;//�궨�������
	vector<String> filelist;//�궨������µ�ͼ���ļ��б�
	int flag;//ָ���궨��ʽ�ı�־
	static int nCalibPictures;
	VideoCapture cap;
	String path;
};
class stereoCalibrator :public CameraCalibrator
{
public:
	stereoCalibrator(String cameraname, Size boardsize = Size(11, 8)) :CameraCalibrator(cameraname, boardsize)
	{}
	void initCalibdata();//��calibrate�ڲ�����
	virtual double calibrate();//��̬����,����calibrate����
	void computeCalibMap();//�������ӳ��
	Mat getQMatirx() { return Q; }//���ش������Ϣ��Q����
	void getLeftMap(Mat &leftMap1, Mat &leftMap2) { leftMap1 = leftMap_1.clone(); leftMap2 = leftMap_2.clone(); }
	void getRightMap(Mat &rightMap1, Mat &rightMap2) { rightMap1 = rightMap_1.clone(); rightMap2 = rightMap_2.clone(); }
protected:
	vector<vector<Point2f>> leftImagePTS, rightImagePTS;
	Mat R, T;//��֤����ƽ���Լ���ƽ�湲�����ת�����ƽ�ƾ���
	Mat cameraMTX_l, cameraMTX_r;
	Mat distCoeffs_l, distCoeffs_r;
	Size calibrateImageSize;
	Mat leftMap_1, leftMap_2;
	Mat rightMap_1, rightMap_2;
	Mat newProjectionMatrix_left, newProjectionMatrix_right;//����궨��3x4�������������
	Mat rectedRotion_left, rectedRotion_right; //����궨��ת����ʹ�����������ж���
	Mat Q;//���������Ϣ�ľ���
};
#endif // 180925_����궨_H

