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