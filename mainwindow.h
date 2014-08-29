#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QSlider>
#include <QLabel>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/flann/flann.hpp>

#include <iostream>
#include <stdio.h>
#include <cstdio>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

// Point Cloud Filter
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/point_types.h>
//#include <flann/flann.h>

using namespace cv;
using namespace std;

#define DETECTION  0
#define CAPTURING  1
#define CALIBRATED 2

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();
    
private slots:

    void on_openImage_clicked();

    void  on_saveImagePair_clicked();

    void on_enableStereoCamera_toggled();

    void on_disableStereoCamera_clicked();

    void stereoCalibration(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true);

    bool readStringList(const string& filename, vector<string>& l);

    int on_loadandStereoCalibration_clicked();

    int on_captureImagePairs_clicked();

    void on_sadWindowSizeSlider_valueChanged(int value);

    void on_numOfDisparitiesSlider_valueChanged(int value);

    void on_preFilterCapSlider_valueChanged(int value);

    void on_minDisparitySlider_valueChanged(int value);

    void on_uniquenessRatioSlider_valueChanged(int value);

    void on_speckleWindowSizeSlider_valueChanged(int value);

    void on_speckleRangeSlider_valueChanged(int value);

    void on_disp12MaxDiffSlider_valueChanged(int value);

    void on_preFilterSize_valueChanged(int value);

    void on_pcl_clicked();

    void on_textureThreshold_valueChanged(int value);

    void on_threshold_Slider_valueChanged(int value);

    void creatPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat imgLRectified, cv::Mat disp, double Q03, double Q13, double Q23, double Q32, double Q33);

private:
    Ui::MainWindow *ui;
    cv::Mat src, image, viewL, viewR, viewLGray, viewRGray, frameL, frameR;
    cv::VideoCapture cameraL;
    cv::VideoCapture cameraR;
    cv::Size boardSize;
    bool stopDisplays;
    int cameraID_L;
    int cameraID_R;
    int mode;

    // Variables Used for UI
    int SADWindowSize;
    int numberofDisparities;
    int preFilterCap;
    int preFilterSize;
    int minDisparity;
    int uniquenessRatio;
    int speckleRange;
    int speckleWindowSize;
    int disp12MaxDiff;
    int textureThreshold;
    int thresholdValue;

//    // GUI
//    QLabel *min;
//    QLabel *max;
//    QLabel *currentValue;
//    QSlider *sliderSADWindowSize;
};

#endif // MAINWINDOW_H
