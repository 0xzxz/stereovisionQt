#include "mainwindow.h"
#include "ui_mainwindow.h"

#define maxThresholdValue 255
#define matchingAlgorithm 2
#define cameraID_L 0
#define cameraID_R 1
#define squareSize 22 // set this to your actual square size
#define nframes 14     // the number of images to capture

int point_x, point_y;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    SADWindowSize = 9;
    numberofDisparities = 128;
    preFilterCap = 63;
    preFilterSize = 5;
    minDisparity = 0;
    uniquenessRatio = 16;
    speckleWindowSize = 1;
    speckleRange = 32;
    disp12MaxDiff = 1;
    textureThreshold = 10;
    thresholdValue = 50;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_openImage_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Image"), ".", tr("Image Files (*.png *.jpg *.jpeg *.bmp"));
    src = cv::imread(filename.toAscii().data());
    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Original Image", src);
    cv::Mat dst, dstGray;
    cv::blur(src, dst, cv::Size(3,3), cv::Point(-1,-1), BORDER_DEFAULT);
    cv::imshow("bulrred", dst);
    cv::cvtColor(dst, dstGray, CV_RGB2GRAY);
    cv::imshow("Gray", dstGray);
    while(true)
    {
        if(cv::waitKey(1) == 27)
            break;
    }
    cv::destroyAllWindows();
}

void MainWindow::on_saveImagePair_clicked()
{
    cv::VideoCapture cameraL(0);
    cv::VideoCapture cameraR(1);
    int iFileName = 1;
    char filename[100] = "";
    int skipFrame = 1;
    cv::Mat frameL;
    cv::Mat frameR;

    cv::namedWindow("Left Camera", 1);
    cv::namedWindow("Right Camera", 1);

    std::string savedDir="./imagesCaptured/";

    for(;;)
    {
        std::cout << "Get Ready" << std::endl;
        cameraL >> frameL;
        cameraR >> frameR;
        imshow("Left Camera", frameL);
        imshow("Right Camera", frameR);

        if(cv::waitKey(1) >= 0)
            break;

        if(skipFrame > 50)
        {
            // Need modification: save images to child folder
            sprintf(filename, "%s%s%02d%s", savedDir.c_str(), "imageLeft", iFileName, ".jpg");
            cv::imwrite(filename, frameL);

            sprintf(filename, "%s%s%02d%s", savedDir.c_str(), "imageRight", iFileName, ".jpg");
            cv::imwrite(filename, frameR);

            std::cout << iFileName << " Pairs Captured" << std::endl;

            iFileName++;
            if(iFileName > 10)
                break;
        }

        skipFrame++;
        std::cout<< skipFrame <<std::endl;
    }

    std::cout << "Closing cameras" << std::endl;
    cameraL.release();
    cameraR.release();
    std::cout << "Cameras are closed..." << std::endl;
}

void MainWindow::on_enableStereoCamera_toggled()
{
    cameraL = cv::VideoCapture(cameraID_L);
    cameraR = cv::VideoCapture(cameraID_R);

    std::cout << CV_VERSION << std::endl;

    cv::namedWindow("Left Camera ", 1);
    cv::namedWindow("Right Camera ", 1);
    cv::Mat frameL;
    cv::Mat frameR;

    std::cout << "Make sure the NumLock is off and Press ESC to quit cameras...." << std::endl;
    while(1)
    {
        cameraL >> frameL;
        cameraR >> frameR;
        imshow("Left Camera ", frameL);
        imshow("Right Camera ", frameR);
        if(cv::waitKey(1) == 27)
            break;
    }

    std::cout << "Signal to stop camras ..." <<std::endl;
    std::cout << "Closing cameras" << std::endl;
    cameraL.release();
    cameraR.release();
    std::cout << "Cameras are closed..." << std::endl;
    cv::destroyAllWindows();
    cv::waitKey(0);
}

void MainWindow::on_disableStereoCamera_clicked()
{
    if (cameraL.isOpened() && cameraR.isOpened())
    {
        cameraL.release();
        cameraR.release();
    }
    std::cout << "Disabling Stereo Cameras Done!" << std::endl;
}

int MainWindow::on_captureImagePairs_clicked()
{
    bool undistortImage = false;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    std::vector<std::vector<cv::Point2f> > imagePointsL;
    std::vector<std::vector<cv::Point2f> > imagePointsR;
    cv::Mat viewL, viewR, viewLGray, viewRGray, frameL, frameR;
    //    cameraL.set(CV_CAP_PROP_FPS, 60);
    //    cameraR.set(CV_CAP_PROP_FPS, 10);
    char filename[100] = "";
    std::string savedDir="./";
    int iFileName = 1;
    bool showRectified = true;
    cameraL = cv::VideoCapture(cameraID_L);
    cameraR = cv::VideoCapture(cameraID_R);

    cv::Size boardSize(8,6), imageSize;

    if (!cameraL.isOpened()||!cameraR.isOpened())
    {
        if (!cameraL.isOpened())
            return fprintf(stderr, "Could not initialize video (%d) capture\n", cameraID_L);
        else if(!cameraR.isOpened())
        {
            return fprintf(stderr, "Could not initialize video (%d) capture\n", cameraID_R);
        }
        else
            return fprintf(stderr, "Could not initialize both videos capture\n");
    }

    cv::namedWindow("Left Camera");
    cv::namedWindow("Right Camera");

    for(int i = 0; ; i++)
    {
        cameraL >> frameL;
        cameraR >> frameR;
        bool blink = false;

        frameL.copyTo(viewL);
        frameR.copyTo(viewR);

        // output vectors of image points
        std::vector<cv::Point2f> imageCornersL;
        std::vector<cv::Point2f> imageCornersR;
        imageSize = viewL.size();

        cv::cvtColor(viewL, viewLGray, CV_BGR2GRAY);
        cv::cvtColor(viewR, viewRGray, CV_BGR2GRAY);

        // Get the chessboard corners
        bool foundL = cv::findChessboardCorners(viewL, boardSize, imageCornersL, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        bool foundR = cv::findChessboardCorners(viewR, boardSize, imageCornersR, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (mode == CAPTURING && foundL && foundR && clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC)
        {
            imagePointsL.push_back(imageCornersL);
            imagePointsR.push_back(imageCornersR);
            prevTimestamp = clock();
            blink = true;

            sprintf(filename, "%s%s%02d%s", savedDir.c_str(), "imageLeft", iFileName, ".jpg");
            cv::imwrite(filename, viewL);

            sprintf(filename, "%s%s%02d%s", savedDir.c_str(), "imageRight", iFileName, ".jpg");
            cv::imwrite(filename, viewR);

            iFileName ++;
        }

        if (foundL && foundR)
        {
            cv::cornerSubPix(viewLGray, imageCornersL, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
            cv::cornerSubPix(viewRGray, imageCornersR, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(viewL, boardSize, Mat(imageCornersL), foundL);
            cv::drawChessboardCorners(viewR, boardSize, Mat(imageCornersR), foundR);
        }

        string msg = mode == CAPTURING ? "100/100" : mode == CAPTURING ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        cv::Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        cv::Point textOrigin(viewL.cols - 2*textSize.width - 10, viewL.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(undistortImage)
                msg = format( "%d/%d Undist", (int)imagePointsL.size(), nframes );
            else
                msg = format( "%d/%d", (int)imagePointsL.size(), nframes );
        }

        cv::putText( viewL, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));
        cv::putText( viewR, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if (blink)
        {
            cv::bitwise_not(viewL, viewL);
            cv::bitwise_not(viewR, viewR);
        }

        cv::imshow("Left Camera", viewL);
        cv::imshow("Right Camera", viewR);

        int key = 0xff & cv::waitKey(1);

        if ((key & 255) == 27 )
            break;

        if ( key == 'u' && mode == CALIBRATED )
            undistortImage = !undistortImage;

        if ( key == 'g' && cameraL.isOpened() && cameraR.isOpened() )
        {
            mode = CAPTURING;
            imagePointsL.clear();
            imagePointsR.clear();
        }

        if ( mode == CAPTURING && imagePointsL.size() >= (unsigned)nframes)
        {
            mode = CALIBRATED;
        }

        if (mode == CALIBRATED)
            break;

        if (cv::waitKey(1) == 27)
            break;
    }
    std::cout << "Out of capturing ..." << std::endl;
    cameraL.release();
    cameraR.release();
    cv::destroyAllWindows();
    cv::waitKey(1);

    // READING IMAGE PAIRS CAPTURED
    std::cout << "Get ready for stereo calibration...." << std::endl;
    string imagelistfn = "stereo_calib.xml";
    std::vector<string> imagelist;
    bool useCalibrated = true;
    bool ok = MainWindow::readStringList( imagelistfn.c_str(), imagelist);
    if ( !ok || imagelist.empty())
        std::cout << "cannot open" << imagelistfn << "or the string list is empty" << std::endl;
    // CALIBRATION
    MainWindow::stereoCalibration(imagelist, boardSize, useCalibrated, showRectified);
    std::cout << "Out of this module...." << std::endl;
    cv::destroyAllWindows();
    cv::waitKey(1);
    return 0;
}

cv::Point currentCoordinate;
int prt_x, prt_y;
bool captureFrame;
bool moveWindowFlag = true;

void onMouse(int event, int x, int y, int flag, void*)
{
    if(event == CV_EVENT_LBUTTONDOWN)
    {
        currentCoordinate = cv::Point(x,y);
        prt_x = x;
        prt_y = y;
        moveWindowFlag = false;
    }
    if(event == CV_EVENT_RBUTTONDOWN)
    {
        captureFrame = true;
    }
}

int MainWindow::on_loadandStereoCalibration_clicked()
{
    std::cout << "Get in...." << std::endl;
    cv::Mat viewL, viewR, viewLGray, viewRGray, frameL, frameR;

    std::string saveDisp8Dir = "/home/zhixian/Dropbox/AIMLab/Qt/martinPeris/OpenCVReprojectImageToPointCloud/capturedDisp.jpg";
    std::string saveRGBDir = "/home/zhixian/Dropbox/AIMLab/Qt/martinPeris/OpenCVReprojectImageToPointCloud/capturedRGB.jpg";
    std::string save3DDir = "/home/zhixian/Dropbox/AIMLab/Qt/martinPeris/OpenCVReprojectImageToPointCloud/captured3D.jpg";

    bool useCalibrated = true;
    bool showRectified = false;
    boardSize = cv::Size(8,6);

    // READING IMAGE PAIRS CAPTURED
    std::cout << "Get ready for stereo calibration...." << std::endl;
    string imagelistfn = "stereo_calib.xml";
    std::vector<string> imagelist;
    bool ok = MainWindow::readStringList( imagelistfn.c_str(), imagelist);
    if ( !ok || imagelist.empty())
        std::cout << "cannot open" << imagelistfn << "or the string list is empty" << std::endl;
    // CALIBRATION
    MainWindow::stereoCalibration(imagelist, boardSize, useCalibrated, showRectified);

    // GET Q MATRIX FOR DEPTH MAP
    cv::Mat disp, disp8;
    cv::Mat image3D;
    disp = cv::Mat::zeros(640, 480, CV_16UC1);
    //    image3D = cv::Mat(viewL.rows, viewL.cols, CV_32FC3);

    cv::Mat M1, D1, M2, D2, R, T, R1, P1, R2, P2, Q;
    cv::FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open("extrinsics.yml", CV_STORAGE_READ);
    if( !fs.isOpened() )
        std::cout << "Failed to open extrinsics file" << std::endl;
    fs["R"]  >> R;
    fs["T"]  >> T;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"]  >> Q;

    //    fs.open("Q.xml", CV_STORAGE_READ);
    //    fs["Q"] >> Q;

    cv::Rect roi1, roi2;
    cv::Size img_size;
    cv::Mat map11, map12, map21, map22, imgLRectified, imgRRectified;

    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);

    bool labelDisplay = true;

    cameraL = cv::VideoCapture(cameraID_L);
    cameraR = cv::VideoCapture(cameraID_R);

    if (!cameraL.isOpened()||!cameraR.isOpened())
    {
        if (!cameraL.isOpened())
            fprintf(stderr, "Could not initialize video (%d) capture\n", cameraID_L);
        else if(!cameraR.isOpened())
        {
            fprintf(stderr, "Could not initialize video (%d) capture\n", cameraID_R);
        }
        else
            fprintf(stderr, "Could not initialize both videos capture\n");
    }

    int saveCount = 1;
    bool initFlag = true;
    cv::Mat imageSaved, nearest;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // Real Time Recification
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imgLRectifiedGray, imgRRectifiedGray;
    cv::Rect validRoi[2];
    bool isVerticalStereo;
    Mat canvas;
    double sf;
    int w, h, j;
    double scale = 50;
    int viewport = 0;
    viewer->addCoordinateSystem(scale, viewport);

    while (1)
    {
        cameraL >> frameL;
        cameraR >> frameR;

        cv::imshow("Left Camera", frameR);
        cv::imshow("Right Camera", frameL);

        //********************** Image Rectification *********************************//
        if (initFlag)
        {
            img_size = frameL.size();
            initFlag = false;
            prt_x = img_size.width/2;
            prt_y = img_size.height/2;
            cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &validRoi[0], &validRoi[1]);
            cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
            isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
            if( !isVerticalStereo )
            {
                sf = 600./MAX(img_size.width, img_size.height);
                w = cvRound(img_size.width*sf);
                h = cvRound(img_size.height*sf);
                canvas.create(h, w*2, CV_8UC3);
            }
            else
            {
                sf = 300./MAX(img_size.width, img_size.height);
                w = cvRound(img_size.width*sf);
                h = cvRound(img_size.height*sf);
                canvas.create(h*2, w, CV_8UC3);
            }
        }

        cv::remap(frameL, imgLRectified, map11, map12, INTER_LINEAR);
        cv::remap(frameR, imgRRectified, map21, map22, INTER_LINEAR);
        //cv::cvtColor(imgLRectified, imgLRectifiedGray, CV_GRAY2BGR);
        //cv::cvtColor(imgRRectified, imgRRectifiedGray, CV_GRAY2BGR);
        cv::Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*0, 0, w, h)) : canvas(Rect(0, h*0, w, h));
        cv::resize(imgLRectified, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
        if( useCalibrated )
        {
            Rect vroi(cvRound(validRoi[0].x*sf), cvRound(validRoi[0].y*sf),
                      cvRound(validRoi[0].width*sf), cvRound(validRoi[0].height*sf));
            rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
        }
        canvasPart = !isVerticalStereo ? canvas(Rect(w*1, 0, w, h)) : canvas(Rect(0, h*1, w, h));
        cv::resize(imgRRectified, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
        if( useCalibrated )
        {
            Rect vroi(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf),
                      cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));
            rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        cv::imshow("Rectified Pair", canvas);

        //********************************** End of Rectification *********************************//

        switch(matchingAlgorithm)
        {
        case 1:
        {
            cv::StereoBM sbm;
            sbm.state->roi1 = roi1;
            sbm.state->roi2 = roi2;
            sbm.state->SADWindowSize = 5;
            sbm.state->numberOfDisparities = numberofDisparities;
            sbm.state->preFilterSize = preFilterSize;
            sbm.state->preFilterCap = 61;
            sbm.state->minDisparity = minDisparity;
            sbm.state->textureThreshold = textureThreshold;
            sbm.state->uniquenessRatio = uniquenessRatio;
            sbm.state->speckleWindowSize = speckleWindowSize;
            sbm.state->speckleRange = speckleRange;
            sbm.state->disp12MaxDiff = disp12MaxDiff;
            cv::cvtColor(imgLRectified, viewLGray, CV_BGR2GRAY);
            cv::cvtColor(imgRRectified, viewRGray, CV_BGR2GRAY);
            sbm(viewLGray, viewRGray, disp);
            cv::normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
            if (labelDisplay)
            {
                std::cout << "Using Blocking Matching...." << std::endl;
                labelDisplay = false;
            }
            break;
        }
        case 2:
        {
            cv::StereoSGBM sgbm;
            sgbm.SADWindowSize = SADWindowSize;
            sgbm.numberOfDisparities = numberofDisparities;
            sgbm.preFilterCap = preFilterCap;
            sgbm.minDisparity = minDisparity;
            sgbm.uniquenessRatio = uniquenessRatio;
            sgbm.speckleWindowSize = speckleWindowSize;
            sgbm.speckleRange = speckleRange;
            sgbm.disp12MaxDiff = disp12MaxDiff;
            sgbm.fullDP = false;
            int cn = viewLGray.channels();
            sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
            sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
            sgbm(imgRRectified, imgLRectified, disp);
            cv::normalize(disp, disp8, 0, 200, CV_MINMAX, CV_8U);
            if(labelDisplay)
            {
                std::cout << "Using Semi-glboal Blocking Matching..." << std::endl;
                labelDisplay = false;
            }
            break;
        }
        default:
            std::cout << "No algorithm for stereo correspondence..." << std::endl;
            return -1;
        }

        cv::reprojectImageTo3D(disp, image3D, Q, true, CV_32F); // CV_32F depth
        Point3f img3DPoint = image3D.at<Point3f>(prt_y, prt_x);
        float depth = 16*img3DPoint.z/1000;
        string depthmsg = format("Current Distance is %4.3f m", depth);
        if (cv::waitKey(5) == 27)
            break;
        cv::circle(imgLRectified, cv::Point(prt_x,prt_y), 10, cv::Scalar(0, 255, 0), 1, 1);
        cv::imshow("Disparity", disp8);

        // ************************* Capture a frame *********************************//
        // ***************************************************************************//
        if (captureFrame == true)
        {
            cv::imwrite(saveDisp8Dir, disp8);
            cv::imwrite(saveRGBDir, imgLRectified);
            cv::imwrite(save3DDir, image3D);
            captureFrame = false;
        }

        cv::putText(imgLRectified, depthmsg, cv::Point(400, 450), 1, 1, cv::Scalar(0,255,0));
        cv::imshow("Distance Measure", imgLRectified);
        cv::setMouseCallback("Distance Measure", onMouse, 0);

        if (moveWindowFlag)
        {
            cvMoveWindow("Rectified Pair", 10, 10);
            cvMoveWindow("Disparity", 700, 600 );
            cvMoveWindow("Distance Measure", 10, 600);
            cvMoveWindow("Edge", 1400, 600);
        }

        //***************************** PCL ***********************************//
        //*********************************************************************//
        //creatPointCloud(viewer, imgRRectified, disp8, Q03, Q13, Q23, Q32, Q33);
    }
    cameraL.release();
    cameraR.release();
    cv::destroyAllWindows();
    viewer->close();
    cv::waitKey(5);
    std::cout << "Out of Generating Depth Map ..." << std::endl;
    return 0;
}

void MainWindow::creatPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat imgLRectified, cv::Mat disp, double Q03, double Q13, double Q23, double Q32, double Q33)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    double px, py, pz;
    uchar pr, pg, pb;
    viewer->removePointCloud("point cloud");
    for (int i = 0; i < imgLRectified.rows; i++)
    {
        uchar* rgb_ptr = imgLRectified.ptr<uchar>(i);
        uchar* disp_ptr = disp.ptr<uchar>(i);
        for (int j = 0; j < imgLRectified.cols; j++)
        {
            uchar d = disp_ptr[j];
            if (d == 0) continue;
            double pw = 1.0 * static_cast<double>(d) * Q32 + Q33;
            px = static_cast<double>(j) + Q03;
            py = static_cast<double>(i) + Q13;
            pz = Q23;

            px = px/pw;
            py = py/pw;
            pz = pz/pw;

            //Get RGB info
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];

            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                            static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    //    point_cloud_ptr->width = 1920;
    //    point_cloud_ptr->height = 1080;
    viewer->removeAllPointClouds();
    viewer->addPointCloud(point_cloud_ptr, "point cloud");
    viewer->spinOnce();
}

bool MainWindow::readStringList( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

void MainWindow::stereoCalibration(const vector<string>& imagelist, Size boardSize, bool useCalibrated, bool showRectified)
{
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return;
    }

    bool displayCorners = false;//true;
    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    cv::resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
                                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, CV_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                cv::resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(30);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                 CV_CALIB_FIX_ASPECT_RATIO +
                                 CV_CALIB_ZERO_TANGENT_DIST +
                                 CV_CALIB_SAME_FOCAL_LENGTH +
                                 CV_CALIB_RATIONAL_MODEL +
                                 CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    cout << "done with RMS error=" << rms << endl;

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                    fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                         imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
              "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    // COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
    // OR ELSE HARTLEY'S METHOD
    else
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            cvtColor(rimg, cimg, CV_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}

// ********************************** UI Sliders ********************************************* //
// ******************************************************************************************* //

void MainWindow::on_sadWindowSizeSlider_valueChanged(int value)
{
    SADWindowSize = value;
}

void MainWindow::on_numOfDisparitiesSlider_valueChanged(int value)
{
    if (value % 16 == 0)
        numberofDisparities = value;
}

void MainWindow::on_preFilterCapSlider_valueChanged(int value)
{
    preFilterCap = value;
}

void MainWindow::on_minDisparitySlider_valueChanged(int value)
{
    minDisparity = value;
}

void MainWindow::on_uniquenessRatioSlider_valueChanged(int value)
{
    uniquenessRatio = value;
}

void MainWindow::on_speckleWindowSizeSlider_valueChanged(int value)
{
    speckleWindowSize = value;
}

void MainWindow::on_speckleRangeSlider_valueChanged(int value)
{
    speckleRange = value;
}

void MainWindow::on_disp12MaxDiffSlider_valueChanged(int value)
{
    disp12MaxDiff = value;
}

void MainWindow::on_preFilterSize_valueChanged(int value)
{
    preFilterSize = value;
}

void MainWindow::on_textureThreshold_valueChanged(int value)
{
    textureThreshold = value;
}

void MainWindow::on_threshold_Slider_valueChanged(int value)
{
    thresholdValue = value;
}

void MainWindow::on_pcl_clicked()
{
    cv::Mat depthImage;
    depthImage = cv::imread("snapshot.jpg", 1);
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pcl::visualization::PCLVisualizer viewer("Point Cloud");
    //    Eigen::
                //createPointcloudFromRegisteredDepthImage();
}
