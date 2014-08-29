/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Tue Aug 26 22:11:56 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QScrollBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QRadioButton *enableStereoCamera;
    QRadioButton *openImage;
    QRadioButton *disableStereoCamera;
    QPushButton *captureImagePairs;
    QPushButton *loadandStereoCalibration;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QScrollBar *sadWindowSizeSlider;
    QScrollBar *numOfDisparitiesSlider;
    QScrollBar *preFilterCapSlider;
    QScrollBar *minDisparitySlider;
    QScrollBar *uniquenessRatioSlider;
    QScrollBar *speckleWindowSizeSlider;
    QScrollBar *speckleRangeSlider;
    QScrollBar *disp12MaxDiffSlider;
    QScrollBar *preFilterSize;
    QScrollBar *textureThreshold;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QLabel *label_4;
    QLabel *label_3;
    QLabel *label_2;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_6;
    QLabel *label_5;
    QLabel *label_18;
    QLabel *label_20;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_19;
    QPushButton *pcl;
    QScrollBar *threshold_Slider;
    QLabel *label_21;
    QLabel *label_22;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QButtonGroup *stereoCameraSwitch;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(640, 554);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        enableStereoCamera = new QRadioButton(centralWidget);
        stereoCameraSwitch = new QButtonGroup(MainWindow);
        stereoCameraSwitch->setObjectName(QString::fromUtf8("stereoCameraSwitch"));
        stereoCameraSwitch->addButton(enableStereoCamera);
        enableStereoCamera->setObjectName(QString::fromUtf8("enableStereoCamera"));
        enableStereoCamera->setGeometry(QRect(60, 80, 201, 22));
        openImage = new QRadioButton(centralWidget);
        openImage->setObjectName(QString::fromUtf8("openImage"));
        openImage->setGeometry(QRect(60, 10, 116, 22));
        disableStereoCamera = new QRadioButton(centralWidget);
        stereoCameraSwitch->addButton(disableStereoCamera);
        disableStereoCamera->setObjectName(QString::fromUtf8("disableStereoCamera"));
        disableStereoCamera->setGeometry(QRect(60, 110, 181, 22));
        captureImagePairs = new QPushButton(centralWidget);
        stereoCameraSwitch->addButton(captureImagePairs);
        captureImagePairs->setObjectName(QString::fromUtf8("captureImagePairs"));
        captureImagePairs->setGeometry(QRect(60, 210, 201, 27));
        loadandStereoCalibration = new QPushButton(centralWidget);
        loadandStereoCalibration->setObjectName(QString::fromUtf8("loadandStereoCalibration"));
        loadandStereoCalibration->setGeometry(QRect(60, 260, 201, 27));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(280, 30, 141, 341));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        sadWindowSizeSlider = new QScrollBar(verticalLayoutWidget);
        sadWindowSizeSlider->setObjectName(QString::fromUtf8("sadWindowSizeSlider"));
        sadWindowSizeSlider->setMinimum(1);
        sadWindowSizeSlider->setMaximum(25);
        sadWindowSizeSlider->setPageStep(1);
        sadWindowSizeSlider->setValue(5);
        sadWindowSizeSlider->setSliderPosition(5);
        sadWindowSizeSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(sadWindowSizeSlider);

        numOfDisparitiesSlider = new QScrollBar(verticalLayoutWidget);
        numOfDisparitiesSlider->setObjectName(QString::fromUtf8("numOfDisparitiesSlider"));
        numOfDisparitiesSlider->setMinimum(16);
        numOfDisparitiesSlider->setMaximum(300);
        numOfDisparitiesSlider->setSingleStep(16);
        numOfDisparitiesSlider->setPageStep(16);
        numOfDisparitiesSlider->setValue(64);
        numOfDisparitiesSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(numOfDisparitiesSlider);

        preFilterCapSlider = new QScrollBar(verticalLayoutWidget);
        preFilterCapSlider->setObjectName(QString::fromUtf8("preFilterCapSlider"));
        preFilterCapSlider->setMinimum(1);
        preFilterCapSlider->setSliderPosition(63);
        preFilterCapSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(preFilterCapSlider);

        minDisparitySlider = new QScrollBar(verticalLayoutWidget);
        minDisparitySlider->setObjectName(QString::fromUtf8("minDisparitySlider"));
        minDisparitySlider->setMinimum(-100);
        minDisparitySlider->setMaximum(200);
        minDisparitySlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(minDisparitySlider);

        uniquenessRatioSlider = new QScrollBar(verticalLayoutWidget);
        uniquenessRatioSlider->setObjectName(QString::fromUtf8("uniquenessRatioSlider"));
        uniquenessRatioSlider->setMaximum(50);
        uniquenessRatioSlider->setValue(15);
        uniquenessRatioSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(uniquenessRatioSlider);

        speckleWindowSizeSlider = new QScrollBar(verticalLayoutWidget);
        speckleWindowSizeSlider->setObjectName(QString::fromUtf8("speckleWindowSizeSlider"));
        speckleWindowSizeSlider->setMaximum(1000);
        speckleWindowSizeSlider->setValue(100);
        speckleWindowSizeSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(speckleWindowSizeSlider);

        speckleRangeSlider = new QScrollBar(verticalLayoutWidget);
        speckleRangeSlider->setObjectName(QString::fromUtf8("speckleRangeSlider"));
        speckleRangeSlider->setMinimum(1);
        speckleRangeSlider->setMaximum(500);
        speckleRangeSlider->setValue(32);
        speckleRangeSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(speckleRangeSlider);

        disp12MaxDiffSlider = new QScrollBar(verticalLayoutWidget);
        disp12MaxDiffSlider->setObjectName(QString::fromUtf8("disp12MaxDiffSlider"));
        disp12MaxDiffSlider->setMinimum(1);
        disp12MaxDiffSlider->setMaximum(640);
        disp12MaxDiffSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(disp12MaxDiffSlider);

        preFilterSize = new QScrollBar(verticalLayoutWidget);
        preFilterSize->setObjectName(QString::fromUtf8("preFilterSize"));
        preFilterSize->setMinimum(5);
        preFilterSize->setMaximum(255);
        preFilterSize->setSingleStep(2);
        preFilterSize->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(preFilterSize);

        textureThreshold = new QScrollBar(verticalLayoutWidget);
        textureThreshold->setObjectName(QString::fromUtf8("textureThreshold"));
        textureThreshold->setMaximum(400);
        textureThreshold->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(textureThreshold);

        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(420, 40, 81, 321));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(verticalLayoutWidget_2);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_2->addWidget(label);

        label_4 = new QLabel(verticalLayoutWidget_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_2->addWidget(label_4);

        label_3 = new QLabel(verticalLayoutWidget_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_2->addWidget(label_3);

        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        label_7 = new QLabel(verticalLayoutWidget_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_2->addWidget(label_7);

        label_8 = new QLabel(verticalLayoutWidget_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_2->addWidget(label_8);

        label_6 = new QLabel(verticalLayoutWidget_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_2->addWidget(label_6);

        label_5 = new QLabel(verticalLayoutWidget_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_2->addWidget(label_5);

        label_18 = new QLabel(verticalLayoutWidget_2);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        verticalLayout_2->addWidget(label_18);

        label_20 = new QLabel(verticalLayoutWidget_2);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        verticalLayout_2->addWidget(label_20);

        verticalLayoutWidget_3 = new QWidget(centralWidget);
        verticalLayoutWidget_3->setObjectName(QString::fromUtf8("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(500, 40, 143, 321));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_9 = new QLabel(verticalLayoutWidget_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_3->addWidget(label_9);

        label_10 = new QLabel(verticalLayoutWidget_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        verticalLayout_3->addWidget(label_10);

        label_11 = new QLabel(verticalLayoutWidget_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        verticalLayout_3->addWidget(label_11);

        label_12 = new QLabel(verticalLayoutWidget_3);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        verticalLayout_3->addWidget(label_12);

        label_13 = new QLabel(verticalLayoutWidget_3);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        verticalLayout_3->addWidget(label_13);

        label_14 = new QLabel(verticalLayoutWidget_3);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        verticalLayout_3->addWidget(label_14);

        label_15 = new QLabel(verticalLayoutWidget_3);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        verticalLayout_3->addWidget(label_15);

        label_16 = new QLabel(verticalLayoutWidget_3);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        verticalLayout_3->addWidget(label_16);

        label_17 = new QLabel(verticalLayoutWidget_3);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        verticalLayout_3->addWidget(label_17);

        label_19 = new QLabel(verticalLayoutWidget_3);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        verticalLayout_3->addWidget(label_19);

        pcl = new QPushButton(centralWidget);
        pcl->setObjectName(QString::fromUtf8("pcl"));
        pcl->setGeometry(QRect(60, 370, 98, 27));
        threshold_Slider = new QScrollBar(centralWidget);
        threshold_Slider->setObjectName(QString::fromUtf8("threshold_Slider"));
        threshold_Slider->setGeometry(QRect(280, 420, 160, 16));
        threshold_Slider->setMaximum(255);
        threshold_Slider->setOrientation(Qt::Horizontal);
        label_21 = new QLabel(centralWidget);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(460, 420, 66, 17));
        label_22 = new QLabel(centralWidget);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(560, 420, 71, 17));
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);
        QObject::connect(preFilterCapSlider, SIGNAL(valueChanged(int)), label_3, SLOT(setNum(int)));
        QObject::connect(uniquenessRatioSlider, SIGNAL(valueChanged(int)), label_7, SLOT(setNum(int)));
        QObject::connect(numOfDisparitiesSlider, SIGNAL(valueChanged(int)), label_4, SLOT(setNum(int)));
        QObject::connect(minDisparitySlider, SIGNAL(valueChanged(int)), label_2, SLOT(setNum(int)));
        QObject::connect(sadWindowSizeSlider, SIGNAL(valueChanged(int)), label, SLOT(setNum(int)));
        QObject::connect(speckleWindowSizeSlider, SIGNAL(valueChanged(int)), label_8, SLOT(setNum(int)));
        QObject::connect(speckleRangeSlider, SIGNAL(valueChanged(int)), label_6, SLOT(setNum(int)));
        QObject::connect(preFilterSize, SIGNAL(valueChanged(int)), label_18, SLOT(setNum(int)));
        QObject::connect(disp12MaxDiffSlider, SIGNAL(valueChanged(int)), label_5, SLOT(setNum(int)));
        QObject::connect(textureThreshold, SIGNAL(valueChanged(int)), label_20, SLOT(setNum(int)));
        QObject::connect(threshold_Slider, SIGNAL(valueChanged(int)), label_21, SLOT(setNum(int)));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        enableStereoCamera->setText(QApplication::translate("MainWindow", "Enable Stereo Camera", 0, QApplication::UnicodeUTF8));
        openImage->setText(QApplication::translate("MainWindow", "Open Image", 0, QApplication::UnicodeUTF8));
        disableStereoCamera->setText(QApplication::translate("MainWindow", "Disable Stereo Camera", 0, QApplication::UnicodeUTF8));
        captureImagePairs->setText(QApplication::translate("MainWindow", "Capture Image Pairs", 0, QApplication::UnicodeUTF8));
        loadandStereoCalibration->setText(QApplication::translate("MainWindow", "Load and Stereo Calibration", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        label_4->setText(QString());
        label_3->setText(QString());
        label_2->setText(QString());
        label_7->setText(QString());
        label_8->setText(QString());
        label_6->setText(QString());
        label_5->setText(QString());
        label_18->setText(QString());
        label_20->setText(QString());
        label_9->setText(QApplication::translate("MainWindow", "SADWindowSize", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "Num of Disparities", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "preFilterCap", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "minDisparity", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "Uniqueness Ratio", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "Speckle Window Size", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "Speckle Range", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindow", "Disp12MaxDiff", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindow", "preFilter Size in BM", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("MainWindow", "TextureThreshold ", 0, QApplication::UnicodeUTF8));
        pcl->setText(QApplication::translate("MainWindow", "pcl", 0, QApplication::UnicodeUTF8));
        label_21->setText(QString());
        label_22->setText(QApplication::translate("MainWindow", "Threshold", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
