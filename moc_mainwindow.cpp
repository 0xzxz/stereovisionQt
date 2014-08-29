/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Tue Aug 26 22:12:17 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      35,   11,   11,   11, 0x08,
      62,   11,   11,   11, 0x08,
      94,   11,   11,   11, 0x08,
     175,  127,   11,   11, 0x08,
     258,  224,   11,   11, 0x28,
     322,  302,   11,   11, 0x28,
     377,  366,  361,   11, 0x08,
     420,   11,  416,   11, 0x08,
     458,   11,  416,   11, 0x08,
     495,  489,   11,   11, 0x08,
     536,  489,   11,   11, 0x08,
     580,  489,   11,   11, 0x08,
     620,  489,   11,   11, 0x08,
     660,  489,   11,   11, 0x08,
     703,  489,   11,   11, 0x08,
     748,  489,   11,   11, 0x08,
     788,  489,   11,   11, 0x08,
     829,  489,   11,   11, 0x08,
     864,   11,   11,   11, 0x08,
     881,  489,   11,   11, 0x08,
     919,  489,   11,   11, 0x08,
    1003,  957,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0on_openImage_clicked()\0"
    "on_saveImagePair_clicked()\0"
    "on_enableStereoCamera_toggled()\0"
    "on_disableStereoCamera_clicked()\0"
    "imagelist,boardSize,useCalibrated,showRectified\0"
    "stereoCalibration(vector<string>,Size,bool,bool)\0"
    "imagelist,boardSize,useCalibrated\0"
    "stereoCalibration(vector<string>,Size,bool)\0"
    "imagelist,boardSize\0"
    "stereoCalibration(vector<string>,Size)\0"
    "bool\0filename,l\0readStringList(string,vector<string>&)\0"
    "int\0on_loadandStereoCalibration_clicked()\0"
    "on_captureImagePairs_clicked()\0value\0"
    "on_sadWindowSizeSlider_valueChanged(int)\0"
    "on_numOfDisparitiesSlider_valueChanged(int)\0"
    "on_preFilterCapSlider_valueChanged(int)\0"
    "on_minDisparitySlider_valueChanged(int)\0"
    "on_uniquenessRatioSlider_valueChanged(int)\0"
    "on_speckleWindowSizeSlider_valueChanged(int)\0"
    "on_speckleRangeSlider_valueChanged(int)\0"
    "on_disp12MaxDiffSlider_valueChanged(int)\0"
    "on_preFilterSize_valueChanged(int)\0"
    "on_pcl_clicked()\0on_textureThreshold_valueChanged(int)\0"
    "on_threshold_Slider_valueChanged(int)\0"
    "viewer,imgLRectified,disp,Q03,Q13,Q23,Q32,Q33\0"
    "creatPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer>,c"
    "v::Mat,cv::Mat,double,double,double,double,double)\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_openImage_clicked(); break;
        case 1: _t->on_saveImagePair_clicked(); break;
        case 2: _t->on_enableStereoCamera_toggled(); break;
        case 3: _t->on_disableStereoCamera_clicked(); break;
        case 4: _t->stereoCalibration((*reinterpret_cast< const vector<string>(*)>(_a[1])),(*reinterpret_cast< Size(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4]))); break;
        case 5: _t->stereoCalibration((*reinterpret_cast< const vector<string>(*)>(_a[1])),(*reinterpret_cast< Size(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 6: _t->stereoCalibration((*reinterpret_cast< const vector<string>(*)>(_a[1])),(*reinterpret_cast< Size(*)>(_a[2]))); break;
        case 7: { bool _r = _t->readStringList((*reinterpret_cast< const string(*)>(_a[1])),(*reinterpret_cast< vector<string>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 8: { int _r = _t->on_loadandStereoCalibration_clicked();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 9: { int _r = _t->on_captureImagePairs_clicked();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 10: _t->on_sadWindowSizeSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_numOfDisparitiesSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_preFilterCapSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_minDisparitySlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->on_uniquenessRatioSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->on_speckleWindowSizeSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: _t->on_speckleRangeSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->on_disp12MaxDiffSlider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 18: _t->on_preFilterSize_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 19: _t->on_pcl_clicked(); break;
        case 20: _t->on_textureThreshold_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 21: _t->on_threshold_Slider_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 22: _t->creatPointCloud((*reinterpret_cast< boost::shared_ptr<pcl::visualization::PCLVisualizer>(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6])),(*reinterpret_cast< double(*)>(_a[7])),(*reinterpret_cast< double(*)>(_a[8]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
