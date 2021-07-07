#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QWindow>
#include <QtConcurrent/QtConcurrent>

#include <iostream>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"

#include <QtCore>
#include <QObject>

// TIP: use #define RDK_SKIP_NAMESPACE to avoid using namespaces
#include "robodk_api.h"
using namespace RoboDK_API;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void timerEvent(QTimerEvent);

    void initUI();

    /// Select a robot
    void select_Robot();

    /// Validate if RoboDK is running (RDK is valid)
    bool check_RoboDK();

    /// Validate if a Robot has been selected (ROBOT variable is valid)
    bool check_Robot();

    /// Apply an incremental movement
    void incrementalMove(int id, double sense);

private slots:
    void on_pushButton_Txm_clicked();

    void on_pushButton_Txp_clicked();

    void on_pushButton_Tym_clicked();

    void on_pushButton_Typ_clicked();

    void on_pushButton_Tzm_clicked();

    void on_pushButton_Tzp_clicked();

    void on_pushButton_Rxm_clicked();

    void on_pushButton_Rxp_clicked();

    void on_pushButton_Rym_clicked();

    void on_pushButton_Ryp_clicked();

    void on_pushButton_Rzm_clicked();

    void on_pushButton_Rzp_clicked();

    void on_comboBox_rotationDisplayType_currentIndexChanged(int index);

    void on_pushButton_RDKStart_clicked();

    void on_pushButton_RDKStop_clicked();

    void on_pushButton_RDKLoadModel_clicked();

    void on_pushButton_selectRobot_clicked();

    void on_pushButton_RDKMoveToHome_clicked();

    void on_pushButton_RDKMoveToStart_clicked();

    void on_pushButton_saveConfig_clicked();

    void on_pushButton_loadProgramFile_clicked();

    void on_pushButton_saveProgramFile_clicked();

    void on_pushButton_runProgram_clicked();

    void on_pushButton_stopProgram_clicked();

    void on_pushButton_loadPointsFile_clicked();

    void on_pushButton_savePointsFile_clicked();

    void on_pushButton_moveToPoint_clicked();

    void on_pushButton_moveToPrePoint_clicked();

    void on_pushButton_moveToNextPoint_clicked();

    void on_pushButton_updatePointsToMemory_clicked();

private:
    Ui::MainWindow *ui;

    /// Pointer to RoboDK
    RoboDK *RDK;

    /// Pointer to the robot item
    Item *ROBOT;

    /// Pointer to the RoboDK window
    QWindow *robodk_window;
};
#endif // MAINWINDOW_H
