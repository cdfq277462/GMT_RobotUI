#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QCommandLinkButton>
#include <QtGui/QScreen>

#include <Qt3DExtras/qtorusmesh.h>
#include <Qt3DRender/qmesh.h>
#include <Qt3DRender/qtechnique.h>
#include <Qt3DRender/qmaterial.h>
#include <Qt3DRender/qeffect.h>
#include <Qt3DRender/qtexture.h>
#include <Qt3DRender/qrenderpass.h>
#include <Qt3DRender/qsceneloader.h>
#include <Qt3DRender/qpointlight.h>

#include <Qt3DCore/qtransform.h>
#include <Qt3DCore/qaspectengine.h>

#include <Qt3DRender/qrenderaspect.h>
#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DExtras/qt3dwindow.h>
#include <Qt3DExtras/qfirstpersoncameracontroller.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    robodk_window = NULL;
    //adjustSize();

    // Start RoboDK API here (RoboDK will start if it is not running)
    ROBOT = NULL;
    RDK = new RoboDK();
    if (!RDK->Connected()){
        qDebug() << "Failed to start RoboDK API!!";
    }

    initUI();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUI()
{
    // init workspace value
    ui->lineEdit_X->setText(QString::number(double(ui->workSpace_X->value())/100, 'f', 3));
    ui->lineEdit_Y->setText(QString::number(double(ui->workSpace_Y->value())/100, 'f', 3));
    ui->lineEdit_Z->setText(QString::number(double(ui->workSpace_Z->value())/100, 'f', 3));
    ui->lineEdit_theta->setText(QString::number(double(ui->workSpace_theta->value())/100, 'f', 3));
    ui->lineEdit_phi->setText(QString::number(double(ui->workSpace_phi->value())/100, 'f', 3));
    ui->lineEdit_psi->setText(QString::number(double(ui->workSpace_psi->value())/100, 'f', 3));
    // end init workspace value
}


bool MainWindow::check_RoboDK()
{
    if (RDK == NULL){
        statusBar()->showMessage("RoboDK API is not connected");
        return false;
    }
    if (!RDK->Connected()){
        statusBar()->showMessage("RoboDK is not running");
        return false;
    }
    return true;
}

bool MainWindow::check_Robot()
{
    if (!check_RoboDK()){ return false; }

    if (ROBOT == NULL){
        statusBar()->showMessage("Select a robot first");
        return false;
    }
    if (!ROBOT->Valid()){
        statusBar()->showMessage("Robot item is not valid");
        return false;
    }
    return true;
}

void MainWindow::select_Robot()
{
    if (ROBOT != NULL){
        delete ROBOT;
        ROBOT = NULL;
    }
    ROBOT = new Item(RDK->ItemUserPick("Select a robot", RoboDK::ITEM_TYPE_ROBOT));
    //ROBOT = new Item(RDK->getItem("UR10", RoboDK::ITEM_TYPE_ROBOT));
    if (check_Robot()){
        statusBar()->showMessage("Robot selected: " + ROBOT->Name());
    }
}
/*
void MainWindow::on_pushButton_Txm_clicked(){ incrementalMove(0, -1); }
void MainWindow::on_pushButton_Tym_clicked(){ incrementalMove(1, -1); }
void MainWindow::on_pushButton_Tzm_clicked(){ incrementalMove(2, -1); }
void MainWindow::on_pushButton_Rxm_clicked(){ incrementalMove(3, -1); }
void MainWindow::on_pushButton_Rym_clicked(){ incrementalMove(4, -1); }
void MainWindow::on_pushButton_Rzm_clicked(){ incrementalMove(5, -1); }

void MainWindow::on_pushButton_Txp_clicked(){ incrementalMove(0, +1); }
void MainWindow::on_pushButton_Typ_clicked(){ incrementalMove(1, +1); }
void MainWindow::on_pushButton_Tzp_clicked(){ incrementalMove(2, +1); }
void MainWindow::on_pushButton_Rxp_clicked(){ incrementalMove(3, +1); }
void MainWindow::on_pushButton_Ryp_clicked(){ incrementalMove(4, +1); }
void MainWindow::on_pushButton_Rzp_clicked(){ incrementalMove(5, +1); }
*/

void MainWindow::incrementalMove(int id, double sense)
{
    if (!check_Robot()) { return; }

    // check the index
    if (id < 0 || id >= 6){
        qDebug()<< "Invalid id provided to for an incremental move";
        return;
    }

    // calculate the relative movement
    double step = sense;

    // apply to XYZWPR
    tXYZWPR xyzwpr;
    for (int i=0; i<6; i++){
        xyzwpr[i] = 0;
    }
    xyzwpr[id] = step;

    Mat pose_increment;
    pose_increment.FromXYZRPW(xyzwpr);

    Mat pose_robot = ROBOT->Pose();

    Mat pose_robot_new;

    // apply relative to the TCP:
    pose_robot_new = pose_robot * pose_increment;

    ROBOT->MoveJ(pose_robot_new);
}

void MainWindow::frameUpdate()
{

}

void MainWindow::workSapceUpdate()
{
    // set workspace value
    ui->lineEdit_X->setText(QString::number(double(ui->workSpace_X->value())/100, 'f', 3));
    ui->lineEdit_Y->setText(QString::number(double(ui->workSpace_Y->value())/100, 'f', 3));
    ui->lineEdit_Z->setText(QString::number(double(ui->workSpace_Z->value())/100, 'f', 3));
    ui->lineEdit_theta->setText(QString::number(double(ui->workSpace_theta->value())/100, 'f', 3));
    ui->lineEdit_phi->setText(QString::number(double(ui->workSpace_phi->value())/100, 'f', 3));
    ui->lineEdit_psi->setText(QString::number(double(ui->workSpace_psi->value())/100, 'f', 3));
}

void MainWindow::on_comboBox_rotationDisplayType_currentIndexChanged(int index)
{

}

void MainWindow::on_pushButton_RDKStart_clicked()
{

}

void MainWindow::on_pushButton_RDKStop_clicked()
{

}

void MainWindow::on_pushButton_RDKLoadModel_clicked()
{

}

void MainWindow::on_pushButton_selectRobot_clicked()
{
    select_Robot();
}

void MainWindow::on_pushButton_RDKMoveToHome_clicked()
{

}

void MainWindow::on_pushButton_RDKMoveToStart_clicked()
{

}

void MainWindow::on_pushButton_saveConfig_clicked()
{

}

void MainWindow::on_pushButton_loadProgramFile_clicked()
{

}

void MainWindow::on_pushButton_saveProgramFile_clicked()
{

}

void MainWindow::on_pushButton_runProgram_clicked()
{

}

void MainWindow::on_pushButton_stopProgram_clicked()
{

}

void MainWindow::on_pushButton_loadPointsFile_clicked()
{

}

void MainWindow::on_pushButton_savePointsFile_clicked()
{

}

void MainWindow::on_pushButton_moveToPoint_clicked()
{

}

void MainWindow::on_pushButton_moveToPrePoint_clicked()
{

}

void MainWindow::on_pushButton_moveToNextPoint_clicked()
{

}

void MainWindow::on_pushButton_updatePointsToMemory_clicked()
{

}

void MainWindow::on_workSpace_X_valueChanged(int value)
{
    workSapceUpdate();
}
void MainWindow::on_workSpace_Y_valueChanged(int value)
{
    workSapceUpdate();
}

void MainWindow::on_workSpace_Z_valueChanged(int value)
{
    workSapceUpdate();
}

void MainWindow::on_workSpace_theta_valueChanged(int value)
{
    workSapceUpdate();
}

void MainWindow::on_workSpace_phi_valueChanged(int value)
{
    workSapceUpdate();
}

void MainWindow::on_workSpace_psi_valueChanged(int value)
{
    workSapceUpdate();
}
