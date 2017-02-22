// This file is part of dvs-panotracking.
//
// Copyright (C) 2017 Christian Reinbacher <reinbacher at icg dot tugraz dot at>
// Institute for Computer Graphics and Vision, Graz University of Technology
// https://www.tugraz.at/institute/icg/teams/team-pock/
//
// dvs-panotracking is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// dvs-panotracking is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "trackingmainwindow.h"
#include <QGridLayout>
#include <QLabel>
#include <QSpacerItem>
#include <QApplication>
#include <QDockWidget>
#include <qmdisubwindow.h>
#include <QGroupBox>
#include <QMenuBar>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QToolBar>
#include <fstream>
#include "common.h"
//#define DAVIS

TrackingMainWindow::TrackingMainWindow(char *camera_configuration_file, int device_number) : QMainWindow(NULL)
{
#ifdef DAVIS
    int width = 240;
    int height = 180;
#else
    int width = 128;
    int height = 128;
#endif
    parameters_.readFromfile(camera_configuration_file);
    tracking_worker_ = new TrackingWorker(parameters_,width,height, device_number,1.5f);
    camera_worker_ = new DVSCameraWorker(tracking_worker_);

    std::cout << parameters_.K_cam << std::endl;

    mdi_area_ = new QMdiArea(this);
    setCentralWidget(mdi_area_);

    output_win_ = new iu::Qt5ImageGpuWidget(iu::Size<2>(parameters_.output_size_x,parameters_.output_size_y),this);
    QMdiSubWindow* window =  mdi_area_->addSubWindow(output_win_);
    window->setGeometry(QRect(0,0,parameters_.output_size_x+10,parameters_.output_size_y+40));
    window->setWindowTitle("Output Map");
    window->setWindowFlags(Qt::CustomizeWindowHint|Qt::WindowTitleHint);
    window->setMaximumSize(QSize(parameters_.output_size_x+10,parameters_.output_size_y+40));
    window->setMinimumSize(QSize(parameters_.output_size_x+10,parameters_.output_size_y+40));

    status_bar_ = statusBar();

    show();

    while(!output_win_->isValid())    // wait until events are processed and window is created
        QApplication::instance()->processEvents();


    dock_ = new QDockWidget("Parameters", this);
    QGridLayout* layout = new QGridLayout;
    spin_events_per_image_ = new QSpinBox;
    spin_events_per_image_->setMinimum(1);
    spin_events_per_image_->setMaximum(10000);
    spin_events_per_image_->setValue(1500);
    spin_events_per_image_->setSingleStep(100);
    spin_image_skip_ = new QSpinBox;
    spin_image_skip_->setMinimum(1);
    spin_image_skip_->setMaximum(10000);
    spin_image_skip_->setValue(5);
    spin_image_skip_->setSingleStep(1);
    spin_iterations_ = new QSpinBox;
    spin_iterations_->setMinimum(1);
    spin_iterations_->setMaximum(1000);
    spin_iterations_->setValue(10);
    spin_iterations_->setSingleStep(1);
    spin_acceleration_ = new QDoubleSpinBox;
    spin_acceleration_->setMinimum(0);
    spin_acceleration_->setMaximum(1);
    spin_acceleration_->setValue(0.4);
    spin_acceleration_->setSingleStep(0.1);
    action_start_ = new QAction(QIcon(":play.png"),tr("&Start algorithm"),this);
    action_stop_ = new QAction(QIcon(":pause.png"),tr("S&top algorithm"),this);
    action_camera_ = new QAction(QIcon(":camera.png"),tr("St&art camera"),this);
    check_show_camera_pose_ = new QCheckBox("Show camera pose?");
    check_show_camera_pose_->setChecked(true);
    check_show_camera_pose_->setToolTip("Display the current estimated pose");
    check_show_input_events_ = new QCheckBox("Show input events?");
    check_show_input_events_->setChecked(true);
    check_show_input_events_->setToolTip("Display the current events");

    QGroupBox* parameters = new QGroupBox;
    QLabel* label_iterations = new QLabel("Iterations:");
    spin_iterations_->setToolTip("Optimization iterations per pose update");
    QLabel* label_acceleration = new QLabel("Accel. Factor:");
    spin_acceleration_->setToolTip("Weighting of momentum term in optimization");
    QLabel* label_image_skip = new QLabel("Show every nth image:");
    spin_image_skip_->setToolTip("Only display every xth image on screen");
    QLabel* label_events_per_image = new QLabel("Events/image:");
    spin_events_per_image_->setToolTip("Accumulate x events before reconstructing an image");
    QSpacerItem *space = new QSpacerItem(1,1,QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);

    layout->addWidget(label_events_per_image,  0, 0, 1, 1);
    layout->addWidget(spin_events_per_image_,  0, 1, 1, 1);
    layout->addWidget(label_image_skip,        1, 0, 1, 1);
    layout->addWidget(spin_image_skip_,        1, 1, 1, 1);
    layout->addWidget(label_iterations,        2, 0, 1, 1);
    layout->addWidget(spin_iterations_,        2, 1, 1, 1);
    layout->addWidget(label_acceleration,      3, 0, 1, 1);
    layout->addWidget(spin_acceleration_,      3, 1, 1, 1);
    layout->addWidget(check_show_camera_pose_ ,4, 0, 1, 2);
    layout->addWidget(check_show_input_events_,5, 0, 1, 2);
    layout->addItem(space,                     6, 0,-1,-1);

    parameters->setLayout(layout);
    dock_->setWidget(parameters);
    dock_->setFeatures(QDockWidget::NoDockWidgetFeatures);
    dock_->setSizePolicy(QSizePolicy::MinimumExpanding,QSizePolicy::Minimum);
    addDockWidget(Qt::LeftDockWidgetArea, dock_);

    connect(spin_events_per_image_,SIGNAL(valueChanged(int)),tracking_worker_,SLOT(updateEventsPerImage(int)));
    connect(spin_image_skip_,SIGNAL(valueChanged(int)),tracking_worker_,SLOT(updateImageSkip(int)));
    connect(spin_iterations_,SIGNAL(valueChanged(int)),tracking_worker_,SLOT(updateIterations(int)));
    connect(spin_acceleration_,SIGNAL(valueChanged(double)),tracking_worker_,SLOT(updateAcceleration(double)));
    connect(tracking_worker_,SIGNAL(update_output(iu::ImageGpu_8u_C4*)),output_win_,SLOT(update_image(iu::ImageGpu_8u_C4*)));
    connect(tracking_worker_,SIGNAL(update_info(const QString&,int)),status_bar_,SLOT(showMessage(const QString&,int)));
    connect(action_start_,SIGNAL(triggered(bool)),this,SLOT(startTracking()));
    connect(action_stop_,SIGNAL(triggered(bool)),this,SLOT(stopTracking()));
    connect(action_camera_,SIGNAL(triggered(bool)),this,SLOT(startCamera()));
    connect(check_show_camera_pose_,SIGNAL(clicked(bool)),tracking_worker_,SLOT(updateShowCameraPose(bool)));
    connect(check_show_input_events_,SIGNAL(clicked(bool)),tracking_worker_,SLOT(updateShowInputEvents(bool)));

    // Menu Stuff
    action_open_ = new QAction(QIcon(":fileopenevents.png"),tr("&Load events from file"),this);
    connect(action_open_,SIGNAL(triggered(bool)),this,SLOT(loadEvents()));
    action_save_events_ = new QAction(QIcon(":filesaveevents.png"),tr("Sa&ve events to file"),this);
    connect(action_save_events_,SIGNAL(triggered(bool)),this,SLOT(saveEvents()));
    action_save_state_ = new QAction(QIcon(":filesave.png"),tr("Sa&ve state to file"),this);
    connect(action_save_state_,SIGNAL(triggered(bool)),this,SLOT(saveState()));
    action_view_about_ = new QAction(tr("Abo&ut"),this);
    connect(action_view_about_,SIGNAL(triggered(bool)),this,SLOT(showAbout()));

    menu_file_ = menuBar()->addMenu(tr("&File"));
    menu_file_->addAction(action_open_);
    menu_file_->addAction(action_save_events_);
    menu_file_->addAction(action_save_state_);

    menu_view_ = menuBar()->addMenu(tr("&View"));
    menu_view_->addAction(action_view_about_);

    QToolBar *toolbar = new QToolBar;
    toolbar->setMovable(false);
    toolbar->addAction(action_start_);
    toolbar->addAction(action_stop_);
    toolbar->addAction(action_camera_);
    toolbar->addSeparator();
    toolbar->addAction(action_open_);
    toolbar->addAction(action_save_events_);
    toolbar->addAction(action_save_state_);
    addToolBar(Qt::LeftToolBarArea,toolbar);

    // Window title
    setWindowTitle("Real-Time DVS Tracking");
    setGeometry(QRect(0,0,parameters_.output_size_x+10+dock_->geometry().width()+220,(parameters_.output_size_y+10)+90));
    setWindowIcon(QIcon(":vlo_logo.png"));
}

TrackingMainWindow::~TrackingMainWindow()
{
    stopTracking();
    tracking_worker_->wait();
    camera_worker_->wait();
}

void TrackingMainWindow::startTracking()
{
    tracking_worker_->stop();
    if(events_.empty()) { // start camera thread
        camera_worker_->start();
    } else {
        tracking_worker_->addEvents(events_);
    }
    tracking_worker_->start();
}

void TrackingMainWindow::stopTracking()
{
    tracking_worker_->stop();
    camera_worker_->stop();
}

void TrackingMainWindow::startCamera()
{
    events_.clear();
    startTracking();
}

void TrackingMainWindow::loadEvents()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Event File"), "", tr("Event Files (*.aer2 *.dat *.txt)"));
    status_bar_->showMessage("Loading...",0);
    readevents(fileName.toStdString());
}

void TrackingMainWindow::saveEvents()
{
    stopTracking();
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save Events to File"),"/home/christian/data/testdata/event_camera_testdata",tr("Event Files (*.aer2)"));
    tracking_worker_->saveEvents(fileName.toStdString());
}

void TrackingMainWindow::saveState()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save State File"),"/home/christian/data/testdata/event_camera_testdata",tr("All Files, no ext (*.*)"));
    tracking_worker_->saveCurrentState(fileName.toStdString());
}

void TrackingMainWindow::showAbout()
{
    QMessageBox::about(this,"About","Demo application for our publication\n"
                                    "Real-Time Panorama Tracking for Event Cameras\n"
                                    "(c) Institute for Computer Graphics and Vision\n"
                                    "    Vision, Learning and Optimization Group, 2017\n");
}

void TrackingMainWindow::readevents(std::string filename)
{
    QFileInfo info(filename.c_str());
    events_.clear();
    if(info.suffix()=="aer2" || info.suffix()=="txt") {
        ::loadEvents(events_,filename);
    } else if(info.suffix()=="dat"){ // read Bardow files
        Event temp_event;
        float first_timestamp=0;
        float time;
        float last_timestamp=0;
        bool normalize_time=true;
        std::ifstream ifs;
        ifs.open(filename.c_str(),std::ios::in|std::ios::binary);
        if(ifs.good()) {
            unsigned int data;

            while(!ifs.eof()) {
                ifs.read((char*)&data,4);
                time = data;
//                if(first_timestamp==0) {
//                    first_timestamp=time;
//                }
                time-=first_timestamp;
                ifs.read((char*)&data,4);
                temp_event.x = (data & 0x000001FF);
                temp_event.y = (data & 0x0001FE00) >> 9;
                temp_event.polarity = (data & 0x00020000) >> 17;
//                if(flip_ud)
//                    temp_event.y = 127-temp_event.y;
                temp_event.t = time*TIME_CONSTANT;
                temp_event.polarity=temp_event.polarity>0?1:-1;
                events_.push_back(temp_event);
            }
            ifs.close();
        }
    }
    status_bar_->showMessage(tr("Loaded a file with %1 events").arg(events_.size()),0);
}

TrackingMainWindow::TrackingMainWindow()
{

}
