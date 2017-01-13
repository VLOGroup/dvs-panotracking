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

#ifndef DENOISINGMAINWINDOW_H
#define DENOISINGMAINWINDOW_H

#include <QMainWindow>
#include <QDoubleSpinBox>
#include <QStatusBar>
#include <QMdiArea>
#include <QPushButton>
#include <QTimer>
#include <QAction>
#include <QMenu>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>

#include "parameters.h"
#include <vector>
#include "event.h"
#include "iu/iugui.h"
#include "trackingworker.h"
#include "dvscameraworker.h"


class TrackingMainWindow : public QMainWindow
{
    Q_OBJECT
  public:
    TrackingMainWindow();
    TrackingMainWindow(char* camera_configuration_file, int device_number);
    ~TrackingMainWindow();

  protected slots:
    void startTracking();
    void stopTracking();
    void startCamera();
    void loadEvents();
    void saveEvents();
    void showAbout();
    void saveState();

  protected:
    void readevents(std::string filename);

    iu::Qt5ImageGpuWidget *output_win_;
    iu::Qt5ImageGpuWidget *events_win_;
    std::vector<Event> events_;
    TrackingWorker *tracking_worker_;
    DVSCameraWorker *camera_worker_;
    Parameters parameters_;

    QMdiArea *mdi_area_;
    QStatusBar *status_bar_;
    QTimer update_gl_timer_;
    QDockWidget* dock_;

    QSpinBox *spin_iterations_;
    QSpinBox *spin_image_skip_;
    QSpinBox *spin_events_per_image_;
    QDoubleSpinBox *spin_acceleration_;
    QCheckBox *check_just_display_;
    QCheckBox *check_show_camera_pose_;
    QCheckBox *check_show_input_events_;

    QAction *action_start_;
    QAction *action_stop_;
    QAction *action_camera_;

    QMenu *menu_file_;
    QAction *action_open_;
    QAction *action_save_events_;
    QAction *action_save_state_;
    QMenu *menu_view_;
    QAction *action_view_about_;

    bool simple_mode_;
};

#endif // DENOISINGMAINWINDOW_H
