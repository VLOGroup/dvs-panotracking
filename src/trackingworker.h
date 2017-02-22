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

#ifndef DENOISINGWORKER_H
#define DENOISINGWORKER_H

#include <QThread>
#include <queue>
#include <QMutex>
#include <Eigen/Dense>

#include "iu/iucore.h"
#include "event.h"
#include "parameters.h"


class TrackingWorker : public QThread
{
    Q_OBJECT
    void run() Q_DECL_OVERRIDE;
public:
    TrackingWorker(const Parameters& cam_parameters, int width=128, int height=128, int device_number=0, float upscale=1.f);
    void addEvents(std::vector<Event>& events);
    void saveEvents(std::string filename);
    void saveCurrentState(std::string filename);
    void track(std::vector<Event>& events);
    Eigen::Vector3f getPose(void){return pose_;}

signals:
    void update_output(iu::ImageGpu_8u_C4*);
    void update_info(const QString&,int);

public slots:
    void stop();
    void updateEventsPerImage(int value){events_per_image_ = value;}
    void updateIterations(int value){iterations_ = value;}
    void updateImageSkip(int value){image_skip_ = value;}
    void updateShowCameraPose(bool value){show_camera_pose_ = value;}
    void updateShowInputEvents(bool value){show_events_ = value;}
    void updateScale(float value);
    void updateAcceleration(double value){alpha_ = value;}

protected:

    bool updatePose(void);
    void clearEvents(void);
    Matrix3fr rodrigues(Eigen::Vector3f in);
    Matrix3fr crossmat(Eigen::Vector3f t);

    int events_per_image_;
    int iterations_;
    int width_;
    int height_;
    Parameters camera_parameters_;

    bool show_camera_pose_;
    bool show_events_;
    int device_number_;
    bool running_;
    int image_id_;
    int image_skip_;
    float upscale_;

    std::queue<Event>  events_;
    std::vector<Event>  all_events_;
    QMutex mutex_events_;
    iu::ImageGpu_32f_C1 *output_;
    iu::ImageGpu_8u_C4 *output_color_;
    iu::ImageGpu_32f_C1 *occurences_;
    iu::ImageGpu_32f_C1 *normalization_;

    iu::LinearHostMemory_32f_C2 *events_cpu_;
    iu::LinearDeviceMemory_32f_C2 * events_gpu_;
    iu::LinearHostMemory_32f_C4 *image_gradients_cpu_;
    iu::LinearDeviceMemory_32f_C4 * image_gradients_gpu_;

    Eigen::Vector3f pose_;
    Eigen::Vector3f old_pose_;
    Matrix3fr R_sphere_;
    float tracking_quality_;

    // optimizer
    float lambda_;
    float lambda_a_;
    float lambda_b_;
    float alpha_;
};

#endif // DENOISINGWORKER_H
