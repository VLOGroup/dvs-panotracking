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

#include "common.h"
#include <fstream>
#include "cnpy.h"
#include "iu/iuio.h"
#include "iu/iumath.h"
#include "iu/iuio/openexrio.h"
#include <Eigen/Dense>

void saveEvents(std::string filename, std::vector<Event> &events)
{
    // create text file and go through all events
    std::ofstream file;

    file.open(filename);
    for (int i=0;i<events.size();i++) {
        file << std::fixed << std::setprecision(6) << events[i].t << " " << events[i].x << " " << events[i].y << " " << std::setprecision(0) << events[i].polarity << std::endl;
    }
    file.close();
}

bool undistortPoint(Event& event, const Matrix3fr& K, float radial)
{
    float ccx = K(0,2)-1;
    float ccy = K(1,2)-1;
    float fc = K(0,0);

    // normalize coordinate
    float ux = (event.x-ccx)/fc;
    float uy = (event.y-ccy)/fc;
    float r2 = ux*ux+uy*uy;
    float rdist = 1.f + radial*r2;
    float rcomp = r2/rdist;
    rdist = 1.f + radial * rcomp;

    event.x_undist = ux / rdist * fc + ccx;
    event.y_undist = uy / rdist * fc + ccy;
    if(event.x_undist<0 || event.x_undist>127 || event.y_undist<0 || event.y_undist>127)
        return false;
    else
        return true;
}

void loadEvents(std::vector<Event>& events, const Matrix3fr &K, float radial, std::string filename)
{
    std::ifstream ifs;
    ifs.open(filename.c_str(),std::ifstream::in);
    if(ifs.good())
    {
        Event temp_event;
        double time;
//        // throw away the first events
//        for(int i=0;i<100000;i++)
//        {
//            ifs >> time;
//            ifs >> temp_event.y;
//            ifs >> temp_event.x;
//            ifs >> temp_event.polarity;
//        }
        while(!ifs.eof())
        {
            ifs >> temp_event.t;
            ifs >> temp_event.x;
            ifs >> temp_event.y;
            ifs >> temp_event.polarity;

            if(!undistortPoint(temp_event,K,radial)) // no points outside the original image
                continue;

            events.push_back(temp_event);
        }
        ifs.close();
    }
}


void loadEvents(std::vector<Event>& events, std::string filename)
{
    std::ifstream ifs;
    ifs.open(filename.c_str(),std::ifstream::in);
    if(ifs.good())
    {
        Event temp_event;
//        double time;
//        // throw away the first events
//        for(int i=0;i<100000;i++)
//        {
//            ifs >> time;
//            ifs >> temp_event.y;
//            ifs >> temp_event.x;
//            ifs >> temp_event.polarity;
//        }
        while(!ifs.eof())
        {
            ifs >> temp_event.t;
            ifs >> temp_event.x;
            ifs >> temp_event.y;
            ifs >> temp_event.polarity;

            events.push_back(temp_event);
        }
        ifs.close();
    }
}

void saveState(std::string filename, const iu::ImageGpu_32f_C1 *mat, bool as_png, bool as_npy, bool as_exr)
{
    iu::ImageCpu_32f_C1 in_cpu(mat->width(),mat->height());
    iu::Size<2> sz = mat->size();
    const unsigned int shape[] = {sz.width,sz.height};
    iu::copy(mat,&in_cpu);
    if(as_npy) {
        // save current image as npy
        cnpy::npy_save(filename + ".npy",in_cpu.data(),shape,2);
    }
    if(as_png) {
        // save current image as png
        iu::imsave(&in_cpu,filename + ".png",true);
    }
    if(as_exr) {
        iu::OpenEXROutputFile out(filename + ".exr", in_cpu.size());
        out.add_channel("u", in_cpu);
        out.write();
    }
}

void saveState(std::string filename, const iu::ImageGpu_8u_C4 *mat)
{
    iu::ImageCpu_8u_C4 in_cpu(mat->width(),mat->height());
    iu::Size<2> sz = mat->size();
    const unsigned int shape[] = {sz.width,sz.height};
    iu::copy(mat,&in_cpu);
    // save current image as png
    iu::imsave(&in_cpu,filename + ".png",true);
}
