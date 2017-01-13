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

// system includes
#include <fstream>
#include <QApplication>
#include <QVBoxLayout>
#include "iu/iugui.h"
#include <cuda.h>

#include "event.h"
#include "scopedtimer.h"
#include "trackingmainwindow.h"
#include "common.h"

int main(int argc, char**argv)
{

    int numDevices = 0;
    CudaSafeCall(cudaGetDeviceCount(&numDevices));
    std::cout << "found " << numDevices << " devices" << std::endl;

    for (int device = 0; device < numDevices; ++device)
    {
        cudaDeviceProp deviceProperties;
        CudaSafeCall(cudaGetDeviceProperties(&deviceProperties, device));
        std::cout << "device number=" << device << " info= " << deviceProperties.name << std::endl;
    }

    int deviceNumber = 0;
    if (numDevices > 1)
        deviceNumber = 1;

    QApplication app(argc, argv);
    TrackingMainWindow window(argv[1],deviceNumber);
    window.show();

    return app.exec();



}
