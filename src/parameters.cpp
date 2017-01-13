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

#include "parameters.h"
#include <fstream>

Parameters::Parameters()
{
}

void Parameters::readFromfile(std::string filename)
{
    // load intrinsics
    std::ifstream intrinsics_file;
    intrinsics_file.open(filename.c_str());
    std::string intrinsics_file_line;
    for(size_t i=0; i<3; ++i)
    {
        std::getline(intrinsics_file,intrinsics_file_line);

        std::stringstream str(intrinsics_file_line);
        str >> K_cam(i,0) >> K_cam(i,1) >> K_cam(i,2);
    }
     std::getline(intrinsics_file,intrinsics_file_line);
     std::stringstream(intrinsics_file_line) >> output_size_x;
     std::getline(intrinsics_file,intrinsics_file_line);
     std::stringstream(intrinsics_file_line) >> output_size_y;
     std::getline(intrinsics_file,intrinsics_file_line);
     std::stringstream(intrinsics_file_line) >> radial;
    intrinsics_file.close();

    K_caminv = K_cam.inverse();
    px = output_size_x/2.f;
    py = output_size_y/2.f;
}
