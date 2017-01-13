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

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <string>
#include <Eigen/Dense>

#include "common.h"

class Parameters
{
public:
    Parameters();
    void readFromfile(std::string filename);
    int input_size;
    int output_size_x;
    int output_size_y;
    Matrix3fr K_caminv;
    Matrix3fr K_cam;
    float px;
    float py;
    float radial; // only distortion parameter
};

#endif // PARAMETERS_H
