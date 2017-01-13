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

#ifndef DIRECT_CUH
#define DIRECT_CUH
#include <iu/iucore.h>
#include <Eigen/Dense>

#include "common.h"

namespace  cuda {
    void setCameraMatrices(Matrix3fr &Kcam, Matrix3fr &Kcaminv, float p_x, float p_y, float scale);
    void updateMap(iu::ImageGpu_32f_C1 *map, iu::ImageGpu_32f_C1 *occurences, iu::ImageGpu_32f_C1 *normalization, iu::LinearDeviceMemory_32f_C2 *events, float3 pose, float3 old_pose, int cam_width, int cam_height);
    void getGradients(iu::LinearDeviceMemory_32f_C4 *output, iu::ImageGpu_32f_C1* map, iu::LinearDeviceMemory_32f_C2 *events, float3 pose);
    void createOutput(iu::ImageGpu_8u_C4 *out, iu::ImageGpu_32f_C1 *map, iu::LinearDeviceMemory_32f_C2 *events, float3 pose, int cam_width, int cam_height, float quality);
}

#endif //DIRECT_CUH
