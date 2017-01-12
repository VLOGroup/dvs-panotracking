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
