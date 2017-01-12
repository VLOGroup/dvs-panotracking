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
