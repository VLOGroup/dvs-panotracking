#ifndef EIGENHELPERS_H
#define EIGENHELPERS_H

#include <Eigen/Dense>

// define a atan2 binary functor
struct atan2Op {
  EIGEN_EMPTY_STRUCT_CTOR(atan2Op)
  float operator()(const float& a, const float& b) const { return atan2(a,b); }
};

#endif // EIGENHELPERS_H
