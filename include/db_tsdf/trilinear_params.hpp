#ifndef DB_TSDF_TRILINEAR_PARAMS_HPP
#define DB_TSDF_TRILINEAR_PARAMS_HPP

#include <array>
#include <limits>

struct TrilinearParams {
    // Coefficients in normalized local voxel coordinates, in meters.
    double a0{0}, a1{0}, a2{0}, a3{0}, a4{0}, a5{0}, a6{0}, a7{0};
    std::array<double,3> origin{};
    double inverse_resolution{1};
    bool valid{false};

    double interpolate(double x,double y,double z) const {
        if (!valid) return std::numeric_limits<double>::quiet_NaN();
        x=(x-origin[0])*inverse_resolution;
        y=(y-origin[1])*inverse_resolution;
        z=(z-origin[2])*inverse_resolution;
        return a0+a1*x+a2*y+a3*z+a4*x*y+a5*x*z+a6*y*z+a7*x*y*z;
    }
};
#endif
