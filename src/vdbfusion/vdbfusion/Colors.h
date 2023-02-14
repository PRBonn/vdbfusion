#pragma once

#include <openvdb/Types.h>

namespace {

openvdb::Vec3f BlendColors(const openvdb::Vec3f& color1,
                           float weight1,
                           const openvdb::Vec3f& color2,
                           float weight2,
                           double gamma=2.2) {
    float weight_sum = weight1 + weight2;
    weight1 /= weight_sum;
    weight2 /= weight_sum;
    return {
        powf(powf(color1[0], gamma) * weight1 + powf(color2[0], gamma) * weight2, 1./gamma),
        powf(powf(color1[1], gamma) * weight1 + powf(color2[1], gamma) * weight2, 1./gamma),
        powf(powf(color1[2], gamma) * weight1 + powf(color2[2], gamma) * weight2, 1./gamma)
    };
}

}  // unnamed namespace