/*
 * MIT License
 *
 * # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <openvdb/Types.h>

namespace {

openvdb::Vec3f BlendColors(const openvdb::Vec3f& color1,
                           float weight1,
                           const openvdb::Vec3f& color2,
                           float weight2,
                           double gamma = 2.2) {
    float weight_sum = weight1 + weight2;
    weight1 /= weight_sum;
    weight2 /= weight_sum;
    return openvdb::Vec3f(
        powf(powf(color1[0], gamma) * weight1 + powf(color2[0], gamma) * weight2, 1. / gamma),
        powf(powf(color1[1], gamma) * weight1 + powf(color2[1], gamma) * weight2, 1. / gamma),
        powf(powf(color1[2], gamma) * weight1 + powf(color2[2], gamma) * weight2, 1. / gamma));
}

}  // unnamed namespace
