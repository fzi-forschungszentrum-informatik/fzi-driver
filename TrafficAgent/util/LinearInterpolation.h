/**
 * @file LinearInterpolation.h
 * @author Markus Lemmer
 * @brief Implementation of the Interpolation Interface with Cubic Interpolation
 * 
 * @copyright (c) 2022, FZI Forschungszentrum Informatik
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef LINEAR_INTERPOLATION
#define LINEAR_INTERPOLATION

#include <vector>

#include "Interpolation.h"
#include "Vector2d.h"

namespace FZIDriver {
    namespace util {
        class LinearInterpolation : public FZIDriver::util::Interpolation {
            private:
                std::vector<Vector2d<double>> params;

            public:
                LinearInterpolation(){};
                /**
                 * @brief Construct a new Linear Interpolation object using x and y data
                 * 
                 * @param xData vector of x values
                 * @param yData vecotr of y values corresponding to the respective x values
                 */
                LinearInterpolation(std::vector<double> xData, std::vector<double> yData);
                ~LinearInterpolation(){};

                /**
                 * @brief Calculate the value of the interpolated function at a given point
                 * 
                 * @param x 
                 * @return double 
                 */
                double operator()(double x) override;
                /**
                 * @brief Calculate the derivative of the interpolated function at a given point.
                 * 
                 * @param x 
                 * @return double 
                 */
                double prime(double x) override;
        };
    }
}

#endif