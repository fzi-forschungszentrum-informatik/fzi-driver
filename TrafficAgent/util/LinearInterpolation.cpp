/*
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
*/

#include "LinearInterpolation.h"

FZIDriver::util::LinearInterpolation::LinearInterpolation(std::vector<double> xData, std::vector<double> yData) : FZIDriver::util::Interpolation(xData, yData) {
    //Interpolation(xData, yData);
    //this->xData = xData;
    //this->yData = yData;

    // iterate over all points in the dataset
    for (int i = 1; i < xData.size(); i++) {
        // calculate the gradient
        double m = (this->yData[i] - this->yData[i-1]) / (this->xData[i] - this->xData[i-1]);
        // calculate the y intercept
        double c = this->yData[i] - m*this->xData[i];
        // store the parameters 
        this->params.push_back(Vector2d<double>(m, c));
    }
}

double FZIDriver::util::LinearInterpolation::operator()(double x) {
    // find the interval the given position x belongs to
    // this assumes that the x data is ordered and strictly increasing
    if (this->xData[0] > x) {
        throw std::runtime_error("The given value of x is to small.");
    }
    int i = 0;
    while (this->xData[i+1] < x) {
        i++;
        if (i > this->params.size()-1) {
            throw std::runtime_error("The given value of x is to big.");
        }
    }

    // read the parameters
    Vector2d<double> params = this->params[i];
    double y = params[0] * x + params[1];

    return y;
}

double FZIDriver::util::LinearInterpolation::prime(double x) {
    // find the interval the given position x belongs to
    // this assumes that the x data is ordered and strictly increasing
    if (this->xData[0] > x) {
        throw std::runtime_error("The given value of x is to small.");
    }
    int i = 0;
    while (this->xData[i+1] < x) {
        i++;
        if (i > this->params.size()-1) {
            throw std::runtime_error("The given value of x is to big.");
        }
    }
    // return the gradient in the given section
    return this->params[i][0];
}