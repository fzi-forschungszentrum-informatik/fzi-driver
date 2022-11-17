/**
 * @file Vector2d.h
 * @author Markus Lemmer
 * @brief Custom vector with 2 elements.
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
#ifndef VECTOR2D
#define VECTOR2D

#include "math.h"

#include "osi3/osi_common.pb.h"

namespace FZIDriver {
    namespace util {

        template <typename T>

        class Vector2d {
            private:
                T elements[2];

            public:
                Vector2d() {};

                Vector2d(T e1, T e2) {
                    elements[0] = e1;
                    elements[1] = e2;
                }

                Vector2d(osi3::Vector3d const osiVector) {
                    elements[0] = osiVector.x();
                    elements[1] = osiVector.y();
                }

                T operator[] (int i) const {return elements[i];}
                T &operator[] (int i) {return elements[i];}

                // define sum and difference of two vectors
                Vector2d<T> operator + (Vector2d<T> const &obj) {
                    return Vector2d(this->elements[0] + obj[0], this->elements[1] + obj[1]);
                }
                Vector2d<T> operator - (Vector2d<T> const &obj) {
                    return Vector2d(this->elements[0] - obj[0], this->elements[1] - obj[1]);
                }

                // define the dot product
                double operator * (Vector2d<T> const &obj) {
                    return this->elements[0] * obj[0] + this->elements[1] * obj[1];
                }

                // define the norm
                double norm() {
                    return sqrt(*this * *this);
                }

                // calculate the distance to another vector
                double distance(Vector2d<double> obj) {
                    return (*this - obj).norm();
                }
        };
    }
}

#endif