/* This is FAST corner detector, contributed to OpenCV by the author, Edward Rosten.
   Below is the original copyright and the references */

/*
Copyright (c) 2006, 2008 Edward Rosten
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    *Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

    *Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

    *Neither the name of the University of Cambridge nor the names of
     its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
The references are:
 * Machine learning for high-speed corner detection,
   E. Rosten and T. Drummond, ECCV 2006
 * Faster and better: A machine learning approach to corner detection
   E. Rosten, R. Porter and T. Drummond, PAMI, 2009
*/

#ifndef ORB_SLAM3_FAST_GRID_KERNEL_H
#define ORB_SLAM3_FAST_GRID_KERNEL_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace ORB_SLAM3 {

// One workspace per executor, owned by its extractor; never shared writable state.
// The only custom domain is CPU CV_8UC1 FAST-9 with thresholds 7/20, NMS enabled,
// at least 7x7, and stride small enough for OpenCV's signed sampling offsets.
class FastGridWorkspace {
public:
    FastGridWorkspace();
    void Prepare(const cv::Mat &image);
    std::size_t CapacityBytes() const {
        std::size_t bytes = sizeof(*this);
        for (std::size_t row = 0; row < 3; ++row)
            bytes += scores[row].capacity() * sizeof(unsigned char) +
                     cornerPositions[row].capacity() * sizeof(int);
        return bytes;
    }
    std::array<unsigned char, 512> threshold7{};
    std::array<unsigned char, 512> threshold20{};
    std::array<int, 25> pixel{};
    std::array<std::vector<unsigned char>, 3> scores;
    std::array<std::vector<int>, 3> cornerPositions;
    std::size_t stride = 0;
#ifdef ORB_SLAM3_FASTGRID_TEST_HOOKS
    std::uint64_t kernelCalls = 0, fallbackCalls = 0, strideRebuilds = 0;
    std::uint64_t growths = 0, injectedAllocationFailures = 0;
    bool failNextGrowth = false;
#endif
};

void FastGridDetect(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                    int threshold, bool nonmaxSuppression,
                    FastGridWorkspace &workspace);

} // namespace ORB_SLAM3
#endif
