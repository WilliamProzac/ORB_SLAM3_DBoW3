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

// Adapted only for executor-owned workspace reuse from official OpenCV tag 4.2.0.
// Original fast.cpp SHA256: 26ae7f8d3cb64df4dae9c6d0ed784e03ec321fd724347b4a0cb982535b9a99de
// Original fast_score.cpp SHA256: 59117706c02f73b26e8eace331c3fbaf02f72c5d12ee27b3ed4ecd4e431f369a
// https://github.com/opencv/opencv/tree/4.2.0/modules/features2d/src
#include "FastGridKernel.h"
#include <climits>
#include <cstring>
#include <new>
#include <opencv2/features2d.hpp>
#include <opencv2/core/hal/intrin.hpp>

namespace ORB_SLAM3 {
namespace {
using namespace cv;
int CornerScore16(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

#if CV_SIMD128
    if (true)
    {
        v_int16x8 q0 = v_setall_s16(-1000), q1 = v_setall_s16(1000);
        for (k = 0; k < 16; k += 8)
        {
            v_int16x8 v0 = v_load(d + k + 1);
            v_int16x8 v1 = v_load(d + k + 2);
            v_int16x8 a = v_min(v0, v1);
            v_int16x8 b = v_max(v0, v1);
            v0 = v_load(d + k + 3);
            a = v_min(a, v0);
            b = v_max(b, v0);
            v0 = v_load(d + k + 4);
            a = v_min(a, v0);
            b = v_max(b, v0);
            v0 = v_load(d + k + 5);
            a = v_min(a, v0);
            b = v_max(b, v0);
            v0 = v_load(d + k + 6);
            a = v_min(a, v0);
            b = v_max(b, v0);
            v0 = v_load(d + k + 7);
            a = v_min(a, v0);
            b = v_max(b, v0);
            v0 = v_load(d + k + 8);
            a = v_min(a, v0);
            b = v_max(b, v0);
            v0 = v_load(d + k);
            q0 = v_max(q0, v_min(a, v0));
            q1 = v_min(q1, v_max(b, v0));
            v0 = v_load(d + k + 9);
            q0 = v_max(q0, v_min(a, v0));
            q1 = v_min(q1, v_max(b, v0));
        }
        q0 = v_max(q0, v_setzero_s16() - q1);
        threshold = v_reduce_max(q0) - 1;
    }
    else
#endif
    {

        int a0 = threshold;
        for( k = 0; k < 16; k += 2 )
        {
            int a = std::min((int)d[k+1], (int)d[k+2]);
            a = std::min(a, (int)d[k+3]);
            if( a <= a0 )
                continue;
            a = std::min(a, (int)d[k+4]);
            a = std::min(a, (int)d[k+5]);
            a = std::min(a, (int)d[k+6]);
            a = std::min(a, (int)d[k+7]);
            a = std::min(a, (int)d[k+8]);
            a0 = std::max(a0, std::min(a, (int)d[k]));
            a0 = std::max(a0, std::min(a, (int)d[k+9]));
        }

        int b0 = -a0;
        for( k = 0; k < 16; k += 2 )
        {
            int b = std::max((int)d[k+1], (int)d[k+2]);
            b = std::max(b, (int)d[k+3]);
            b = std::max(b, (int)d[k+4]);
            b = std::max(b, (int)d[k+5]);
            if( b >= b0 )
                continue;
            b = std::max(b, (int)d[k+6]);
            b = std::max(b, (int)d[k+7]);
            b = std::max(b, (int)d[k+8]);

            b0 = std::min(b0, std::max(b, (int)d[k]));
            b0 = std::min(b0, std::max(b, (int)d[k+9]));
        }

        threshold = -b0 - 1;
    }

    return threshold;
}

} // namespace

FastGridWorkspace::FastGridWorkspace()
{
    for (int difference = -255; difference <= 255; ++difference)
    {
        threshold7[difference + 255] = difference < -7 ? 1 : difference > 7 ? 2 : 0;
        threshold20[difference + 255] = difference < -20 ? 1 : difference > 20 ? 2 : 0;
    }
}

void FastGridWorkspace::Prepare(const cv::Mat &image)
{
    if (stride != image.step)
    {
        static const int offsets16[16][2] = {
            {0,3},{1,3},{2,2},{3,1},{3,0},{3,-1},{2,-2},{1,-3},
            {0,-3},{-1,-3},{-2,-2},{-3,-1},{-3,0},{-3,1},{-2,2},{-1,3}
        };
        for (int k = 0; k < 16; ++k)
            pixel[k] = offsets16[k][0] + offsets16[k][1] * static_cast<int>(image.step);
        for (int k = 16; k < 25; ++k) pixel[k] = pixel[k - 16];
        stride = image.step;
#ifdef ORB_SLAM3_FASTGRID_TEST_HOOKS
        ++strideRebuilds;
#endif
    }
    const std::size_t width = static_cast<std::size_t>(image.cols);
    for (int row = 0; row < 3; ++row)
    {
#ifdef ORB_SLAM3_FASTGRID_TEST_HOOKS
        if (scores[row].capacity() < width || cornerPositions[row].capacity() < width + 1)
        {
            if (failNextGrowth)
            {
                failNextGrowth = false;
                ++injectedAllocationFailures;
                throw std::bad_alloc();
            }
            ++growths;
        }
#endif
        scores[row].resize(width);
        cornerPositions[row].resize(width + 1);
    }
}

using namespace cv;

void FastGridDetect(const Mat &img, std::vector<KeyPoint>& keypoints, int threshold,
                    bool nonmax_suppression, FastGridWorkspace &workspace)
{
    if (img.type() != CV_8UC1 || img.cols < 7 || img.rows < 7 ||
        img.step > static_cast<std::size_t>((INT_MAX - 3) / 3) ||
        (threshold != 7 && threshold != 20) || !nonmax_suppression)
    {
#ifdef ORB_SLAM3_FASTGRID_TEST_HOOKS
        ++workspace.fallbackCalls;
#endif
        cv::FAST(img, keypoints, threshold, nonmax_suppression,
                 cv::FastFeatureDetector::TYPE_9_16);
        return;
    }
    workspace.Prepare(img);
#ifdef ORB_SLAM3_FASTGRID_TEST_HOOKS
    ++workspace.kernelCalls;
#endif
    const int patternSize = 16, K = 8, N = 25;
    int i, j, k;
    const int *pixel = workspace.pixel.data();

#if CV_SIMD128
    const int quarterPatternSize = patternSize/4;
    v_uint8x16 delta = v_setall_u8(0x80), t = v_setall_u8((char)threshold), K16 = v_setall_u8((char)K);


#endif

    keypoints.clear();

    const uchar *threshold_tab = threshold == 7 ? workspace.threshold7.data()
                                                : workspace.threshold20.data();
    uchar* buf[3];
    int* cpbuf[3];
    for (int row = 0; row < 3; ++row)
    {
        buf[row] = workspace.scores[row].data();
        cpbuf[row] = workspace.cornerPositions[row].data() + 1;
        // Clear every logical row on every cell. Capacity from a previous cell
        // is never read, and cornerpos[-1] is written before it is consumed.
        memset(buf[row], 0, img.cols);
    }

    for(i = 3; i < img.rows-2; i++)
    {
        const uchar* ptr = img.ptr<uchar>(i) + 3;
        uchar* curr = buf[(i - 3)%3];
        int* cornerpos = cpbuf[(i - 3)%3];
        memset(curr, 0, img.cols);
        int ncorners = 0;

        if( i < img.rows - 3 )
        {
            j = 3;
#if CV_SIMD128
            {
                if( patternSize == 16 )
                {

                    //vz if (j <= (img.cols - 27)) //it doesn't make sense using vectors for less than 8 elements
                    {
                        for (; j < img.cols - 16 - 3; j += 16, ptr += 16)
                        {
                            v_uint8x16 v = v_load(ptr);
                            v_int8x16 v0 = v_reinterpret_as_s8((v + t) ^ delta);
                            v_int8x16 v1 = v_reinterpret_as_s8((v - t) ^ delta);

                            v_int8x16 x0 = v_reinterpret_as_s8(v_sub_wrap(v_load(ptr + pixel[0]), delta));
                            v_int8x16 x1 = v_reinterpret_as_s8(v_sub_wrap(v_load(ptr + pixel[quarterPatternSize]), delta));
                            v_int8x16 x2 = v_reinterpret_as_s8(v_sub_wrap(v_load(ptr + pixel[2*quarterPatternSize]), delta));
                            v_int8x16 x3 = v_reinterpret_as_s8(v_sub_wrap(v_load(ptr + pixel[3*quarterPatternSize]), delta));

                            v_int8x16 m0, m1;
                            m0 = (v0 < x0) & (v0 < x1);
                            m1 = (x0 < v1) & (x1 < v1);
                            m0 = m0 | ((v0 < x1) & (v0 < x2));
                            m1 = m1 | ((x1 < v1) & (x2 < v1));
                            m0 = m0 | ((v0 < x2) & (v0 < x3));
                            m1 = m1 | ((x2 < v1) & (x3 < v1));
                            m0 = m0 | ((v0 < x3) & (v0 < x0));
                            m1 = m1 | ((x3 < v1) & (x0 < v1));
                            m0 = m0 | m1;

                            if( !v_check_any(m0) )
                                continue;
                            if( !v_check_any(v_combine_low(m0, m0)) )
                            {
                                j -= 8;
                                ptr -= 8;
                                continue;
                            }

                            v_int8x16 c0 = v_setzero_s8();
                            v_int8x16 c1 = v_setzero_s8();
                            v_uint8x16 max0 = v_setzero_u8();
                            v_uint8x16 max1 = v_setzero_u8();
                            for( k = 0; k < N; k++ )
                            {
                                v_int8x16 x = v_reinterpret_as_s8(v_load((ptr + pixel[k])) ^ delta);
                                m0 = v0 < x;
                                m1 = x < v1;

                                c0 = v_sub_wrap(c0, m0) & m0;
                                c1 = v_sub_wrap(c1, m1) & m1;

                                max0 = v_max(max0, v_reinterpret_as_u8(c0));
                                max1 = v_max(max1, v_reinterpret_as_u8(c1));
                            }

                            max0 = K16 < v_max(max0, max1);
                            unsigned int m = v_signmask(v_reinterpret_as_s8(max0));

                            for( k = 0; m > 0 && k < 16; k++, m >>= 1 )
                            {
                                if( m & 1 )
                                {
                                    cornerpos[ncorners++] = j+k;
                                    if(nonmax_suppression)
                                    {
                                        short d[25];
                                        for (int _k = 0; _k < 25; _k++)
                                            d[_k] = (short)(ptr[k] - ptr[k + pixel[_k]]);

                                        v_int16x8 a0, b0, a1, b1;
                                        a0 = b0 = a1 = b1 = v_load(d + 8);
                                        for(int shift = 0; shift < 8; ++shift)
                                        {
                                            v_int16x8 v_nms = v_load(d + shift);
                                            a0 = v_min(a0, v_nms);
                                            b0 = v_max(b0, v_nms);
                                            v_nms = v_load(d + 9 + shift);
                                            a1 = v_min(a1, v_nms);
                                            b1 = v_max(b1, v_nms);
                                        }
                                        curr[j + k] = (uchar)(v_reduce_max(v_max(v_max(a0, a1), v_setzero_s16() - v_min(b0, b1))) - 1);
                                    }
                                }
                            }
                        }
                    }
                }
            }
#endif
            for( ; j < img.cols - 3; j++, ptr++ )
            {
                int v = ptr[0];
                const uchar* tab = &threshold_tab[0] - v + 255;
                int d = tab[ptr[pixel[0]]] | tab[ptr[pixel[8]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[2]]] | tab[ptr[pixel[10]]];
                d &= tab[ptr[pixel[4]]] | tab[ptr[pixel[12]]];
                d &= tab[ptr[pixel[6]]] | tab[ptr[pixel[14]]];

                if( d == 0 )
                    continue;

                d &= tab[ptr[pixel[1]]] | tab[ptr[pixel[9]]];
                d &= tab[ptr[pixel[3]]] | tab[ptr[pixel[11]]];
                d &= tab[ptr[pixel[5]]] | tab[ptr[pixel[13]]];
                d &= tab[ptr[pixel[7]]] | tab[ptr[pixel[15]]];

                if( d & 1 )
                {
                    int vt = v - threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x < vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                if(nonmax_suppression)
                                    curr[j] = (uchar)CornerScore16(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }

                if( d & 2 )
                {
                    int vt = v + threshold, count = 0;

                    for( k = 0; k < N; k++ )
                    {
                        int x = ptr[pixel[k]];
                        if(x > vt)
                        {
                            if( ++count > K )
                            {
                                cornerpos[ncorners++] = j;
                                if(nonmax_suppression)
                                    curr[j] = (uchar)CornerScore16(ptr, pixel, threshold);
                                break;
                            }
                        }
                        else
                            count = 0;
                    }
                }
            }
        }

        cornerpos[-1] = ncorners;

        if( i == 3 )
            continue;

        const uchar* prev = buf[(i - 4 + 3)%3];
        const uchar* pprev = buf[(i - 5 + 3)%3];
        cornerpos = cpbuf[(i - 4 + 3)%3];
        ncorners = cornerpos[-1];

        for( k = 0; k < ncorners; k++ )
        {
            j = cornerpos[k];
            int score = prev[j];
            if( !nonmax_suppression ||
               (score > prev[j+1] && score > prev[j-1] &&
                score > pprev[j-1] && score > pprev[j] && score > pprev[j+1] &&
                score > curr[j-1] && score > curr[j] && score > curr[j+1]) )
            {
                keypoints.push_back(KeyPoint((float)j, (float)(i-1), 7.f, -1, (float)score));
            }
        }
    }
}

} // namespace ORB_SLAM3
