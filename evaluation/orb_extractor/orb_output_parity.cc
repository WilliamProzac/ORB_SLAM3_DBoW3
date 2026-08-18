#include "ORBextractor.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cstring>
#include <iostream>
#include <vector>

namespace {

bool EqualKeypoint(const cv::KeyPoint &left, const cv::KeyPoint &right)
{
    return std::memcmp(&left.pt.x, &right.pt.x, sizeof(left.pt.x)) == 0 &&
           std::memcmp(&left.pt.y, &right.pt.y, sizeof(left.pt.y)) == 0 &&
           std::memcmp(&left.size, &right.size, sizeof(left.size)) == 0 &&
           std::memcmp(&left.angle, &right.angle, sizeof(left.angle)) == 0 &&
           std::memcmp(&left.response, &right.response,
                       sizeof(left.response)) == 0 &&
           left.octave == right.octave && left.class_id == right.class_id;
}

bool CompareOutputs(int case_index, int mono_serial, int mono_parallel,
                    const std::vector<cv::KeyPoint> &keys_serial,
                    const std::vector<cv::KeyPoint> &keys_parallel,
                    const cv::Mat &desc_serial, const cv::Mat &desc_parallel,
                    std::uint64_t hash_serial, std::uint64_t hash_parallel)
{
    if(mono_serial != mono_parallel ||
       keys_serial.size() != keys_parallel.size() ||
       desc_serial.size() != desc_parallel.size() ||
       desc_serial.type() != desc_parallel.type() ||
       hash_serial != hash_parallel)
    {
        std::cerr << "metadata mismatch case=" << case_index
                  << " mono=" << mono_serial << '/' << mono_parallel
                  << " keys=" << keys_serial.size() << '/'
                  << keys_parallel.size() << " hash=" << hash_serial << '/'
                  << hash_parallel << std::endl;
        return false;
    }
    for(std::size_t i = 0; i < keys_serial.size(); ++i)
    {
        if(!EqualKeypoint(keys_serial[i], keys_parallel[i]))
        {
            std::cerr << "keypoint mismatch case=" << case_index
                      << " index=" << i << std::endl;
            return false;
        }
    }
    if(!desc_serial.empty())
    {
        cv::Mat difference;
        cv::compare(desc_serial, desc_parallel, difference, cv::CMP_NE);
        if(cv::countNonZero(difference) != 0)
        {
            std::cerr << "descriptor mismatch case=" << case_index
                      << std::endl;
            return false;
        }
    }
    return true;
}

cv::Mat MakeImage(int case_index)
{
    cv::Mat image(544, 640, CV_8U);
    cv::RNG rng(0x7193u + case_index*97u);
    rng.fill(image, cv::RNG::UNIFORM, 0, 256);
    for(int y = 20; y < image.rows; y += 47)
        cv::line(image, cv::Point(0, y), cv::Point(image.cols-1, y),
                 cv::Scalar((case_index*31 + y) & 255), 2);
    for(int x = 15; x < image.cols; x += 53)
        cv::circle(image, cv::Point(x, (x*7 + case_index*19) % image.rows),
                   11, cv::Scalar((x*3 + case_index) & 255), 2);
    return image;
}

} // namespace

int main()
{
    ORB_SLAM3::ORBextractor serial(600, 1.2f, 5, 20, 7, 1);
    ORB_SLAM3::ORBextractor parallel(600, 1.2f, 5, 20, 7, 3);
    std::vector<int> lapping = {0, 0};
    for(int case_index = 0; case_index < 30; ++case_index)
    {
        const cv::Mat image = MakeImage(case_index);
        std::vector<cv::KeyPoint> keys_serial;
        std::vector<cv::KeyPoint> keys_parallel;
        cv::Mat desc_serial;
        cv::Mat desc_parallel;
        const int mono_serial = serial(image, cv::noArray(), keys_serial,
                                       desc_serial, lapping);
        const int mono_parallel = parallel(image, cv::noArray(), keys_parallel,
                                           desc_parallel, lapping);
        if(!CompareOutputs(case_index, mono_serial, mono_parallel,
                           keys_serial, keys_parallel, desc_serial,
                           desc_parallel,
                           serial.GetLastStageTiming().output_hash,
                           parallel.GetLastStageTiming().output_hash))
            return 1;
        std::cout << "signature," << case_index << ','
                  << serial.GetLastStageTiming().output_hash << std::endl;
    }
    std::cout << "ORB output parity passed: 30 cases, workers 1 versus 3"
              << std::endl;
    return 0;
}
