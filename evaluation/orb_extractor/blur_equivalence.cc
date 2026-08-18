#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr int kBorder = 19;
constexpr int kKernelRadius = 3;

bool Compare(const cv::Mat &reference, const cv::Mat &candidate,
             const std::string &label, int case_index)
{
    cv::Mat difference;
    cv::absdiff(reference, candidate, difference);
    const int full_difference = cv::countNonZero(difference);

    cv::Mat boundary_mask(reference.size(), CV_8U, cv::Scalar(0));
    boundary_mask.rowRange(0, kKernelRadius).setTo(255);
    boundary_mask.rowRange(reference.rows-kKernelRadius, reference.rows)
        .setTo(255);
    boundary_mask.colRange(0, kKernelRadius).setTo(255);
    boundary_mask.colRange(reference.cols-kKernelRadius, reference.cols)
        .setTo(255);
    cv::Mat boundary_difference;
    cv::bitwise_and(difference, boundary_mask, boundary_difference);
    const int boundary_count = cv::countNonZero(boundary_difference);

    if(full_difference == 0 && boundary_count == 0)
        return true;
    double maximum_difference = 0.0;
    cv::minMaxLoc(difference, nullptr, &maximum_difference);
    std::cerr << label << " mismatch case=" << case_index
              << " pixels=" << full_difference
              << " boundary_pixels=" << boundary_count
              << " max_difference=" << maximum_difference << std::endl;
    return false;
}

cv::Mat MakePattern(cv::Size size, int case_index)
{
    cv::Mat image(size, CV_8U);
    if(case_index == 0)
    {
        cv::RNG rng(0x5a17c3u + size.width*31u + size.height);
        rng.fill(image, cv::RNG::UNIFORM, 0, 256);
    }
    else if(case_index == 1)
    {
        image.setTo(0);
        image.rowRange(0, 8).setTo(255);
        image.rowRange(image.rows-8, image.rows).setTo(192);
        image.colRange(0, 8).setTo(128);
        image.colRange(image.cols-8, image.cols).setTo(64);
    }
    else
    {
        for(int y = 0; y < image.rows; ++y)
            for(int x = 0; x < image.cols; ++x)
                image.at<std::uint8_t>(y, x) =
                    static_cast<std::uint8_t>((x*17 + y*29 + case_index*43) & 255);
    }
    return image;
}

} // namespace

int main()
{
    const std::vector<cv::Size> sizes = {
        {640, 544}, {533, 453}, {444, 378}, {370, 315}, {309, 262}};
    bool passed = true;
    int case_index = 0;
    for(const cv::Size &size : sizes)
    {
        for(int pattern = 0; pattern < 10; ++pattern, ++case_index)
        {
            cv::Mat parent(size.height + 2*kBorder,
                           size.width + 2*kBorder, CV_8U);
            cv::RNG outside_rng(0x98ab12u + case_index);
            outside_rng.fill(parent, cv::RNG::UNIFORM, 0, 256);
            cv::Mat roi = parent(cv::Rect(kBorder, kBorder,
                                          size.width, size.height));
            MakePattern(size, pattern).copyTo(roi);

            cv::Mat reference = roi.clone();
            cv::GaussianBlur(reference, reference, cv::Size(7, 7),
                             2, 2, cv::BORDER_REFLECT_101);

            cv::Mat reusable_copy(size, CV_8U);
            roi.copyTo(reusable_copy);
            cv::GaussianBlur(reusable_copy, reusable_copy, cv::Size(7, 7),
                             2, 2, cv::BORDER_REFLECT_101);
            passed &= Compare(reference, reusable_copy, "copy", case_index);

            cv::Mat isolated(size, CV_8U);
            cv::GaussianBlur(roi, isolated, cv::Size(7, 7), 2, 2,
                             cv::BORDER_REFLECT_101 | cv::BORDER_ISOLATED);
            passed &= Compare(reference, isolated, "isolated", case_index);
        }
    }
    if(!passed)
        return 1;
    std::cout << "blur equivalence passed: " << case_index
              << " cases, full image and 3-pixel boundary identical"
              << std::endl;
    return 0;
}
