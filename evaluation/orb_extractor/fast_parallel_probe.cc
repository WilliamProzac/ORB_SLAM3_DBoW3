#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

double ProcessCpuSeconds()
{
    timespec value{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &value) != 0)
        return -1.0;
    return static_cast<double>(value.tv_sec) +
           static_cast<double>(value.tv_nsec) * 1e-9;
}

std::uint64_t RunFastStage(const std::vector<cv::Mat> &pyramid)
{
    constexpr float kCellWidth = 35.0f;
    constexpr int kEdgeThreshold = 19;
    std::vector<cv::KeyPoint> cellKeys;
    std::uint64_t detected = 0;

    for (const cv::Mat &level : pyramid)
    {
        const int minBorder = kEdgeThreshold - 3;
        const int maxBorderX = level.cols - kEdgeThreshold + 3;
        const int maxBorderY = level.rows - kEdgeThreshold + 3;
        const float width = maxBorderX - minBorder;
        const float height = maxBorderY - minBorder;
        const int columns = static_cast<int>(width / kCellWidth);
        const int rows = static_cast<int>(height / kCellWidth);
        const int cellWidth = cvCeil(width / columns);
        const int cellHeight = cvCeil(height / rows);

        for (int row = 0; row < rows; ++row)
        {
            const int beginY = minBorder + row * cellHeight;
            const int endY = std::min(beginY + cellHeight + 6, maxBorderY);
            if (beginY >= maxBorderY - 3)
                continue;
            for (int column = 0; column < columns; ++column)
            {
                const int beginX = minBorder + column * cellWidth;
                const int endX = std::min(beginX + cellWidth + 6, maxBorderX);
                if (beginX >= maxBorderX - 6)
                    continue;
                cellKeys.clear();
                cv::FAST(level.rowRange(beginY, endY).colRange(beginX, endX),
                         cellKeys, 20, true);
                if (cellKeys.empty())
                    cv::FAST(level.rowRange(beginY, endY).colRange(beginX, endX),
                             cellKeys, 7, true);
                detected += cellKeys.size();
            }
        }
    }
    return detected;
}

} // namespace

int main()
{
    constexpr int kLevels = 5;
    constexpr int kMeasuredRuns = 80;
    constexpr float kScaleFactor = 1.2f;

    cv::Mat source(544, 640, CV_8UC1);
    cv::RNG rng(0x52444b35);
    rng.fill(source, cv::RNG::UNIFORM, 0, 256);

    std::vector<cv::Mat> pyramid(kLevels);
    for (int level = 0; level < kLevels; ++level)
    {
        const float scale = 1.0f / std::pow(kScaleFactor, level);
        cv::resize(source, pyramid[level],
                   cv::Size(cvRound(source.cols * scale),
                            cvRound(source.rows * scale)),
                   0.0, 0.0, cv::INTER_LINEAR);
    }

    std::uint64_t detected = RunFastStage(pyramid); // warm OpenCV dispatch
    const double cpuStart = ProcessCpuSeconds();
    const auto wallStart = std::chrono::steady_clock::now();
    for (int run = 0; run < kMeasuredRuns; ++run)
        detected += RunFastStage(pyramid);
    const double wallSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wallStart).count();
    const double cpuSeconds = ProcessCpuSeconds() - cpuStart;
    const double ratio = cpuSeconds / wallSeconds;

    std::cout << std::fixed << std::setprecision(6)
              << "opencv_threads," << cv::getNumThreads() << '\n'
              << "runs," << kMeasuredRuns << '\n'
              << "wall_seconds," << wallSeconds << '\n'
              << "process_cpu_seconds," << cpuSeconds << '\n'
              << "cpu_wall_ratio," << ratio << '\n'
              << "detected_checksum," << detected << '\n'
              << "fast_internally_serial," << (ratio <= 1.2 ? 1 : 0) << '\n';
    return ratio <= 1.2 ? 0 : 3;
}
