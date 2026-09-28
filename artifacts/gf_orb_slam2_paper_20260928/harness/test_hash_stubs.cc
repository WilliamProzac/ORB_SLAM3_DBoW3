// The isolated index fixture must never dereference its opaque pointer tokens.
#include "MapPoint.h"
#include <stdexcept>
namespace ORB_SLAM2 {
cv::Mat MapPoint::GetDescriptor() { throw std::logic_error("unexpected MapPoint dereference"); }
bool MapPoint::isBad() { throw std::logic_error("unexpected MapPoint dereference"); }
}
