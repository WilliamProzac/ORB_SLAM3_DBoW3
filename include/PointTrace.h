#pragma once
// Test-only scalar diagnostics. Configure once before System starts workers.
// Point IDs are values: no trace API accepts or retains MapPoint ownership.
#include "FieldObservation.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
namespace ORB_SLAM3 {
class Frame;
bool ConfigurePointTrace(uint64_t core_frame,const uint64_t* ids,size_t count);
bool PointTraceEnabled(uint64_t core_frame);
bool PointTraceSelected(uint64_t point_id);
void PointTraceAbort(uint64_t core_frame,uint64_t reason);
uint32_t PointTraceProjection(uint64_t core_frame);
void PointTraceSnapshot(uint64_t core_frame,uint32_t phase,const Frame& frame);
inline uint64_t PointTraceFloat(float value) {
 uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;
}
}
