#pragma once
#include <cstdint>
namespace ORB_SLAM3 {
// Diagnostics only; sink is configured before System/worker creation and immutable.
using FieldSink = void (*)(void*, uint16_t, uint64_t, uint32_t, uint16_t,
                           uint64_t,uint64_t,uint64_t,uint64_t,uint64_t);
void SetFieldSink(FieldSink sink, void* context);
bool FieldEnabled();
void FieldEmit(uint16_t kind,uint64_t frame,uint32_t id=0,uint16_t valid=31,
               uint64_t a=0,uint64_t b=0,uint64_t c=0,uint64_t d=0,uint64_t e=0);
void SetHeavyDiagnostics(bool enabled);
bool HeavyDiagnostics();
class FieldStage {
 uint64_t frame_; uint32_t stage_;
public:
 FieldStage(uint64_t frame,uint32_t stage):frame_(frame),stage_(stage) {
  FieldEmit(10,frame_,stage_,0);
 }
 bool Result(bool result,uint32_t reason) {
  FieldEmit(11,frame_,stage_,3,result,reason); return result;
 }
};
}
