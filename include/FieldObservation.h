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
// Stable decision IDs; __LINE__ remains supplementary provenance only.
// Scalar decisions encode measured >= minimum_to_pass. Rule 103 is compound:
// c=remaining matches, d=map matches; thresholds are fixed by the rule ID.
// Other stage/IMU exits retain the original valid=3 schema.
enum class FieldDecisionRule : uint64_t {
 MotionCoarseMatchesMin20 = 101,
 MotionMapMatchesMin10 = 102,
 MotionLocalizationMatchesGt20AndMapMin10 = 103,
 ReferenceCoarseMatchesMin10 = 201,
 ReferenceMapMatchesMin10 = 202
};
class FieldStage {
 uint64_t frame_; uint32_t stage_;
public:
 FieldStage(uint64_t frame,uint32_t stage):frame_(frame),stage_(stage) {
  FieldEmit(10,frame_,stage_,0);
 }
 bool Result(bool result,uint32_t reason) {
  FieldEmit(11,frame_,stage_,3,result,reason); return result;
 }
 bool Result(bool result,uint32_t reason,uint64_t measured,
             uint64_t minimum_to_pass,FieldDecisionRule rule) {
  FieldEmit(11,frame_,stage_,31,result,reason,measured,minimum_to_pass,
            static_cast<uint64_t>(rule));
  return result;
 }
 bool ResultMotionLocalization(bool result,uint32_t reason,
                               uint64_t matches,uint64_t map_matches) {
  FieldEmit(11,frame_,stage_,31,result,reason,matches,map_matches,
            static_cast<uint64_t>(
                FieldDecisionRule::MotionLocalizationMatchesGt20AndMapMin10));
  return result;
 }
};
}
