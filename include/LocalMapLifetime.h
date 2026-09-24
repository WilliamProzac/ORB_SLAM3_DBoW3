#pragma once
#include <cstdint>
namespace ORB_SLAM3 {
class Map;
enum class LocalPointExit { Other=0, FoundRatio=1, AgeLowObservation=2, ObservationRemoval=3, Reset=4 };
namespace LocalLifetime {
// Opt-in before constructing System: ORB_LM_LIFETIME_FILE. Diagnostics are
// independent of selection scores and never modify lifecycle counters.
bool Enabled();
// No object method is called while the logger mutex is held. Map generation
// comes from its separate event ledger, never its object-container lock.
void Event(Map* map,const char* type,uint64_t point,uint64_t key,
           int64_t value=0,int64_t extra=0,double stamp=0);
void Finish();
}
}
