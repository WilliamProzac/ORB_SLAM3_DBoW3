#pragma once
#include <atomic>
#include <cstdint>
#include <cstddef>
namespace ORB_SLAM3 { namespace MemoryStats {
// Process-wide, opt-in before constructing any System. No object registry.
bool Enabled();
struct Counts { uint64_t created, retired, destroyed, retired_destroyed; };
Counts Read(unsigned kind);
void Created(unsigned kind);
void Retired(unsigned kind);
void Destroyed(unsigned kind, bool retired);
template<unsigned Kind> class Lifetime {
  const bool counted_ = Enabled();
  std::atomic<bool> retired_{false};
public:
  Lifetime() { if (counted_) Created(Kind); }
  Lifetime(const Lifetime&) = delete;
  Lifetime& operator=(const Lifetime&) = delete;
  void Retire() { if (counted_ && !retired_.exchange(true, std::memory_order_relaxed)) Retired(Kind); }
  ~Lifetime() { if (counted_) Destroyed(Kind, retired_.load(std::memory_order_relaxed)); }
};
// kind 0 = KF, 1 = MP. Each row has fixed width; at most 20000 rows/process.
void BA(const char* kind, size_t kfs, size_t mps, size_t vertices, size_t edges,
        size_t mono, size_t body, size_t stereo, const size_t (&capacities)[9]);
void Save(size_t maps, size_t backups);
// 0=start,1=cancel request,2=optimization returned,3=map committed,4=joined,
// 5=worker returned,6=worker exception. IDs are values, never owning references.
void GBA(unsigned event, unsigned long map, unsigned long loopKF);
void Sample(size_t maps, size_t kfs, size_t mps, size_t local_queue, size_t loop_queue, bool gba);
} }
