#include "MemoryStats.h"
#include "MapPointHandle.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
namespace ORB_SLAM3 { namespace MemoryStats {
namespace {
struct Counters { std::atomic<uint64_t> created{0}, retired{0}, destroyed{0}, retired_destroyed{0}; } counts[2];
std::mutex output_mutex;
uint64_t rows=0, save_calls=0;
FILE* Output() {
  static FILE* file=[] { const char* p=std::getenv("ORB_MEMORY_STATS_FILE"); return p && *p ? std::fopen(p,"w") : nullptr; }();
  return file;
}
double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
}
bool Enabled() { static const bool enabled=[] { const char* p=std::getenv("ORB_MEMORY_STATS"); return p && std::strcmp(p,"1")==0; }(); return enabled; }
void Created(unsigned k) { counts[k].created.fetch_add(1,std::memory_order_relaxed); }
void Retired(unsigned k) { counts[k].retired.fetch_add(1,std::memory_order_relaxed); }
void Destroyed(unsigned k,bool retired) {
  if(retired) counts[k].retired_destroyed.fetch_add(1,std::memory_order_relaxed);
  counts[k].destroyed.fetch_add(1,std::memory_order_relaxed);
}
Counts Read(unsigned k) { auto& c=counts[k]; return {c.created.load(),c.retired.load(),c.destroyed.load(),c.retired_destroyed.load()}; }
void BA(const char* kind,size_t kfs,size_t mps,size_t vertices,size_t edges,size_t mono,size_t body,size_t stereo,const size_t (&cap)[9]) {
  if(!Enabled()) return;
  std::lock_guard<std::mutex> lock(output_mutex); FILE* f=Output(); if(!f || rows++>=20000) return;
  std::fprintf(f,"BA,%.9f,%s,%zu,%zu,%zu,%zu,%zu,%zu,%zu",Now(),kind,kfs,mps,vertices,edges,mono,body,stereo);
  for(auto n:cap) std::fprintf(f,",%zu",n);
  // Element widths: edge pointer, KF pointer, owning MP handle.
  std::fprintf(f,",%zu,%zu,%zu\n",sizeof(void*),sizeof(void*),sizeof(MapPointPtr)); std::fflush(f);
}
void Save(size_t maps,size_t backups) {
  if(!Enabled()) return;
  std::lock_guard<std::mutex> lock(output_mutex); ++save_calls; FILE* f=Output(); if(!f || rows++>=20000) return;
  std::fprintf(f,"SAVE,%.9f,%llu,%zu,%zu\n",Now(),(unsigned long long)save_calls,maps,backups); std::fflush(f);
}
void GBA(unsigned event,unsigned long map,unsigned long loopKF) {
  if(!Enabled()) return;
  std::lock_guard<std::mutex> lock(output_mutex); FILE* f=Output(); if(!f || rows++>=20000) return;
  std::fprintf(f,"GBA,%.9f,%u,%lu,%lu\n",Now(),event,map,loopKF); std::fflush(f);
}
void Sample(size_t maps,size_t kfs,size_t mps,size_t local_queue,size_t loop_queue,bool gba) {
  if(!Enabled()) return;
  std::lock_guard<std::mutex> lock(output_mutex); FILE* f=Output(); if(!f || rows++>=20000) return;
  std::fprintf(f,"LIVE,%.9f,%zu,%zu,%zu,%zu,%zu,%d",Now(),maps,kfs,mps,local_queue,loop_queue,int(gba));
  for(unsigned k=0;k<2;++k) {auto c=Read(k);std::fprintf(f,",%llu,%llu,%llu,%llu",(unsigned long long)c.created,(unsigned long long)c.retired,(unsigned long long)c.destroyed,(unsigned long long)c.retired_destroyed);}
  std::fprintf(f,"\n");std::fflush(f);
}
} }
