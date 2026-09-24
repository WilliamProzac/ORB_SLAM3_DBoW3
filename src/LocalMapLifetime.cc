#include "LocalMapLifetime.h"
#include "Map.h"
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>
#include <chrono>
namespace ORB_SLAM3 { namespace LocalLifetime {
namespace {
struct Sink {
  std::mutex mutex;
  std::FILE* file=nullptr;
  bool enabled=false,closed=false;
  uint64_t serial=0;
  std::chrono::steady_clock::time_point start=std::chrono::steady_clock::now();
  Sink() {
    const char* path=std::getenv("ORB_LM_LIFETIME_FILE");
    enabled=path && *path;
    if(!enabled) return;
    file=std::fopen(path,"wx");
    if(!file) throw std::runtime_error("cannot create exclusive ORB_LM_LIFETIME_FILE");
    std::fprintf(file,"serial,wall_s,generation,type,mp_id,kf_id,value,extra,stamp\n");
  }
  ~Sink() {if(file) std::fclose(file);}
};
Sink& sink() {static Sink value;return value;}
}
bool Enabled() {return sink().enabled;}
void Event(Map* map,const char* type,uint64_t point,uint64_t key,int64_t value,int64_t extra,double stamp) {
  auto& out=sink();if(!out.enabled || !map) return;
  const auto generation=map->GetLocalTrackingEvent().generation;
  std::lock_guard<std::mutex> lock(out.mutex);
  if(out.closed) return;
  const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-out.start).count();
  std::fprintf(out.file,"%llu,%.9f,%llu,%s,%llu,%llu,%lld,%lld,%.9f\n",
      (unsigned long long)++out.serial,wall,(unsigned long long)generation,type,
      (unsigned long long)point,(unsigned long long)key,(long long)value,(long long)extra,stamp);
}
void Finish() {
  auto& out=sink();std::lock_guard<std::mutex> lock(out.mutex);
  if(out.file) {std::fclose(out.file);out.file=nullptr;}out.closed=true;
}
} }
