#pragma once
#include "bus.h"
#include "../recorder.h"
namespace camera_delivery {
class Bridge{
 char path_[480]{};field::Recorder*recorder_;Mapping map_;pthread_t thread_{};
 std::atomic<bool>stop_{false},done_{false};bool started_=false;
 void Record(const Row&r){field::Row f{};static_assert(sizeof(f)==sizeof(r),"bridge layout");memcpy(&f,&r,sizeof(f));recorder_->Record(f);}
 static void*Run(void*p){auto&b=*static_cast<Bridge*>(p);uint64_t next=0;
  while(!b.stop_){uint64_t now=Now();bool tick=now>=next;if(!b.map_.get()&&tick)b.map_.Open(b.path_,false);
   Row health{now,0,63,0,0,{0,0,0,0,0}},progress{};
   if(b.map_.get()){Row rows[64];unsigned n=Drain(b.map_.get(),rows,64,health,progress);for(unsigned i=0;i<n;++i)b.Record(rows[i]);}
   if(tick){b.Record(health);if(progress.kind)b.Record(progress);next=now+1000000000;}
   timespec t{0,100000000};nanosleep(&t,nullptr);
  }b.done_=true;return nullptr;
 }
public:
 Bridge(const char*path,field::Recorder*r):recorder_(r){snprintf(path_,sizeof(path_),"%s",path);}
 bool Start(){if(!ValidPath(path_))return false;pthread_attr_t a;pthread_attr_init(&a);pthread_attr_setstacksize(&a,128*1024);started_=pthread_create(&thread_,&a,Run,this)==0;pthread_attr_destroy(&a);return started_;}
 bool Stop(){stop_=true;uint64_t until=Now()+1000000000;while(started_&&!done_&&Now()<until){timespec t{0,10000000};nanosleep(&t,nullptr);}if(started_&&!done_)return false;if(started_)pthread_join(thread_,nullptr);started_=false;return true;}
};
}
