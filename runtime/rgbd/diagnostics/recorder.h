#pragma once
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <ctime>
#include <cmath>
#include <new>
#include <pthread.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
namespace field {
inline uint64_t Now(){ timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return uint64_t(t.tv_sec)*1000000000+t.tv_nsec; }
struct Row { uint64_t time,source; uint16_t kind,valid; uint32_t id; uint64_t v[5]; };
static_assert(sizeof(Row)==64,"record budget");
struct Bank { Row rows[7168]; uint64_t count=0,trigger=0,cutoff=0; uint32_t event=0,reason=0; };
struct Config { double before=30,after=10,stall=2,cooldown=60,recovery=5; unsigned events=3; uint64_t disk=32*1024*1024; };
class Recorder {
 Bank banks_[2]; int active_=0,pending_=-1; pthread_mutex_t mutex_=PTHREAD_MUTEX_INITIALIZER;
 pthread_t sampler_{},writer_{}; bool sampler_started_=false,writer_started_=false;
 std::atomic<bool> stop_{false},sampler_done_{false},writer_done_{false};
 std::atomic<uint64_t> requested_trigger_{0}; std::atomic<int> observed_state_{-1};
 std::atomic<uint64_t> dropped_{0},write_errors_{0},busy_{0},last_input_{0};
 Config cfg_; char directory_[512]{}; uint64_t trigger_=0,deadline_=0,last_event_end_=0,ok_since_=0;
 bool ever_ok_=false,ok_=false,armed_=true; unsigned events_=0;
 uint64_t counts_[64]{},hist_[4096]{},seq_bits_[3][1024]{}; uint32_t seq_base_[3]{}; bool seq_have_[3]{};
 uint64_t seq_overflow_[3]{},disk_used_=0,written_=0; int last_state_=-1; uint64_t last_map_[5]{}; unsigned trigger_reason_=0; uint64_t limit_attempts_=0;
 char groups_[8][512]{}; unsigned groups_count_=0; bool groups_truncated_=false;
 // Test-only hooks supplied by the standalone executable, never ROS parameters.
 unsigned test_write_delay_ms_=0;
 static void Pause(unsigned ms){timespec t{time_t(ms/1000),long(ms%1000)*1000000};nanosleep(&t,nullptr);}
 void Add(const Row&r){auto& b=banks_[active_];b.rows[b.count++%7168]=r;}
 void Trigger(uint64_t t,unsigned reason){if(events_>=cfg_.events){++limit_attempts_;return;}if(!armed_||trigger_)return;trigger_=t;ok_since_=0;trigger_reason_=reason;deadline_=t+uint64_t(cfg_.after*1e9);armed_=false;Row r{};r.time=t;r.kind=40;r.valid=1;r.v[0]=reason;Add(r);}
 void Tick(uint64_t t){
  uint64_t requested=requested_trigger_.exchange(0);if(requested)Trigger(requested,1);
  uint64_t last=last_input_.load();
  if(last&&t>=last&&t-last>=uint64_t(cfg_.stall*1e9))Trigger(t,2);
  if(trigger_&&t>=deadline_){
   last_event_end_=t; ++events_;
   if(pending_<0){auto&b=banks_[active_];b.trigger=trigger_;b.cutoff=trigger_>uint64_t(cfg_.before*1e9)?trigger_-uint64_t(cfg_.before*1e9):0;b.event=events_;b.reason=trigger_reason_;pending_=active_;active_=1-active_;banks_[active_].count=0;}
   else ++busy_;
   trigger_=0;
  }
  if(!armed_&&!trigger_&&last&&t>=last&&t-last<uint64_t(cfg_.stall*1e9)&&ok_&&ok_since_&&t>=ok_since_&&t-ok_since_>=uint64_t(cfg_.recovery*1e9)&&t>=last_event_end_&&t-last_event_end_>=uint64_t(cfg_.cooldown*1e9))armed_=true;
 }
 void Seq(int stream,uint32_t seq){if(!seq_have_[stream]){seq_have_[stream]=true;seq_base_[stream]=seq;}uint32_t d=seq-seq_base_[stream];if(d<65536)seq_bits_[stream][d/64]|=uint64_t(1)<<(d%64);else ++seq_overflow_[stream];}
 bool Write(int fd,const char* data,size_t n){
  if(disk_used_+n>cfg_.disk){++write_errors_;return false;}
  size_t pos=0;while(pos<n){ssize_t r=::write(fd,data+pos,n-pos);if(r<0&&errno==EINTR)continue;if(r<=0){++write_errors_;return false;}pos+=size_t(r);disk_used_+=size_t(r);}return true;
 }
 int Open(const char* name){char path[1024];snprintf(path,sizeof(path),"%s/%s",directory_,name);int fd=open(path,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)++write_errors_;return fd;}
 void OutputBank(int index){
  if(test_write_delay_ms_)Pause(test_write_delay_ms_);
  auto& b=banks_[index];char name[64];snprintf(name,sizeof(name),"event_%u.csv",b.event);int fd=Open(name);if(fd<0)return;
  char line[512];int n=snprintf(line,sizeof(line),"# schema=1 trigger_ns=%llu count=%llu overwritten=%llu reason=%u\ntime,source,kind,valid,id,a,b,c,d,e\n",(unsigned long long)b.trigger,(unsigned long long)b.count,(unsigned long long)(b.count>7168?b.count-7168:0),b.reason);
  bool good=Write(fd,line,n);uint64_t begin=b.count>7168?b.count-7168:0;
  for(uint64_t i=begin;good&&i<b.count;++i){const Row&r=b.rows[i%7168];if(r.time<b.cutoff||r.time>b.trigger+uint64_t(cfg_.after*1e9))continue;n=snprintf(line,sizeof(line),"%llu,%llu,%u,%u,%u,%llu,%llu,%llu,%llu,%llu\n",(unsigned long long)r.time,(unsigned long long)r.source,r.kind,r.valid,r.id,(unsigned long long)r.v[0],(unsigned long long)r.v[1],(unsigned long long)r.v[2],(unsigned long long)r.v[3],(unsigned long long)r.v[4]);good=Write(fd,line,n);}
  if(close(fd)!=0){++write_errors_;good=false;}if(good)++written_;
 }
 void Summary(const char*name){int fd=Open(name);if(fd<0)return;char s[512];
  if(std::strcmp(name,"startup.txt")==0){int n=snprintf(s,sizeof(s),"schema=1\nallocated_object=%zu\nthread_stacks=262144\ndriver=unsupported\nfifo=NA\ncgroup_truncated=%d\n",sizeof(*this),groups_truncated_);bool good=Write(fd,s,n);for(unsigned i=0;good&&i<groups_count_;++i){n=snprintf(s,sizeof(s),"cgroup.%u=%.480s\n",i,groups_[i]);good=Write(fd,s,n);}close(fd);return;}

  int n=snprintf(s,sizeof(s),"schema=1\nallocated_object=%zu\nthread_stacks=262144\ndropped=%llu\nwriter_busy=%llu\nwrite_errors=%llu\nevents=%u\nwritten=%llu\ndisk_bytes_before_summary=%llu\nlast_state=%d\ndriver=unsupported\nfifo=NA\ncgroup_truncated=%d\n",sizeof(*this),(unsigned long long)dropped_.load(),(unsigned long long)busy_.load(),(unsigned long long)write_errors_.load(),events_,(unsigned long long)written_,(unsigned long long)disk_used_,last_state_,groups_truncated_);bool good=Write(fd,s,n);
  for(unsigned i=0;good&&i<groups_count_;++i){n=snprintf(s,sizeof(s),"cgroup.%u=%.480s\n",i,groups_[i]);good=Write(fd,s,n);}
  if(std::strcmp(name,"summary.txt")==0){
   n=snprintf(s,sizeof(s),"event_limit_attempts=%llu\n",(unsigned long long)limit_attempts_);good=good&&Write(fd,s,n);
   n=snprintf(s,sizeof(s),"last_map_time=%llu\nlast_map_id=%llu\nlast_active_maps=%llu\nlast_keyframes=%llu\nlast_map_points=%llu\n",(unsigned long long)last_map_[0],(unsigned long long)last_map_[1],(unsigned long long)last_map_[2],(unsigned long long)last_map_[3],(unsigned long long)last_map_[4]);good=good&&Write(fd,s,n);
   for(unsigned i=0;good&&i<64;++i)if(counts_[i]){n=snprintf(s,sizeof(s),"count.%u=%llu\n",i,(unsigned long long)counts_[i]);good=Write(fd,s,n);}
   for(unsigned i=0;good&&i<4096;++i)if(hist_[i]){n=snprintf(s,sizeof(s),"callback_ms_bin.%u=%llu\n",i,(unsigned long long)hist_[i]);good=Write(fd,s,n);}
   for(unsigned j=0;good&&j<3;++j){n=snprintf(s,sizeof(s),"seq.%u.base=%u\nseq.%u.overflow=%llu\n",j,seq_base_[j],j,(unsigned long long)seq_overflow_[j]);good=Write(fd,s,n);for(unsigned i=0;good&&i<1024;++i)if(seq_bits_[j][i]){n=snprintf(s,sizeof(s),"seq.%u.word.%u=%llu\n",j,i,(unsigned long long)seq_bits_[j][i]);good=Write(fd,s,n);}}
  }close(fd);
 }
 static bool Value(const char*file,const char*key,uint64_t&v){FILE*f=fopen(file,"r");if(!f)return false;char line[512];bool found=false;while(fgets(line,sizeof(line),f)){char k[128];unsigned long long x;if(sscanf(line,"%127s %llu",k,&x)==2&&strcmp(k,key)==0){v=x;found=true;break;}}fclose(f);return found;}
 void Resource(){
  Row r{};r.time=Now();r.kind=30;
  if(Value("/proc/self/status","VmRSS:",r.v[0]))r.valid|=1;
  if(Value("/proc/self/status","VmSwap:",r.v[1]))r.valid|=2;
  FILE*f=fopen("/proc/self/stat","r");char buf[4096];if(f){if(fgets(buf,sizeof(buf),f)){char*p=strrchr(buf,')');if(p){char*save=nullptr;unsigned field=3;uint64_t ut=0;for(char*t=strtok_r(p+2," ",&save);t;t=strtok_r(nullptr," ",&save),++field){if(field==10){r.v[2]=strtoull(t,nullptr,10);r.valid|=4;}if(field==12){r.v[3]=strtoull(t,nullptr,10);r.valid|=8;}if(field==14)ut=strtoull(t,nullptr,10);if(field==15){r.v[4]=ut+strtoull(t,nullptr,10);r.valid|=16;break;}}}}fclose(f);}Record(r);
  r={};r.time=Now();r.kind=31;const char*keys[]={"MemAvailable:","pswpin","pswpout","MemTotal:","SwapTotal:"};for(unsigned i=0;i<5;++i)if(Value(i==1||i==2?"/proc/vmstat":"/proc/meminfo",keys[i],r.v[i]))r.valid|=1<<i;Record(r);
  for(unsigned j=0;j<groups_count_;++j){r={};r.time=Now();r.kind=32;r.id=j;char path[1024];const char*ks[]={"pgscan","pgsteal","high","max","oom"};for(unsigned i=0;i<5;++i){snprintf(path,sizeof(path),"/sys/fs/cgroup%s/%s",groups_[j],i<2?"memory.stat":"memory.events");if(Value(path,ks[i],r.v[i]))r.valid|=1<<i;}Record(r);}
 }
 void Groups(){FILE*f=fopen("/proc/self/cgroup","r");if(!f)return;char line[1024];while(fgets(line,sizeof(line),f)){if(strncmp(line,"0::",3))continue;char*p=line+3;p[strcspn(p,"\r\n")]=0;while(true){if(groups_count_==8){groups_truncated_=true;break;}snprintf(groups_[groups_count_++],512,"%.511s",p);if(strcmp(p,"/")==0)break;char*q=strrchr(p,'/');if(!q)break;if(q==p)p[1]=0;else *q=0;}break;}fclose(f);}
 static void* SampleEntry(void*p){auto&s=*static_cast<Recorder*>(p);uint64_t next=0;while(!s.stop_){uint64_t now=Now();if(now>=next){s.Resource();next=now+1000000000;}if(pthread_mutex_trylock(&s.mutex_)==0){s.Tick(now);pthread_mutex_unlock(&s.mutex_);}Pause(100);}s.sampler_done_=true;return nullptr;}
 static void* WriteEntry(void*p){auto&s=*static_cast<Recorder*>(p);s.Summary("startup.txt");for(;;){int index=-1;pthread_mutex_lock(&s.mutex_);index=s.pending_;pthread_mutex_unlock(&s.mutex_);if(index>=0){s.OutputBank(index);pthread_mutex_lock(&s.mutex_);s.pending_=-1;pthread_mutex_unlock(&s.mutex_);}else if(s.stop_&&s.sampler_done_)break;else Pause(20);}s.Summary("summary.txt");s.writer_done_=true;return nullptr;}
public:
 explicit Recorder(const char* directory,Config cfg=Config()):cfg_(cfg){cfg_.before=!std::isfinite(cfg.before)?30:cfg.before<0?0:cfg.before>60?60:cfg.before;cfg_.after=!std::isfinite(cfg.after)?10:cfg.after<0?0:cfg.after>20?20:cfg.after;cfg_.events=cfg.events>3?3:cfg.events;cfg_.disk=cfg.disk>32*1024*1024?32*1024*1024:cfg.disk;snprintf(directory_,sizeof(directory_),"%s",directory);}
 ~Recorder(){pthread_mutex_destroy(&mutex_);}
 bool Start(){
  if(mkdir(directory_,0700)&&errno!=EEXIST)return false;
  DIR*d=opendir(directory_);if(!d)return false;dirent*e;while((e=readdir(d))){if(!strcmp(e->d_name,".")||!strcmp(e->d_name,".."))continue;char p[1024];snprintf(p,sizeof(p),"%s/%s",directory_,e->d_name);struct stat st{};if(lstat(p,&st)||!S_ISREG(st.st_mode)){closedir(d);return false;}disk_used_+=st.st_size;}closedir(d);Groups();
  pthread_attr_t attr;pthread_attr_init(&attr);pthread_attr_setstacksize(&attr,128*1024);
  if(pthread_create(&sampler_,&attr,SampleEntry,this)==0)sampler_started_=true;else{pthread_attr_destroy(&attr);return false;}
  if(pthread_create(&writer_,&attr,WriteEntry,this)==0)writer_started_=true;else{stop_=true;pthread_join(sampler_,nullptr);sampler_started_=false;pthread_attr_destroy(&attr);return false;}pthread_attr_destroy(&attr);return true;
 }
 // Caller must stop producing before Stop. On false, keep object alive until exit.
 bool Stop(unsigned timeout_ms=2000){if(pthread_mutex_trylock(&mutex_)==0){uint64_t request=requested_trigger_.exchange(0);if(request)Trigger(request,1);if(trigger_)Tick(deadline_);pthread_mutex_unlock(&mutex_);}stop_=true;uint64_t until=Now()+uint64_t(timeout_ms)*1000000;while(Now()<until&&((sampler_started_&&!sampler_done_)||(writer_started_&&!writer_done_)))Pause(10);if((sampler_started_&&!sampler_done_)||(writer_started_&&!writer_done_))return false;if(sampler_started_){pthread_join(sampler_,nullptr);sampler_started_=false;}if(writer_started_){pthread_join(writer_,nullptr);writer_started_=false;}return true;}
 void Record(Row r){
  if(!r.time)r.time=Now();
  if(stop_)return;
  if(r.kind==1||r.kind==2)last_input_=r.time;
  if(r.kind==15){int old=observed_state_.exchange(int(r.v[0]));if((old==2||old==5)&&r.v[0]!=2&&r.v[0]!=5){uint64_t zero=0;requested_trigger_.compare_exchange_strong(zero,r.time);}}
  if(pthread_mutex_trylock(&mutex_)!=0){++dropped_;return;}
  Add(r);if(r.kind<64)++counts_[r.kind];
  if(r.kind==1||r.kind==2)Seq(r.kind-1,uint32_t(r.source));
  if(r.kind==3)Seq(2,uint32_t(r.v[2]));
  if(r.kind==22){last_map_[0]=r.time;for(int i=0;i<4;++i)last_map_[i+1]=r.v[i];}
  if(r.kind==4){uint64_t bin=r.v[3]/1000000;hist_[bin<4095?bin:4095]++;}
  if(r.kind==15){bool is_ok=r.v[0]==2||r.v[0]==5;last_state_=int(r.v[0]);if(is_ok){ever_ok_=true;if(!ok_||!ok_since_)ok_since_=r.time;}else {ok_since_=0;}ok_=is_ok;}
  pthread_mutex_unlock(&mutex_);
 }
 static void Sink(void*p,uint16_t kind,uint64_t source,uint32_t id,uint16_t valid,uint64_t a,uint64_t b,uint64_t c,uint64_t d,uint64_t e){Row r{Now(),source,kind,valid,id,{a,b,c,d,e}};static_cast<Recorder*>(p)->Record(r);}
#ifdef FIELD_RECORDER_TEST
 void TestDelay(unsigned ms){test_write_delay_ms_=ms;}
 void TestTrigger(){pthread_mutex_lock(&mutex_);Trigger(Now(),3);pthread_mutex_unlock(&mutex_);}
#endif
};
static_assert(sizeof(Recorder)+256*1024+128*1024<2*1024*1024,"object/stacks/driver reservation");
}
