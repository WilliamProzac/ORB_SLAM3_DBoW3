#pragma once
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <new>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
namespace camera_delivery {
inline uint64_t Now(clockid_t clock=CLOCK_MONOTONIC){timespec t{};if(clock_gettime(clock,&t))return 0;return uint64_t(t.tv_sec)*1000000000+t.tv_nsec;}
struct Row{uint64_t time,source;uint16_t kind,valid;uint32_t id;uint64_t v[5];};
static_assert(sizeof(Row)==64,"wire row");
constexpr uint64_t Magic=0x5244353343414d31ULL;
constexpr unsigned Capacity=512;
struct Shared {
 uint64_t magic=0;pthread_mutex_t mutex{};
 uint64_t dropped=0,overflow=0,recovered=0,submitted=0,head=0,tail=0,alive=1;
 uint64_t progress_start=0,progress_source=0;uint32_t progress_phase=0;
 Row rows[Capacity]{};
};
inline bool ValidPath(const char*p){return p&&strncmp(p,"/dev/shm/",9)==0&&p[9]&&strlen(p)<480&&!strchr(p+9,'/');}
inline bool Lock(Shared*s){int r=pthread_mutex_trylock(&s->mutex);if(r==EOWNERDEAD){s->tail=s->head;s->progress_phase=0;++s->recovered;pthread_mutex_consistent(&s->mutex);return true;}return r==0;}
class Mapping{
protected: Shared*s_=nullptr;int fd_=-1;
public:
 Mapping()=default;Mapping(const Mapping&)=delete;Mapping&operator=(const Mapping&)=delete;
 ~Mapping(){Close();}void Close(){if(s_)munmap(s_,sizeof(Shared));if(fd_>=0)close(fd_);s_=nullptr;fd_=-1;}
 Shared*get()const{return s_;}
 bool Open(const char*p,bool create){
  if(!ValidPath(p))return false;
  fd_=open(p,O_RDWR|O_CLOEXEC|O_NOFOLLOW|(create?(O_CREAT|O_EXCL):0),0600);if(fd_<0)return false;
  struct stat st{};bool ok=fstat(fd_,&st)==0&&S_ISREG(st.st_mode)&&st.st_uid==geteuid();
  if(create)ok=ok&&ftruncate(fd_,sizeof(Shared))==0;else ok=ok&&st.st_size==sizeof(Shared);
  if(!ok){Close();if(create)unlink(p);return false;}
  void*m=mmap(nullptr,sizeof(Shared),PROT_READ|PROT_WRITE,MAP_SHARED,fd_,0);
  if(m==MAP_FAILED){Close();if(create)unlink(p);return false;}s_=static_cast<Shared*>(m);
  if(create){new(s_)Shared{};pthread_mutexattr_t a;pthread_mutexattr_init(&a);
   int e=pthread_mutexattr_setpshared(&a,PTHREAD_PROCESS_SHARED);e|=pthread_mutexattr_setrobust(&a,PTHREAD_MUTEX_ROBUST);if(!e)e=pthread_mutex_init(&s_->mutex,&a);pthread_mutexattr_destroy(&a);
   if(e){Close();unlink(p);return false;}__atomic_store_n(&s_->magic,Magic,__ATOMIC_RELEASE);
  }else if(__atomic_load_n(&s_->magic,__ATOMIC_ACQUIRE)!=Magic){Close();return false;}
  return true;
 }
};
struct Stamp{uint64_t time=0,cpu=0;};
class Producer:public Mapping{
 uint64_t attempt_=0;
public:
 ~Producer(){if(s_)__atomic_store_n(&s_->alive,0ULL,__ATOMIC_RELEASE);}
 void Attempt(){++attempt_;}
 void Emit(unsigned kind,unsigned phase,unsigned valid,uint64_t a=0,uint64_t b=0,uint64_t c=0,uint64_t d=0,uint64_t e=0){
  if(!s_)return;
  Row r{Now(),attempt_,uint16_t(kind),uint16_t(valid),phase,{a,b,c,d,e}};
  if(!Lock(s_)){__atomic_fetch_add(&s_->dropped,1ULL,__ATOMIC_RELAXED);return;}
  ++s_->submitted;if(s_->head-s_->tail==Capacity)++s_->overflow;else{s_->rows[s_->head%Capacity]=r;++s_->head;}
  pthread_mutex_unlock(&s_->mutex);
 }
 Stamp Begin(unsigned phase){if(!s_)return {};Stamp t{Now(),Now(CLOCK_THREAD_CPUTIME_ID)};
  if(Lock(s_)){s_->progress_start=t.time;s_->progress_source=attempt_;s_->progress_phase=phase;pthread_mutex_unlock(&s_->mutex);}else __atomic_fetch_add(&s_->dropped,1ULL,__ATOMIC_RELAXED);
  return t;
 }
 void End(Stamp t,unsigned phase,int64_t result=0){if(!s_)return;uint64_t cpu=Now(CLOCK_THREAD_CPUTIME_ID);
  Emit(phase==1?51:54,phase,phase==1?7:3,t.time,cpu>=t.cpu?cpu-t.cpu:0,uint64_t(result));
  if(Lock(s_)){s_->progress_phase=0;pthread_mutex_unlock(&s_->mutex);}else __atomic_fetch_add(&s_->dropped,1ULL,__ATOMIC_RELAXED);
 }
};
// Single diagnostic consumer; normal producer is never blocked by this drain.
inline unsigned Drain(Shared*s,Row*out,unsigned limit,Row&health,Row&progress){
 health={};health.time=Now();health.kind=63;health.valid=1;health.id=2;
 health.v[0]=__atomic_load_n(&s->dropped,__ATOMIC_RELAXED);progress={};
 if(!Lock(s))return 0;
 health.id=__atomic_load_n(&s->alive,__ATOMIC_ACQUIRE)?1:3;health.valid=31;
 health.v[1]=s->overflow;health.v[2]=s->recovered;health.v[3]=s->submitted;
 unsigned n=0;while(n<limit&&s->tail<s->head){out[n++]=s->rows[s->tail%Capacity];++s->tail;}health.v[4]=s->tail;
 if(s->progress_phase&&health.time>=s->progress_start&&health.time-s->progress_start>=250000000){progress={health.time,s->progress_source,55,1,s->progress_phase,{s->progress_start,0,0,0,0}};}
 pthread_mutex_unlock(&s->mutex);return n;
}
}
