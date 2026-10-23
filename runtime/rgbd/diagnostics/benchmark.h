#pragma once
#include <cstdint>
#include <cstdio>
namespace field {
// Validation-only common instrumentation for off/on, not enabled in field config.
struct Benchmark {
 uint64_t seq[3][1024]{},hist[4096]{},counts[3]{},overflow[3]{},drops[2]{},nonok=0,changes=0,max_ns=0,max_gap=0,last_stamp=0;
 uint32_t base[3]{};int state=-1;uint64_t map=0,kfs=0,mps=0;
 void Input(unsigned k,uint32_t v){if(counts[k]++==0)base[k]=v;uint32_t d=v-base[k];if(d<65536)seq[k][d/64]|=uint64_t(1)<<(d%64);else ++overflow[k];}
 void End(uint64_t ns,int s,uint64_t stamp){uint64_t bin=ns/1000000;++hist[bin<4095?bin:4095];if(ns>max_ns)max_ns=ns;if(s!=2&&s!=5)++nonok;if(s!=state){++changes;state=s;}if(last_stamp&&stamp>last_stamp&&stamp-last_stamp>max_gap)max_gap=stamp-last_stamp;last_stamp=stamp;}
 void Save(const char* path){FILE*f=fopen(path,"w");if(!f)return;fprintf(f,"nonok=%llu\nchanges=%llu\nmax_ns=%llu\nmax_source_gap_ns=%llu\nmaps=%llu\nkeyframes=%llu\nmap_points=%llu\n",(unsigned long long)nonok,(unsigned long long)changes,(unsigned long long)max_ns,(unsigned long long)max_gap,(unsigned long long)map,(unsigned long long)kfs,(unsigned long long)mps);for(unsigned j=0;j<3;++j){fprintf(f,"count.%u=%llu\nseq.%u.base=%u\nseq.%u.overflow=%llu\n",j,(unsigned long long)counts[j],j,base[j],j,(unsigned long long)overflow[j]);for(unsigned i=0;i<1024;++i)if(seq[j][i])fprintf(f,"seq.%u.word.%u=%llu\n",j,i,(unsigned long long)seq[j][i]);}for(unsigned i=0;i<4096;++i)if(hist[i])fprintf(f,"callback_ms_bin.%u=%llu\n",i,(unsigned long long)hist[i]);fprintf(f,"sync_drop_rgb=%llu\nsync_drop_depth=%llu\n",(unsigned long long)drops[0],(unsigned long long)drops[1]);fclose(f);}
};
}
