#include "Hashing.h"
#include <array>
#include <atomic>
#include <thread>
#include <set>

// Tokens are identities only, never dereferenced as MapPoint objects.
namespace HASHING {
struct IndexTestAccess {
    static void put(MultiIndexHashing& index, Entry token, unsigned char value) {
        std::unique_lock<std::mutex> lock(index.mMutexUpdate);
        ++index.mDiagnostics.insertions;
        index.insertRecentLocked(token, std::vector<size_t>(32, value));
    }
    static void putMixed(MultiIndexHashing& index, Entry token, unsigned int value) {
        std::unique_lock<std::mutex> lock(index.mMutexUpdate);
        std::vector<size_t> bins(32);
        for (size_t t=0;t<32;++t) bins[t]=(value*(t+1)+t*t)%23;
        index.insertRecentLocked(token,bins);
    }
    static bool consistent(MultiIndexHashing& index) {
        std::unique_lock<std::mutex> lock(index.mMutexUpdate);
        size_t references=0;
        for (size_t t=0; t<32; ++t) for (size_t b=0; b<256; ++b) {
            auto entries=index.mvvBuckets[t][b].back(100000);
            if (entries.size()>10 || std::set<Entry>(entries.begin(),entries.end()).size()!=entries.size()) return false;
            for (auto e:entries) {
                auto p=index.mLocations.find(e);
                if (p==index.mLocations.end() || p->second[t]!=b) return false;
                ++references;
            }
        }
        size_t reverse=0;
        for (const auto& p:index.mLocations) for (auto b:p.second) if (b<256) ++reverse;
        return references==reverse && index.mLocations.size()<=references;
    }
};
}
static void check(bool yes,const char* message) { if(!yes) throw std::runtime_error(message); }
static std::vector<HASHING::Entry> query(HASHING::MultiIndexHashing& index, unsigned char value) {
    std::array<unsigned int,8> descriptor;
    descriptor.fill(static_cast<unsigned int>(value)*0x01010101u);
    std::vector<HASHING::Entry> out;
    index.query(descriptor.data(),out,32,10);
    return out;
}
int main() {
    try {
        std::array<int,64> storage{};
        auto token=[&](int i){return reinterpret_cast<HASHING::Entry>(&storage[i]);};
        HASHING::MultiIndexHashing index(256,32,10);
        using Access=HASHING::IndexTestAccess;
        for(int i=0;i<25;++i) Access::put(index,token(i),0);
        auto d=index.diagnostics();
        check(d.entries==320 && d.max_bucket==10 && d.indexed_points==10,"hard storage bound");
        check(d.evictions==15*32,"eviction count");
        Access::put(index,token(15),0); // oldest becomes newest, no duplicate
        Access::put(index,token(25),0); // then evict former second-oldest (16)
        auto r=query(index,0);
        check(r.size()==320 && std::count(r.begin(),r.end(),token(15))==32,"refresh must survive");
        check(std::count(r.begin(),r.end(),token(16))==0,"LRU eviction order");
        Access::put(index,token(15),1); // descriptor migration removes all old keys
        r=query(index,0);
        check(std::count(r.begin(),r.end(),token(15))==0,"old descriptor buckets must be cleared");
        check(query(index,1).size()==32,"new descriptor buckets");
        Access::put(index,token(16),0); // evicted point can be inserted again
        r=query(index,0);check(std::count(r.begin(),r.end(),token(16))==32,"reinsertion after eviction");
        check(Access::consistent(index),"reverse membership");
        std::atomic<bool> valid{true};
        std::thread writer([&]{for(int i=0;i<4000;++i)Access::put(index,token(i%64),i%7);});
        std::thread reader([&]{for(int i=0;i<4000;++i){auto x=query(index,i%7);if(x.size()>320)valid=false;}});
        std::thread writer2([&]{for(int i=0;i<4000;++i)Access::put(index,token((i+19)%64),(i+3)%7);});
        writer.join();reader.join();writer2.join();
        check(valid && Access::consistent(index),"concurrent insert/query consistency");
        // Different table keys exercise partial eviction and migration: a point
        // can leave some buckets while remaining resident in other tables.
        for(int i=0;i<2000;++i) {
            Access::putMixed(index,token(i%64),static_cast<unsigned int>(i));
            if(i%100==0)check(Access::consistent(index),"partial eviction/migration");
        }
        check(Access::consistent(index),"partial membership final");
        index.initialize();
        check(index.diagnostics().entries==0 && index.diagnostics().indexed_points==0 && query(index,0).empty(),"reset clears ownership metadata");
        std::cout<<"GF02_INDEX_TESTS_PASS capacity refresh duplicate eviction migration reinsertion concurrency partial_membership reset\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
