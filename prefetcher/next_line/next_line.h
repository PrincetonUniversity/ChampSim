#ifndef PREFETCHER_NEXT_LINE_H
#define PREFETCHER_NEXT_LINE_H

#include <cstdint>

#include "modules.h"

struct next_line : champsim::modules::prefetcher {
  using prefetcher::prefetcher;
  void prefetcher_initialize();
  uint32_t prefetcher_cache_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(uint64_t addr, long set, long way, uint8_t prefetch, uint64_t evicted_addr, uint32_t metadata_in);

  // void prefetcher_branch_operate(uint64_t ip, uint8_t branch_type, uint64_t branch_target) {}
  // void prefetcher_cycle_operate() {}
  // void prefetcher_final_stats() {}
};

#endif
