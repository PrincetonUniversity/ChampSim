#include "ship.h"

#include <algorithm>
#include <cassert>
#include <random>

// initialize replacement state
ship::ship(CACHE* cache)
    : replacement(cache), NUM_SET(cache->NUM_SET), NUM_WAY(cache->NUM_WAY), sampler(SAMPLER_SET * static_cast<std::size_t>(NUM_WAY)),
      rrpv_values(static_cast<std::size_t>(NUM_SET * NUM_WAY), maxRRPV)
{
  // randomly selected sampler sets
  std::generate_n(std::back_inserter(rand_sets), SAMPLER_SET, std::knuth_b{1});
  std::sort(std::begin(rand_sets), std::end(rand_sets));
}

int& ship::get_rrpv(long set, long way) { return rrpv_values.at(static_cast<std::size_t>(set * NUM_WAY + way)); }

// find replacement victim
long ship::find_victim(uint32_t triggering_cpu, uint64_t instr_id, long set, const CACHE::BLOCK* current_set, uint64_t ip, uint64_t full_addr, access_type type)
{
  // look for the maxRRPV line
  auto begin = std::next(std::begin(rrpv_values), set * NUM_WAY);
  auto end = std::next(begin, NUM_WAY);
  auto victim = std::find(begin, end, maxRRPV);
  while (victim == end) {
    for (auto it = begin; it != end; ++it)
      ++(*it);

    victim = std::find(begin, end, maxRRPV);
  }

  assert(begin <= victim);
  return std::distance(begin, victim); // cast pretected by prior assert
}

// called on every cache hit and cache fill
void ship::update_replacement_state(uint32_t triggering_cpu, long set, long way, uint64_t full_addr, uint64_t ip, uint64_t victim_addr, access_type type,
                                    uint8_t hit)
{
  // handle writeback access
  if (access_type{type} == access_type::WRITE) {
    if (!hit)
      get_rrpv(set, way) = maxRRPV - 1;

    return;
  }

  // update sampler
  auto s_idx = std::find(std::begin(rand_sets), std::end(rand_sets), set);
  if (s_idx != std::end(rand_sets)) {
    auto s_set_begin = std::next(std::begin(sampler), std::distance(std::begin(rand_sets), s_idx));
    auto s_set_end = std::next(s_set_begin, NUM_WAY);

    // check hit
    auto match = std::find_if(s_set_begin, s_set_end,
                              [addr = full_addr, shamt = 8 + champsim::lg2(NUM_WAY)](auto x) { return x.valid && (x.address >> shamt) == (addr >> shamt); });
    if (match != s_set_end) {
      SHCT[triggering_cpu][match->ip % SHCT_PRIME]--;

      match->used = true;
    } else {
      match = std::min_element(s_set_begin, s_set_end, [](auto x, auto y) { return x.last_used < y.last_used; });

      if (match->used)
        SHCT[triggering_cpu][match->ip % SHCT_PRIME]++;

      match->valid = true;
      match->address = full_addr;
      match->ip = ip;
      match->used = false;
    }

    // update LRU state
    match->last_used = access_count++;
  }

  if (hit)
    get_rrpv(set, way) = 0;
  else {
    // SHIP prediction
    get_rrpv(set, way) = maxRRPV - 1;
    if (SHCT[triggering_cpu][ip % SHCT_PRIME].is_max())
      get_rrpv(set, way) = maxRRPV;
  }
}
