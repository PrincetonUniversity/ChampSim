#ifndef REPLACEMENT_SHIP_H
#define REPLACEMENT_SHIP_H

#include <array>
#include <vector>

#include "cache.h"
#include "modules.h"
#include "msl/bits.h"
#include "msl/fwcounter.h"

struct ship : champsim::modules::replacement {
private:
  int& get_rrpv(long set, long way);

public:
  static constexpr int maxRRPV = 3;
  static constexpr std::size_t SHCT_SIZE = 16384;
  static constexpr unsigned SHCT_PRIME = 16381;
  static constexpr std::size_t SAMPLER_SET = (256 * NUM_CPUS);
  static constexpr unsigned SHCT_MAX = 7;

  // sampler structure
  class SAMPLER_class
  {
  public:
    bool valid = false;
    bool used = false;
    uint64_t address = 0, cl_addr = 0, ip = 0;
    uint64_t last_used = 0;
  };

  long NUM_SET, NUM_WAY;
  uint64_t access_count = 0;

  // sampler
  std::vector<std::size_t> rand_sets;
  std::vector<SAMPLER_class> sampler;
  std::vector<int> rrpv_values;

  // prediction table structure
  std::array<std::array<champsim::msl::fwcounter<champsim::msl::lg2(SHCT_MAX + 1)>, SHCT_SIZE>, NUM_CPUS> SHCT;

  explicit ship(CACHE* cache);

  long find_victim(uint32_t triggering_cpu, uint64_t instr_id, long set, const CACHE::BLOCK* current_set, uint64_t ip, uint64_t full_addr, access_type type);
  void update_replacement_state(uint32_t triggering_cpu, long set, long way, uint64_t full_addr, uint64_t ip, uint64_t victim_addr, access_type type,
                                uint8_t hit);

  // use this function to print out your own stats at the end of simulation
  // void replacement_final_stats() {}
};

#endif
