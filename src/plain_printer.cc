/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm> // for transform
#include <array>     // for array
#include <cmath>     // for ceil
#include <cstddef>   // for size_t
#include <iterator>  // for back_insert_iterator, begin, end
#include <numeric>
#include <ratio>
#include <string_view> // for string_view
#include <utility>
#include <vector>
#include <fmt/core.h>
#include <fmt/ostream.h>

#include "cache.h"              // for CACHE::stats_type, CACHE
#include "champsim_constants.h" // for NUM_CPUS
#include "channel.h"            // for access_type, access_type::LOAD, acce...
#include "dram_controller.h"    // for DRAM_CHANNEL::stats_type, DRAM_CHANNEL
#include "instruction.h"        // for branch_type, BRANCH_CONDITIONAL, BRA...
#include "ooo_cpu.h"            // for O3_CPU::stats_type, O3_CPU
#include "phase_info.h"         // for phase_stats
#include "stats_printer.h"
#include "util/bits.h" // for to_underlying

void champsim::plain_printer::print(O3_CPU::stats_type stats)
{
  constexpr std::array types{branch_type::BRANCH_DIRECT_JUMP, branch_type::BRANCH_INDIRECT,      branch_type::BRANCH_CONDITIONAL,
                             branch_type::BRANCH_DIRECT_CALL, branch_type::BRANCH_INDIRECT_CALL, branch_type::BRANCH_RETURN};
  auto total_branch = std::ceil(std::accumulate(std::begin(types), std::end(types), 0LL,
                                                [tbt = stats.total_branch_types](auto acc, auto next) { return acc + tbt.at(champsim::to_underlying(next)); }));
  auto total_mispredictions = std::ceil(std::accumulate(
      std::begin(types), std::end(types), 0LL, [btm = stats.branch_type_misses](auto acc, auto next) { return acc + btm.at(champsim::to_underlying(next)); }));

  fmt::print(stream, "\n{} cumulative IPC: {:.4g} instructions: {} cycles: {} wrong_path_insts: {} wrong_path_insts_skipped: {} wrong_path_insts_executed: {} instr_foot_print: {} data_foot_print: {}\n", stats.name, std::ceil(stats.instrs()) / std::ceil(stats.cycles()),
             stats.instrs(), stats.cycles(), stats.wrong_path_insts, stats.wrong_path_skipped, stats.wrong_path_insts_executed, stats.instr_foot_print.size(), stats.data_foot_print.size());
  fmt::print(stream, "{} is_prefetch_insts: {} is_prefetch_skipped: {}\n", stats.name, stats.is_prefetch_insts, stats.is_prefetch_skipped);
  fmt::print(stream, "{} Branch Prediction Accuracy: {:.4g}% MPKI: {:.4g} Average ROB Occupancy at Mispredict: {:.4g}\n", stats.name,
             (100.0 * std::ceil(total_branch - total_mispredictions)) / total_branch, (std::kilo::num * total_mispredictions) / std::ceil(stats.instrs()),
             std::ceil(stats.total_rob_occupancy_at_branch_mispredict) / total_mispredictions);

  std::vector<double> mpkis;
  std::transform(std::begin(stats.branch_type_misses), std::end(stats.branch_type_misses), std::back_inserter(mpkis),
                 [instrs = stats.instrs()](auto x) { return std::kilo::num * std::ceil(x) / std::ceil(instrs); });

  fmt::print(stream, "Branch type MPKI\n");
  for (auto idx : types) {
    fmt::print(stream, "{}: {:.3}\n", branch_type_names.at(champsim::to_underlying(idx)), mpkis.at(champsim::to_underlying(idx)));
  }
  fmt::print(stream, "\n");

  fmt::print(stream, "Wrong Path Stats\n");
  fmt::print(stream, "Loads: Count {} Issued {}\n", stats.wrong_path_loads, stats.wrong_path_loads_executed);
  fmt::print(stream, "\n");

  fmt::print(stream, "IDLE Cycles\n");
  fmt::print(stream, "Fetch Idle Cycles {}\n", stats.fetch_idle_cycles);
  fmt::print(stream, "Fetch Blocked Cycles {}\n", stats.fetch_blocked_cycles);
  fmt::print(stream, "IFetch Failed Events {}\n", stats.fetch_failed_events);
  fmt::print(stream, "Fetch Buffer Not Empty {}\n", stats.fetch_buffer_not_empty);
  fmt::print(stream, "Decode Idle Cycles {}\n", stats.decode_idle_cycles);
  fmt::print(stream, "Dispatch Idle Cycles {}\n", stats.dispatch_idle_cycles);
  fmt::print(stream, "Execute Idle Cycles {}\n", stats.execute_idle_cycles);
  fmt::print(stream, "Execute None Cycles {}\n", stats.execute_none_cycles);
  fmt::print(stream, "Execute Head Not Ready Cycles {}\n", stats.execute_head_not_ready);
  fmt::print(stream, "Execute Head Not Completed Cycles {}\n", stats.execute_head_not_completed);
  fmt::print(stream, "Execute Pending Cycles {}\n", stats.execute_pending_cycles);
  fmt::print(stream, "Execute Load Blocked Cycles {}\n", stats.execute_load_blocked_cycles);
  fmt::print(stream, "Scheduler Idle Cycles {}\n", stats.sched_idle_cycles);
  fmt::print(stream, "Scheduler None Cycles {}\n", stats.sched_none_cycles);
  fmt::print(stream, "ROB Idle Cycles {}\n", stats.rob_idle_cycles);
  fmt::print(stream, "LQ Full Events {}\n", stats.lq_full_events);
  fmt::print(stream, "SQ Full Events {}\n", stats.sq_full_events);
  fmt::print(stream, "Non Branch Squashes {}\n", stats.non_branch_squashes);
  fmt::print(stream, "\n");

  fmt::print(stream, "Inst Stats\n");
  fmt::print(stream, "Loads: {} \n", stats.loads);
  fmt::print(stream, "Loads Success: {} \n", stats.loads_success);
  fmt::print(stream, "Loads Executed: {} \n", stats.loads_executed);
  fmt::print(stream, "Loads Retired: {} \n", stats.loads_retired);
  fmt::print(stream, "Stores: {} \n", stats.stores);
  fmt::print(stream, "\n");

}

void champsim::plain_printer::print(CACHE::stats_type stats)
{
  using hits_value_type = typename decltype(stats.hits)::value_type::value_type;
  using misses_value_type = typename decltype(stats.misses)::value_type::value_type;

  for (std::size_t cpu = 0; cpu < NUM_CPUS; ++cpu) {
    hits_value_type total_hits = 0;
    misses_value_type total_misses = 0;
    for (const auto type : {access_type::LOAD, access_type::RFO, access_type::PREFETCH, access_type::WRITE, access_type::TRANSLATION}) {
      total_hits += stats.hits.at(champsim::to_underlying(type)).at(cpu);
      total_misses += stats.misses.at(champsim::to_underlying(type)).at(cpu);
    }

    fmt::format_string<std::string_view, std::string_view, int, int, int> hitmiss_fmtstr{"{} {:<12s} ACCESS: {:10d} HIT: {:10d} MISS: {:10d}\n"};
    fmt::print(stream, hitmiss_fmtstr, stats.name, "TOTAL", total_hits + total_misses, total_hits, total_misses);
    for (const auto type : {access_type::LOAD, access_type::RFO, access_type::PREFETCH, access_type::WRITE, access_type::TRANSLATION}) {
      fmt::print(stream, hitmiss_fmtstr, stats.name, access_type_names.at(champsim::to_underlying(type)),
                 stats.hits.at(champsim::to_underlying(type)).at(cpu) + stats.misses.at(champsim::to_underlying(type)).at(cpu),
                 stats.hits.at(champsim::to_underlying(type)).at(cpu), stats.misses.at(champsim::to_underlying(type)).at(cpu));
    }

    fmt::print(stream, "{} PREFETCH REQUESTED: {:10} ISSUED: {:10} USEFUL: {:10} USELESS: {:10}\n", stats.name, stats.pf_requested, stats.pf_issued,
               stats.pf_useful, stats.pf_useless);
    fmt::print(stream, "{} WRONG-PATH ACCESS: {:10} LOAD: {:10} USEFULL: {:10} FILL: {:10} USELESS: {:10} POLLUTUION: {:.4g} WP_FILL: {:10} WP_MISS: {:10} CP_FILL: {:10} CP_MISS: {:10}\n", stats.name, stats.wp_load + stats.wp_store, stats.wp_load, stats.wp_useful, stats.wp_fill, stats.wp_useless, stats.avg_pollution, stats.wp_fill, stats.wp_miss, stats.cp_fill, stats.cp_miss);

    fmt::print(stream, "{} AVERAGE MISS LATENCY: {:.4g} cycles\n", stats.name, stats.avg_miss_latency);
  }
}

void champsim::plain_printer::print(DRAM_CHANNEL::stats_type stats)
{
  fmt::print(stream, "\n{} RQ ROW_BUFFER_HIT: {:10}\n  ROW_BUFFER_MISS: {:10}\n", stats.name, stats.RQ_ROW_BUFFER_HIT, stats.RQ_ROW_BUFFER_MISS);
  if (stats.dbus_count_congested > 0) {
    fmt::print(stream, " AVG DBUS CONGESTED CYCLE: {:.4g}\n", std::ceil(stats.dbus_cycle_congested) / std::ceil(stats.dbus_count_congested));
  } else {
    fmt::print(stream, " AVG DBUS CONGESTED CYCLE: -\n");
  }
  fmt::print(stream, "WQ ROW_BUFFER_HIT: {:10}\n  ROW_BUFFER_MISS: {:10}\n  FULL: {:10}\n", stats.name, stats.WQ_ROW_BUFFER_HIT, stats.WQ_ROW_BUFFER_MISS,
             stats.WQ_FULL);
}

void champsim::plain_printer::print(champsim::phase_stats& stats)
{
  fmt::print(stream, "=== {} ===\n", stats.name);

  int i = 0;
  for (auto tn : stats.trace_names) {
    fmt::print(stream, "CPU {} runs {}", i++, tn);
  }

  if (NUM_CPUS > 1) {
    fmt::print(stream, "\nTotal Simulation Statistics (not including warmup)\n");

    for (const auto& stat : stats.sim_cpu_stats) {
      print(stat);
    }

    for (const auto& stat : stats.sim_cache_stats) {
      print(stat);
    }
  }

  fmt::print(stream, "\nRegion of Interest Statistics\n");

  for (const auto& stat : stats.roi_cpu_stats) {
    print(stat);
  }

  for (const auto& stat : stats.roi_cache_stats) {
    print(stat);
  }

  fmt::print(stream, "\nDRAM Statistics\n");
  for (const auto& stat : stats.roi_dram_stats) {
    print(stat);
  }
}

void champsim::plain_printer::print(std::vector<phase_stats>& stats)
{
  for (auto p : stats) {
    print(p);
  }
}
