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

#ifndef DRAM_H
#define DRAM_H

#include <array>
#include <cstddef>  // for size_t
#include <cstdint>  // for uint64_t, uint32_t, uint8_t
#include <deque>    // for deque
#include <iterator> // for end
#include <limits>
#include <optional>
#include <string>
#include <vector> // for vector, vector<>::iterator

#include "champsim_constants.h"
#include "channel.h"
#include "operable.h"

struct ooo_model_instr;

struct dram_stats {
  std::string name{};
  uint64_t dbus_cycle_congested = 0, dbus_count_congested = 0;

  unsigned WQ_ROW_BUFFER_HIT = 0, WQ_ROW_BUFFER_MISS = 0, RQ_ROW_BUFFER_HIT = 0, RQ_ROW_BUFFER_MISS = 0, WQ_FULL = 0;
};

struct DRAM_CHANNEL final : public champsim::operable {
  using response_type = typename champsim::channel::response_type;
  struct request_type {
    bool scheduled = false;
    bool forward_checked = false;

    uint8_t asid[2] = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};

    uint32_t pf_metadata = 0;

    uint64_t address = 0;
    uint64_t v_address = 0;
    uint64_t data = 0;
    uint64_t event_cycle = std::numeric_limits<uint64_t>::max();

    std::vector<uint64_t> instr_depend_on_me{};
    std::vector<std::deque<response_type>*> to_return{};

    explicit request_type(const typename champsim::channel::request_type& req);
  };
  using value_type = request_type;
  using queue_type = std::vector<std::optional<value_type>>;
  queue_type WQ{DRAM_WQ_SIZE};
  queue_type RQ{DRAM_RQ_SIZE};

  // these values control when to send out a burst of writes
  constexpr static std::size_t DRAM_WRITE_HIGH_WM = ((DRAM_WQ_SIZE * 7) >> 3);         // 7/8th
  constexpr static std::size_t DRAM_WRITE_LOW_WM = ((DRAM_WQ_SIZE * 6) >> 3);          // 6/8th
  constexpr static std::size_t MIN_DRAM_WRITES_PER_SWITCH = ((DRAM_WQ_SIZE * 1) >> 2); // 1/4

  struct BANK_REQUEST {
    bool valid = false, row_buffer_hit = false;

    std::optional<std::size_t> open_row{};

    uint64_t event_cycle = 0;

    queue_type::iterator pkt;
  };

  using request_array_type = std::array<BANK_REQUEST, DRAM_RANKS * DRAM_BANKS>;
  request_array_type bank_request = {};
  request_array_type::iterator active_request = std::end(bank_request);

  bool write_mode = false;
  uint64_t dbus_cycle_available = 0;

  using stats_type = dram_stats;
  stats_type roi_stats, sim_stats;

  // Latencies
  const uint64_t tRP, tRCD, tCAS, DRAM_DBUS_TURN_AROUND_TIME, DRAM_DBUS_RETURN_TIME;
  const std::size_t ROWS, COLUMNS, RANKS, BANKS;

  DRAM_CHANNEL(int io_freq, double t_rp, double t_rcd, double t_cas, double turnaround, std::size_t rows, std::size_t columns, std::size_t ranks,
               std::size_t banks);

  void check_write_collision();
  void check_read_collision();
  long finish_dbus_request();
  void swap_write_mode();
  long populate_dbus();
  long schedule_packets();

  void initialize() final;
  long operate() final;
  void begin_phase() final;
  void end_phase(unsigned cpu) final;
  void print_deadlock() final;

  unsigned long get_rank(uint64_t address) const;
  unsigned long get_bank(uint64_t address) const;
  unsigned long get_row(uint64_t address) const;
  unsigned long get_column(uint64_t address) const;
};

class MEMORY_CONTROLLER : public champsim::operable
{
  using channel_type = champsim::channel;
  using request_type = typename channel_type::request_type;
  using response_type = typename channel_type::response_type;
  std::vector<channel_type*> queues;

  void initiate_requests();
  bool add_rq(const request_type& packet, champsim::channel* ul);
  bool add_wq(const request_type& packet);

public:
  std::vector<DRAM_CHANNEL> channels;

  MEMORY_CONTROLLER(double freq_scale, int io_freq, double t_rp, double t_rcd, double t_cas, double turnaround, std::vector<channel_type*>&& ul);

  void initialize() final;
  long operate() final;
  void begin_phase() final;
  void end_phase(unsigned cpu) final;
  void print_deadlock() final;

  [[nodiscard]] std::size_t size() const;

  unsigned long dram_get_channel(uint64_t address) const;
  unsigned long dram_get_rank(uint64_t address) const;
  unsigned long dram_get_bank(uint64_t address) const;
  unsigned long dram_get_row(uint64_t address) const;
  unsigned long dram_get_column(uint64_t address) const;
};

#endif
