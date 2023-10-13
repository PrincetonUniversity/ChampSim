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

#include "ooo_cpu.h"

#include <algorithm>
#include <cassert> // for assert
#include <chrono>
#include <cmath>
#include <iterator> // for end, begin, back_insert_iterator, empty
#include <fmt/chrono.h>
#include <fmt/core.h>

#include "cache.h"
#include "champsim.h"
#include "deadlock.h"
#include "instruction.h"
#include "trace_instruction.h" // for REG_STACK_POINTER, REG_FLAGS, REG_INS...
#include "util/span.h"

//#define BGODALA 0

std::chrono::seconds elapsed_time();

long O3_CPU::operate()
{
  long progress{0};

  progress += retire_rob();                    // retire
  progress += complete_inflight_instruction(); // finalize execution
  progress += execute_instruction();           // execute instructions
  progress += schedule_instruction();          // schedule instructions
  progress += handle_memory_return();          // finalize memory transactions
  progress += operate_lsq();                   // execute memory transactions

  progress += dispatch_instruction(); // dispatch
  progress += decode_instruction();   // decode
  progress += promote_to_decode();

  progress += fetch_instruction(); // fetch
  progress += check_dib();
  initialize_instruction();

  // heartbeat
  if (show_heartbeat && (num_retired >= next_print_instruction)) {
    auto heartbeat_instr{std::ceil(num_retired - last_heartbeat_instr)};
    auto heartbeat_cycle{std::ceil(current_cycle - last_heartbeat_cycle)};

    auto phase_instr{std::ceil(num_retired - begin_phase_instr)};
    auto phase_cycle{std::ceil(current_cycle - begin_phase_cycle)};

    fmt::print("Heartbeat CPU {} instructions: {} cycles: {} heartbeat IPC: {:.4g} cumulative IPC: {:.4g} (Simulation time: {:%H hr %M min %S sec})\n", cpu,
               num_retired, current_cycle, heartbeat_instr / heartbeat_cycle, phase_instr / phase_cycle, elapsed_time());
    next_print_instruction += STAT_PRINTING_PERIOD;

    last_heartbeat_instr = num_retired;
    last_heartbeat_cycle = current_cycle;
  }

  return progress;
}

void O3_CPU::initialize()
{
  // BRANCH PREDICTOR & BTB
  impl_initialize_branch_predictor();
  impl_initialize_btb();
}

void O3_CPU::begin_phase()
{
  begin_phase_instr = num_retired;
  begin_phase_cycle = current_cycle;

  // Record where the next phase begins
  stats_type stats;
  stats.name = "CPU " + std::to_string(cpu);
  stats.begin_instrs = num_retired;
  stats.begin_cycles = current_cycle;
  sim_stats = stats;
}

void O3_CPU::end_phase(unsigned finished_cpu)
{
  // Record where the phase ended (overwrite if this is later)
  sim_stats.end_instrs = num_retired;
  sim_stats.end_cycles = current_cycle;

  if (finished_cpu == this->cpu) {
    finish_phase_instr = num_retired;
    finish_phase_cycle = current_cycle;

    roi_stats = sim_stats;
  }
}

void O3_CPU::initialize_instruction()
{
  //auto instrs_to_read_this_cycle = std::min(FETCH_WIDTH, static_cast<long>(IFETCH_BUFFER_SIZE - std::size(IFETCH_BUFFER)));
  auto instrs_to_read_this_cycle = static_cast<long>(IFETCH_BUFFER_SIZE - std::size(IFETCH_BUFFER));


  if(fetch_resume_cycle == std::numeric_limits<uint64_t>::max()){
     sim_stats.fetch_mispred_block_cycles++; 
  }else if(sim_stats.fetch_mispred_block_cycles){
      //fmt::print("blocked cycles {}\n", sim_stats.fetch_mispred_block_cycles);
      sim_stats.fetch_mispred_block_cycles = 0;
  }

  //if(fetch_instr_id == retire_instr_id){
  if(fetch_instr_id && fetch_instr_id == exec_instr_id){

      prev_fetch_block = 0;
      while(!input_queue.empty() && input_queue.front().is_wrong_path){
          auto inst = input_queue.front();
          if(inst.is_wrong_path && std::size(inst.source_memory)){
              sim_stats.wrong_path_loads++;
          }
	      sim_stats.wrong_path_insts++;
          sim_stats.wrong_path_skipped++;
#ifdef BGODALA
          fmt::print("exec_flush ip: {:#x} wp: {}\n", inst.ip, inst.is_wrong_path);
#endif
          input_queue.pop_front();
      }
      if(!input_queue.empty() && !input_queue.front().is_wrong_path){
        in_wrong_path = false;
        flush_after = 0;
	    fetch_instr_id = 0;
        fetch_resume_cycle = current_cycle + BRANCH_MISPREDICT_PENALTY;
        //fmt::print("finished flushing\n");
      }else{
        //fmt::print("still finished flushing\n");
      }
  }

  if(flush_after){
        //fmt::print("flushing\n");
        while(!input_queue.empty() && input_queue.front().is_wrong_path){
            auto inst = input_queue.front();
            if(inst.is_wrong_path && std::size(inst.source_memory)){
                sim_stats.wrong_path_loads++;
            }
	        sim_stats.wrong_path_insts++;
            sim_stats.wrong_path_skipped++;
#ifdef BGODALA
            fmt::print("decode_flush ip: {:#x} wp: {}\n", inst.ip, inst.is_wrong_path);
#endif
            input_queue.pop_front();
        }
	if(!input_queue.empty() && !input_queue.front().is_wrong_path){
            in_wrong_path = false;
	    flush_after = 0;
            //fmt::print("finished flushing\n");
	}else{
            //fmt::print("still finished flushing\n");
	}
  }

  while (current_cycle >= fetch_resume_cycle && instrs_to_read_this_cycle > 0 && !std::empty(input_queue)) {
    instrs_to_read_this_cycle--;

      if(!enable_wrong_path){
        while(!input_queue.empty() && (input_queue.front().is_wrong_path || input_queue.front().is_prefetch)){
            auto inst = input_queue.front();
        if(inst.is_wrong_path && std::size(inst.source_memory)){
            sim_stats.wrong_path_loads++;
        }
	    sim_stats.wrong_path_insts++;
            sim_stats.wrong_path_skipped++;
#ifdef BGODALA
            fmt::print("ignore ip: {:#x} wp: {}\n", inst.ip, inst.is_wrong_path);
#endif
            input_queue.pop_front();
        }

        if(input_queue.empty()){
            fmt::print("INPUT Queue empty!");
           break;
        }
      }
    //if(last_branch){
    //    WPATH_MAP[last_branch].push_back(input_queue.front());    
    //}

    auto inst = input_queue.front();
  
    if(std::size(inst.source_memory)){
        sim_stats.loads++;
    }
    if(std::size(inst.destination_memory)){
        sim_stats.stores++;
    }

    //auto stop_fetch = do_init_instruction(input_queue.front());
    auto stop_fetch = false;
    #ifdef BGODALA
    fmt::print("inserting ip: {:#x} wp: {}\n", inst.ip, inst.is_wrong_path);
    #endif

    if (in_wrong_path && !inst.is_wrong_path){
        stop_fetch = true;
	    in_wrong_path = false;
        fetch_resume_cycle = std::numeric_limits<uint64_t>::max();
        #ifdef BGODALA
	    fmt::print("wrong path over at ip: {:#x}\n", inst.ip);
        #endif
	    break;
    }

    if(enable_wrong_path){
        if (inst.branch_mispredicted || inst.before_wrong_path){
            
            //if(inst.branch_msipredicted && inst.branch != BRANCH_CONDITIONAL){
            //    inst.branch_prediction = true;	
            //}

            in_wrong_path = true;
            restart = false;
        }

        if(inst.is_wrong_path && std::size(inst.source_memory)){
            sim_stats.wrong_path_loads++;
        }
    }else{
	    if(inst.branch_mispredicted || inst.before_wrong_path){
              in_wrong_path = false;
	          stop_fetch = true;
              fetch_resume_cycle = std::numeric_limits<uint64_t>::max();
              fetch_instr_id = inst.instr_id;
	    }
    }

    if(inst.branch_taken){
        //fmt::print("taken branch found at ip: {:#x}\n", inst.ip);
        stop_fetch = true;
    }

    if(inst.before_wrong_path){
#ifdef BGODALA
        fmt::print("before wrong path instr_id {}\n", inst.instr_id);
#endif
	    if(enable_wrong_path){
            fetch_instr_id = inst.instr_id;
	    }
    }

    if (stop_fetch) {
      instrs_to_read_this_cycle = 0;
    }

    if(inst.is_wrong_path){
        sim_stats.wrong_path_insts++;
    }

    // Update stats of non mispredicting branches here
    if(!inst.branch_mispredicted){
        update_branch_stats(input_queue.front());
    }

    // Add to IFETCH_BUFFER
    IFETCH_BUFFER.push_back(input_queue.front());
    input_queue.pop_front();

    IFETCH_BUFFER.back().event_cycle = current_cycle;
  }

}

void O3_CPU::update_branch_stats(ooo_model_instr& instr)
{
  if(instr.is_branch && !instr.is_wrong_path){
    sim_stats.total_branch_types.at(instr.branch)++;
    if(instr.branch_mispredicted){
      sim_stats.total_rob_occupancy_at_branch_mispredict += std::size(ROB);
      sim_stats.branch_type_misses.at(instr.branch)++;
    }
  }

}
namespace
{
void do_stack_pointer_folding(ooo_model_instr& arch_instr)
{
  // The exact, true value of the stack pointer for any given instruction can usually be determined immediately after the instruction is decoded without
  // waiting for the stack pointer's dependency chain to be resolved.
  bool writes_sp = (std::count(std::begin(arch_instr.destination_registers), std::end(arch_instr.destination_registers), champsim::REG_STACK_POINTER) > 0);
  if (writes_sp) {
    // Avoid creating register dependencies on the stack pointer for calls, returns, pushes, and pops, but not for variable-sized changes in the
    // stack pointer position. reads_other indicates that the stack pointer is being changed by a variable amount, which can't be determined before
    // execution.
    bool reads_other =
        (std::count_if(std::begin(arch_instr.source_registers), std::end(arch_instr.source_registers),
                       [](auto r) { return r != champsim::REG_STACK_POINTER && r != champsim::REG_FLAGS && r != champsim::REG_INSTRUCTION_POINTER; })
         > 0);
    if ((arch_instr.is_branch) || !(std::empty(arch_instr.destination_memory) && std::empty(arch_instr.source_memory)) || (!reads_other)) {
      auto nonsp_end = std::remove(std::begin(arch_instr.destination_registers), std::end(arch_instr.destination_registers), champsim::REG_STACK_POINTER);
      arch_instr.destination_registers.erase(nonsp_end, std::end(arch_instr.destination_registers));
    }
  }
}
} // namespace

bool O3_CPU::do_predict_branch(ooo_model_instr& arch_instr)
{
  bool stop_fetch = false;

  // handle branch prediction for all instructions as at this point we do not know if the instruction is a branch
  sim_stats.total_branch_types.at(arch_instr.branch)++;
  auto [predicted_branch_target, always_taken] = impl_btb_prediction(arch_instr.ip, arch_instr.branch);
  arch_instr.branch_prediction = impl_predict_branch(arch_instr.ip, predicted_branch_target, always_taken, arch_instr.branch) || always_taken;
  if (!arch_instr.branch_prediction) {
    predicted_branch_target = 0;
  }

  if (arch_instr.is_branch) {
    if constexpr (champsim::debug_print) {
      fmt::print("[BRANCH] instr_id: {} ip: {:#x} taken: {}\n", arch_instr.instr_id, arch_instr.ip, arch_instr.branch_taken);
    }

    // call code prefetcher every time the branch predictor is used
    l1i->impl_prefetcher_branch_operate(arch_instr.ip, arch_instr.branch, predicted_branch_target);

    if (predicted_branch_target != arch_instr.branch_target
        || (((arch_instr.branch == BRANCH_CONDITIONAL) || (arch_instr.branch == BRANCH_OTHER))
            && arch_instr.branch_taken != arch_instr.branch_prediction)) { // conditional branches are re-evaluated at decode when the target is computed
      sim_stats.total_rob_occupancy_at_branch_mispredict += std::size(ROB);
      sim_stats.branch_type_misses.at(arch_instr.branch)++;
      if (!warmup) {
        fetch_resume_cycle = std::numeric_limits<uint64_t>::max();
        stop_fetch = true;
        arch_instr.branch_mispredicted = true;
      }
    } else {
      stop_fetch = arch_instr.branch_taken; // if correctly predicted taken, then we can't fetch anymore instructions this cycle
    }

    if(arch_instr.branch_mispredicted){
        auto wpath = arch_instr.branch_taken ? arch_instr.ip : arch_instr.branch_target;
	    fmt::print("WRONG PATH: branch: {:#x} wpath: {:#x}\n", arch_instr.ip, wpath);
        for(auto wp_inst: WPATH_MAP[wpath]){
	    fmt::print("{:#x}\n", wp_inst.ip);
	}	
    }

    fmt::print("[BRANCH] instr_id: {} ip: {:#x} target: {:#x} taken: {}\n", arch_instr.instr_id, arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken);
    impl_update_btb(arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken, arch_instr.branch);
    impl_last_branch_result(arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken, arch_instr.branch);
    //fmt::print("[BRANCH] instr_id: {} ip: {:#x} target: {:#x} taken: {}\n", arch_instr.instr_id, arch_instr.ip, arch_instr.branch_target, arch_instr.branch_taken);

    if(arch_instr.branch_taken){
      last_branch = arch_instr.branch_target;
    }else{
      last_branch = arch_instr.ip;
    }
    WPATH_MAP[last_branch].clear();
  }

  //if (arch_instr.is_squash_after ||
  //    arch_instr.is_serializing ||
  //    arch_instr.is_serialize_after ||
  //    arch_instr.is_serialize_before ||
  //    arch_instr.is_write_barrier ||
  //    arch_instr.is_read_barrier ||
  //    arch_instr.is_non_spec){
  //  //fmt::print("[SQUASH AFTER] instr_id: {} ip: {:#x} taken: {}\n", arch_instr.instr_id, arch_instr.ip, arch_instr.branch_taken);
  //  fetch_resume_cycle = std::numeric_limits<uint64_t>::max();
  //  stop_fetch = true;
  //}

  return stop_fetch;
}

bool O3_CPU::do_init_instruction(ooo_model_instr& arch_instr)
{
  // fast warmup eliminates register dependencies between instructions branch predictor, cache contents, and prefetchers are still warmed up
  if (warmup) {
    arch_instr.source_registers.clear();
    arch_instr.destination_registers.clear();
  }

  ::do_stack_pointer_folding(arch_instr);
  return do_predict_branch(arch_instr);
}

long O3_CPU::check_dib()
{
  // scan through IFETCH_BUFFER to find instructions that hit in the decoded instruction buffer
  auto begin = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), [](const ooo_model_instr& x) { return !x.dib_checked; });
  auto [window_begin, window_end] = champsim::get_span(begin, std::end(IFETCH_BUFFER), FETCH_WIDTH*10);
  std::for_each(window_begin, window_end, [this](auto& ifetch_entry) { this->do_check_dib(ifetch_entry); });
  return std::distance(window_begin, window_end);
}

void O3_CPU::do_check_dib(ooo_model_instr& instr)
{
  // Check DIB to see if we recently fetched this line
  if (auto dib_result = DIB.check_hit(instr.ip); dib_result) {
    // The cache line is in the L0, so we can mark this as complete
    instr.fetch_completed = true;

    // Also mark it as decoded
    instr.decoded = true;

    // It can be acted on immediately
    instr.event_cycle = current_cycle;
  }

  instr.dib_checked = true;
}

long O3_CPU::fetch_instruction()
{
  long progress{0};

  // Fetch a single cache line
  auto fetch_ready = [](const ooo_model_instr& x) {
    return x.dib_checked && !x.fetch_issued;
  };

  // Find the chunk of instructions in the block
  auto no_match_ip = [](const auto& lhs, const auto& rhs) {
    return (lhs.ip >> LOG2_BLOCK_SIZE) != (rhs.ip >> LOG2_BLOCK_SIZE);
  };

  bool failed = false;

  auto l1i_req_begin = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), fetch_ready);
  for (auto to_read = L1I_BANDWIDTH; to_read > 0 && l1i_req_begin != std::end(IFETCH_BUFFER); --to_read) {
    auto l1i_req_end = std::adjacent_find(l1i_req_begin, std::end(IFETCH_BUFFER), no_match_ip);
    if (l1i_req_end != std::end(IFETCH_BUFFER)) {
      l1i_req_end = std::next(l1i_req_end); // adjacent_find returns the first of the non-equal elements
    }

    // Issue to L1I
    auto success = do_fetch_instruction(l1i_req_begin, l1i_req_end);
    if (success) {
      std::for_each(l1i_req_begin, l1i_req_end, [](auto& x) { x.fetch_issued = true; });
      //prev_fetch_block = l1i_req_begin->ip >> LOG2_BLOCK_SIZE;
      ++progress;
    }else{
        failed = true;
        sim_stats.fetch_failed_events++;
    }

    l1i_req_begin = std::find_if(l1i_req_end, std::end(IFETCH_BUFFER), fetch_ready);
  }

  if(progress == 0){
      sim_stats.fetch_idle_cycles++;
      if(!IFETCH_BUFFER.empty()){
          if(!IFETCH_BUFFER.back().fetch_issued){
              sim_stats.fetch_buffer_not_empty++; 
          }
      }
  }

  if(fetch_resume_cycle == std::numeric_limits<uint64_t>::max()){
      sim_stats.fetch_blocked_cycles++;
  }

  return progress;
}

bool O3_CPU::do_fetch_instruction(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end)
{
  //BGODALA
  //fmt::print("[IFETCH] {} instr_id: {} fid: {} ip: {:#x} block: {:#x} cycle: {}\n", __func__, begin->instr_id, begin->fetch_instr_id, begin->ip, (begin->ip >> LOG2_BLOCK_SIZE) << LOG2_BLOCK_SIZE, current_cycle);
  if(prev_fetch_block && begin->ip >> LOG2_BLOCK_SIZE == prev_fetch_block){
    return true;
  }
  //if(!L1I_bus.lower_level->RQ.empty() && begin->ip >> LOG2_BLOCK_SIZE == L1I_bus.lower_level->RQ.back().ip >> LOG2_BLOCK_SIZE){
  //  std::transform(begin, end, std::back_inserter(L1I_bus.lower_level->RQ.back().instr_depend_on_me), [](const auto& instr) { return instr.instr_id; });
  //  return true;
  //}

  CacheBus::request_type fetch_packet;
  fetch_packet.v_address = begin->ip;
  fetch_packet.instr_id = begin->instr_id;
  fetch_packet.ip = begin->ip;

  last_fetch_packet = fetch_packet;

  std::transform(begin, end, std::back_inserter(fetch_packet.instr_depend_on_me), [](const auto& instr) { return instr.instr_id; });

  if constexpr (champsim::debug_print) {
    fmt::print("[IFETCH] {} instr_id: {} ip: {:#x} dependents: {} event_cycle: {}\n", __func__, begin->instr_id, begin->ip,
               std::size(fetch_packet.instr_depend_on_me), begin->event_cycle);
  }


  return L1I_bus.issue_read(fetch_packet);
}

long O3_CPU::promote_to_decode()
{
  auto available_fetch_bandwidth = std::min<long>(FETCH_WIDTH, static_cast<long>(DECODE_BUFFER_SIZE - std::size(DECODE_BUFFER)));
  auto [window_begin, window_end] = champsim::get_span_p(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), available_fetch_bandwidth,
                                                         [cycle = current_cycle](const auto& x) { return x.fetch_completed && x.event_cycle <= cycle; });
  long progress{std::distance(window_begin, window_end)};

  std::for_each(window_begin, window_end,
                [cycle = current_cycle, lat = DECODE_LATENCY, warmup = warmup](auto& x) { return x.event_cycle = cycle + ((warmup || x.decoded) ? 0 : lat); });
  std::move(window_begin, window_end, std::back_inserter(DECODE_BUFFER));
  IFETCH_BUFFER.erase(window_begin, window_end);

  return progress;
}

long O3_CPU::decode_instruction()
{
  auto available_decode_bandwidth = std::min<long>(DECODE_WIDTH, static_cast<long>(DISPATCH_BUFFER_SIZE - std::size(DISPATCH_BUFFER)));
  auto [window_begin, window_end] = champsim::get_span_p(std::begin(DECODE_BUFFER), std::end(DECODE_BUFFER), available_decode_bandwidth,
                                                         [cycle = current_cycle](const auto& x) { return x.event_cycle <= cycle; });
  long progress{std::distance(window_begin, window_end)};

  uint64_t flushed = 0;
  // Send decoded instructions to dispatch
  std::for_each(window_begin, window_end, [&, this](auto& db_entry) {
    this->do_dib_update(db_entry);

    // Resume fetch
    if (db_entry.branch_mispredicted && !db_entry.is_wrong_path) {
      // These branches detect the misprediction at decode
      if ((db_entry.branch == BRANCH_DIRECT_JUMP) 
          || (db_entry.branch == BRANCH_DIRECT_CALL)
        //  || (((db_entry.branch == BRANCH_CONDITIONAL) || (db_entry.branch == BRANCH_OTHER)) 
	//	  && db_entry.branch_taken == db_entry.branch_prediction)) {
      ){
        // clear the branch_mispredicted bit so we don't attempt to resume fetch again at execute
        //db_entry.branch_mispredicted = 0;
        // pay misprediction penalty
        this->fetch_resume_cycle = this->current_cycle + BRANCH_MISPREDICT_PENALTY;
        
        //update branch stats here
        update_branch_stats(db_entry);
	    prev_fetch_block = 0;
	    restart = true;
	    //fmt::print("flush DECODE_BUFFER and IFETCH_BUFFER here at ip: {:#x}\n", db_entry.ip);
	    flushed = db_entry.instr_id;
	    db_entry.squashed = true;
      }
    }

    // Add to dispatch
    db_entry.event_cycle = this->current_cycle + (this->warmup ? 0 : this->DISPATCH_LATENCY);
  });

  if(flushed){
    window_end = find_if(std::begin(DECODE_BUFFER), std::end(DECODE_BUFFER), [id = flushed](auto &x){ return id == x.instr_id;});
  }
  std::move(window_begin, window_end, std::back_inserter(DISPATCH_BUFFER));
  DECODE_BUFFER.erase(window_begin, window_end);

  if(flushed){
        flush_after = flushed;
#ifdef BGODALA
	fmt::print("flush DECODE_BUFFER and IFETCH_BUFFER " 
		   "after instr_id: {} DECODE_BUFFER size {} "
		   "IFETCH_BUFFER size {}\n", flushed, 
		   DECODE_BUFFER.size(), IFETCH_BUFFER.size());
	for_each(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER),
			[](auto inst){fmt::print("ip: {:#x} instr_id: {} wp: {}\n", 
				inst.ip, inst.instr_id, inst.is_wrong_path);
			});
#endif
	DECODE_BUFFER.clear();
	IFETCH_BUFFER.clear();
  }

  if (progress == 0){
      sim_stats.decode_idle_cycles++;
  }

  return progress;
}

void O3_CPU::do_dib_update(const ooo_model_instr& instr) { DIB.fill(instr.ip); }

long O3_CPU::dispatch_instruction()
{
  auto available_dispatch_bandwidth = DISPATCH_WIDTH;

  if(((std::size_t)std::count_if(std::begin(LQ), std::end(LQ), [](const auto& lq_entry) { return !lq_entry.has_value();  })) == 0){
      sim_stats.lq_full_events++;
  }
  
  if(std::size(SQ) == SQ_SIZE){
      sim_stats.sq_full_events++;
  }

  // dispatch DISPATCH_WIDTH instructions into the ROB
  while (available_dispatch_bandwidth > 0 && !std::empty(DISPATCH_BUFFER) && DISPATCH_BUFFER.front().event_cycle < current_cycle && std::size(ROB) != ROB_SIZE
         && ((std::size_t)std::count_if(std::begin(LQ), std::end(LQ), [](const auto& lq_entry) { return !lq_entry.has_value(); })
             >= std::size(DISPATCH_BUFFER.front().source_memory))
         && ((std::size(DISPATCH_BUFFER.front().destination_memory) + std::size(SQ)) <= SQ_SIZE)) {

    ROB.push_back(std::move(DISPATCH_BUFFER.front()));
    auto &inst = ROB.back();

    if( (inst.is_wrong_path || inst.is_prefetch) && !inst.num_mem_ops()){
        inst.executed = true;
        //inst.scheduled = true;
        //inst.completed = true;
        available_dispatch_bandwidth++;
    }
    DISPATCH_BUFFER.pop_front();
    do_memory_scheduling(ROB.back());

    available_dispatch_bandwidth--;
  }

  if((DISPATCH_WIDTH - available_dispatch_bandwidth) == 0){
      sim_stats.dispatch_idle_cycles++;
  }

  return DISPATCH_WIDTH - available_dispatch_bandwidth;
}

long O3_CPU::schedule_instruction()
{
  auto search_bw = SCHEDULER_SIZE;
  int progress{0};
  for (auto rob_it = std::begin(ROB); rob_it != std::end(ROB) && search_bw > 0; ++rob_it) {
    if (!rob_it->scheduled) {
      do_scheduling(*rob_it);
      ++progress;
    }

    if (!rob_it->executed) {
      --search_bw;
    }
  }

  if(progress == 0){
      sim_stats.sched_idle_cycles++;
      if(!ROB.empty() && !ROB.back().scheduled){
          sim_stats.sched_none_cycles++;
      }
  }

  return progress;
}

void O3_CPU::do_scheduling(ooo_model_instr& instr)
{

  //fmt::print("sched instr_id {}\n", instr.instr_id);
  // Don't track dependeces for instructions in the wrong path
  // Hack to let only memory instructions be executed
  if(instr.is_wrong_path || instr.is_prefetch){
    instr.scheduled = true;
    //instr.executed = true;
    //instr.completed = true;
    instr.event_cycle = current_cycle; 
    return;
  }
  // Mark register dependencies
  for (auto src_reg : instr.source_registers) {
    if (!std::empty(reg_producers.at(src_reg))) {
      ooo_model_instr& prior = *reg_producers.at(src_reg).back();
      if (prior.registers_instrs_depend_on_me.empty() || prior.registers_instrs_depend_on_me.back().get().instr_id != instr.instr_id) {
          if (prior.instr_id > instr.instr_id){
              fmt::print("something is wrong! src_reg: {} prior: instr_id {} cur: instr_id {}\n", src_reg, prior.instr_id, instr.instr_id);
              print_deadlock();
          }
        prior.registers_instrs_depend_on_me.emplace_back(instr);
        instr.num_reg_dependent++;
      }
    }
  }

  for (auto dreg : instr.destination_registers) {
    //auto begin = std::begin(reg_producers.at(dreg));
    //auto end = std::end(reg_producers.at(dreg));
    //auto ins = std::lower_bound(begin, end, instr, [](const ooo_model_instr& lhs, const ooo_model_instr& rhs) { return lhs.instr_id < rhs.instr_id; });
      reg_producers.at(dreg).clear();
    reg_producers.at(dreg).push_back(&instr);
    //fmt::print("dreg {} instr_id {}\n", dreg, instr.instr_id);
  }

  instr.scheduled = true;
  instr.event_cycle = current_cycle + (warmup ? 0 : SCHEDULING_LATENCY);
}

long O3_CPU::execute_instruction()
{
  auto exec_bw = EXEC_WIDTH;
  for (auto rob_it = std::begin(ROB); rob_it != std::end(ROB) && exec_bw > 0; ++rob_it) {
    if (rob_it->scheduled && !rob_it->executed && rob_it->num_reg_dependent == 0 && rob_it->event_cycle <= current_cycle) {
      do_execution(*rob_it);
      --exec_bw;
    }
  }

  if( (EXEC_WIDTH - exec_bw) == 0){
      sim_stats.execute_idle_cycles++;
      if(!ROB.empty()){
          sim_stats.execute_none_cycles++;
          auto exec_it = std::find_if(std::begin(ROB), std::end(ROB), [](auto &x){ return x.scheduled && !x.executed; });

          if( exec_it != std::end(ROB)){
            sim_stats.execute_pending_cycles++; 
          }

          if(!ROB.front().executed){
              sim_stats.execute_head_not_ready++;
          }
          if(ROB.front().executed && !ROB.front().completed){
              //fmt::print("ip: {:#x} instr_id {} cycle {}\n", ROB.front().ip, ROB.front().instr_id, current_cycle);
              sim_stats.execute_head_not_completed++;

              if(std::size(ROB.front().source_memory)){
                  sim_stats.execute_load_blocked_cycles++;
              }
          }
      }
  }
  return EXEC_WIDTH - exec_bw;
}

void O3_CPU::do_execution(ooo_model_instr& instr)
{
  instr.executed = true;
  instr.event_cycle = current_cycle + (warmup ? 0 : EXEC_LATENCY);

  // Mark LQ entries as ready to translate
  for (auto& lq_entry : LQ) {
    if (lq_entry.has_value() && lq_entry->instr_id == instr.instr_id) {
        sim_stats.loads_executed++;
      lq_entry->event_cycle = current_cycle + (warmup ? 0 : EXEC_LATENCY);
    }
  }

  // Mark SQ entries as ready to translate
  for (auto& sq_entry : SQ) {
    if (sq_entry.instr_id == instr.instr_id) {
      sq_entry.event_cycle = current_cycle + (warmup ? 0 : EXEC_LATENCY);
      for (std::optional<LSQ_ENTRY>& dependent : sq_entry.lq_depend_on_me) {
        if(!dependent.has_value()){
            continue;
        }
        //assert(dependent.has_value()); // LQ entry is still allocated
        //assert(dependent->producer_id == sq_entry.instr_id);

        if(dependent->producer_id == sq_entry.instr_id){
            //fmt::print("store finished but found dependent load!");
            dependent->finish(std::begin(ROB), std::end(ROB));
            dependent.reset();
        }
      }
    }
  }

  if constexpr (champsim::debug_print) {
    fmt::print("[ROB] {} instr_id: {} event_cycle: {}\n", __func__, instr.instr_id, instr.event_cycle);
  }
}

void O3_CPU::do_memory_scheduling(ooo_model_instr& instr)
{
  // load
  for (auto& smem : instr.source_memory) {
    auto q_entry = std::find_if_not(std::begin(LQ), std::end(LQ), [](const auto& lq_entry) { return lq_entry.has_value(); });
    assert(q_entry != std::end(LQ));
    q_entry->emplace(instr.instr_id, smem, instr.ip, instr.asid, instr.is_wrong_path); // add it to the load queue

    // Check for forwarding
    auto sq_it = std::max_element(std::begin(SQ), std::end(SQ), [smem](const auto& lhs, const auto& rhs) {
      return lhs.virtual_address != smem || (rhs.virtual_address == smem && lhs.instr_id < rhs.instr_id);
    });
    if (sq_it != std::end(SQ) && sq_it->virtual_address == smem && !instr.is_wrong_path) {
      if (sq_it->fetch_issued) { // Store already executed
        (*q_entry)->finish(instr);
        q_entry->reset();
      } else {
        assert(sq_it->instr_id < instr.instr_id);      // The found SQ entry is a prior store
        sq_it->lq_depend_on_me.emplace_back(*q_entry); // Forward the load when the store finishes
        (*q_entry)->producer_id = sq_it->instr_id;     // The load waits on the store to finish

        if constexpr (champsim::debug_print) {
          fmt::print("[DISPATCH] {} instr_id: {} waits on: {}\n", __func__, instr.instr_id, sq_it->event_cycle);
        }
      }

      //Update wrong_path load stats here since entry will not be counted in lsq operate stage
      if(instr.is_wrong_path){
          sim_stats.wrong_path_loads_executed++;
      }
    }
  }

  // store
  for (auto& dmem : instr.destination_memory) {
    SQ.emplace_back(instr.instr_id, dmem, instr.ip, instr.asid, instr.is_wrong_path); // add it to the store queue
  }

  if constexpr (champsim::debug_print) {
    fmt::print("[DISPATCH] {} instr_id: {} loads: {} stores: {}\n", __func__, instr.instr_id, std::size(instr.source_memory),
               std::size(instr.destination_memory));
  }
}

long O3_CPU::operate_lsq()
{
  auto store_bw = SQ_WIDTH;

  const auto complete_id = std::empty(ROB) ? std::numeric_limits<uint64_t>::max() : ROB.front().instr_id;
  auto do_complete = [cycle = current_cycle, complete_id, this](const auto& x) {
    return x.instr_id < complete_id && x.event_cycle <= cycle && this->do_complete_store(x);
  };

  auto unfetched_begin = std::partition_point(std::begin(SQ), std::end(SQ), [](const auto& x) { return x.fetch_issued; });
  auto [fetch_begin, fetch_end] = champsim::get_span_p(unfetched_begin, std::end(SQ), store_bw,
                                                       [cycle = current_cycle](const auto& x) { return !x.fetch_issued && x.event_cycle <= cycle; });
  store_bw -= std::distance(fetch_begin, fetch_end);
  std::for_each(fetch_begin, fetch_end, [cycle = current_cycle, this](auto& sq_entry) {
    this->do_finish_store(sq_entry);
    sq_entry.fetch_issued = true;
    sq_entry.event_cycle = cycle;
  });

  auto [complete_begin, complete_end] = champsim::get_span_p(std::cbegin(SQ), std::cend(SQ), store_bw, do_complete);
  SQ.erase(complete_begin, complete_end);

  auto load_bw = LQ_WIDTH;

  //while(load_bw > 0){
  //  LSQ_ENTRY *smallest_lq_entry = NULL; 
  //  for (auto& lq_entry : LQ) {

  //    if (lq_entry.has_value() && lq_entry->producer_id == std::numeric_limits<uint64_t>::max() && !lq_entry->fetch_issued
  //        && lq_entry->event_cycle < current_cycle) {
  //        if(smallest_lq_entry != NULL){
  //            if(lq_entry->event_cycle < smallest_lq_entry->event_cycle){
  //                smallest_lq_entry = &(*lq_entry);
  //            }
  //        }else{
  //            smallest_lq_entry = &(*lq_entry);
  //        }
  //    }

  //  }
  //  if(smallest_lq_entry == NULL){
  //      break;
  //  }
  //  auto success = execute_load(*smallest_lq_entry);
  //  if (success) {
  //    --load_bw;
  //    (smallest_lq_entry)->fetch_issued = true;
  //    if(smallest_lq_entry->is_wrong_path){
  //        sim_stats.wrong_path_loads_executed++;
  //    }
  //  }else{
  //    break;
  //  }
  //}
  for (auto& lq_entry : LQ) {

    if (load_bw > 0 && lq_entry.has_value() && lq_entry->producer_id == std::numeric_limits<uint64_t>::max() && !lq_entry->fetch_issued
        && lq_entry->event_cycle < current_cycle) {
      auto success = execute_load(*lq_entry);
      if (success) {
          sim_stats.loads_success++;
        --load_bw;
        lq_entry->fetch_issued = true;
        //lq_entry->finish(std::begin(ROB), std::end(ROB));
        //lq_entry.reset();

        if(lq_entry->is_wrong_path){
            sim_stats.wrong_path_loads_executed++;
        }

        //fmt::print("LOAD Issued for address {:#x} instr_id {} ip {:#x} is_wrong_path {}\n", lq_entry->virtual_address, lq_entry->instr_id, lq_entry->ip, lq_entry->is_wrong_path);
      }
    }
  }

  return (SQ_WIDTH - store_bw) + (LQ_WIDTH - load_bw);
}

void O3_CPU::do_finish_store(const LSQ_ENTRY& sq_entry)
{
  sq_entry.finish(std::begin(ROB), std::end(ROB));

  // Release dependent loads
  for (std::optional<LSQ_ENTRY>& dependent : sq_entry.lq_depend_on_me) {
    if(!dependent.has_value()){
        continue;
    }
    //assert(dependent.has_value()); // LQ entry is still allocated
    //assert(dependent->producer_id == sq_entry.instr_id);

    if(dependent->producer_id == sq_entry.instr_id){
        dependent->finish(std::begin(ROB), std::end(ROB));
        dependent.reset();
    }
  }
}

bool O3_CPU::do_complete_store(const LSQ_ENTRY& sq_entry)
{
  if(sq_entry.is_wrong_path){
    print_deadlock();
    std::cout <<std::flush;
  }
  assert(!sq_entry.is_wrong_path && "Do not issue store for wrong path entry\n");

  CacheBus::request_type data_packet;
  data_packet.v_address = sq_entry.virtual_address;
  data_packet.instr_id = sq_entry.instr_id;
  data_packet.ip = sq_entry.ip;

  if constexpr (champsim::debug_print) {
    fmt::print("[SQ] {} instr_id: {} vaddr: {:x}\n", __func__, data_packet.instr_id, data_packet.v_address);
  }

  return L1D_bus.issue_write(data_packet);
}

bool O3_CPU::execute_load(const LSQ_ENTRY& lq_entry)
{
  CacheBus::request_type data_packet;
  data_packet.v_address = lq_entry.virtual_address;
  data_packet.instr_id = lq_entry.instr_id;
  data_packet.ip = lq_entry.ip;

  if constexpr (champsim::debug_print) {
    fmt::print("[LQ] {} instr_id: {} vaddr: {:#x}\n", __func__, data_packet.instr_id, data_packet.v_address);
  }

  return L1D_bus.issue_read(data_packet);
}

void O3_CPU::do_complete_execution(ooo_model_instr& instr)
{

  for (auto dreg : instr.destination_registers) {
      if(!reg_producers.at(dreg).empty() && reg_producers.at(dreg).back()->instr_id == instr.instr_id){
          reg_producers.at(dreg).clear();
      }
    //auto begin = std::begin(reg_producers.at(dreg));
    //auto end = std::end(reg_producers.at(dreg));
    //auto elem = std::find_if(begin, end, [id = instr.instr_id](ooo_model_instr& x) { return x.instr_id == id; });
    //assert(elem != end && "elem shoud exist completing instruction before scheduling\n");
    //if(elem != end){
    //  reg_producers.at(dreg).erase(elem);
    //}
  }

  instr.completed = true;

  for (ooo_model_instr& dependent : instr.registers_instrs_depend_on_me) {
    dependent.num_reg_dependent--;


    assert(dependent.num_reg_dependent >= 0);

    if (dependent.num_reg_dependent == 0) {
      dependent.scheduled = true;
    }
  }

  if (instr.branch_mispredicted && !instr.is_wrong_path && !instr.squashed) {
    update_branch_stats(instr);
    fetch_resume_cycle = current_cycle + BRANCH_MISPREDICT_PENALTY;
    prev_fetch_block = 0;
    restart = true;

    //fmt::print("C[EXEC]: cycle {} ip: {:x} branch {}\n", current_cycle, instr.ip, instr.branch); 
#ifdef BGODALA
    fmt::print("flush ROB cycle {} instr_id {} ROB_SIZE {}\n", current_cycle, instr.instr_id, ROB.size());
#endif
  }

      if(!instr.is_wrong_path && (instr.before_wrong_path || instr.branch_mispredicted)  &&  !instr.squashed && instr.instr_id == fetch_instr_id){
#ifdef BGODALA
        fmt::print("flush ROB cycle {} instr_id {} ROB_SIZE {} fetch_instr_id {}\n", current_cycle, instr.instr_id, ROB.size(), fetch_instr_id);
#endif
        if(instr.before_wrong_path && !instr.is_branch){
          sim_stats.non_branch_squashes++;
        }
        instr.squashed = true;

        for_each(std::begin(ROB), std::end(ROB), [id = instr.instr_id, this](auto &x) { 
            	    if(x.instr_id > id) { 
            	      std::cout <<std::flush;
            	      assert(x.is_wrong_path && "Must be wrong path instruction\n");

		      if(!x.is_prefetch && x.executed)
		          sim_stats.wrong_path_insts_executed++;
#ifdef BGODALA
                      if(!x.executed && x.num_mem_ops()){
            	          fmt::print("FLUSH instr_id: {} is_wrong_path: {} mem_ops: {} ", x.instr_id, x.is_wrong_path, x.num_mem_ops());
                          for (auto& smem : x.source_memory) {
                              fmt::print("{:#x} ",smem);
                          }
                          fmt::print("\n");
                      }
#endif
            	      x.scheduled = true; 
            	      x.executed = true; 
            	      //do_complete_execution(x);
            	      x.completed = true;
#ifdef BGODALA
            	      fmt::print("FLUSH instr_id: {} is_wrong_path: {}\n", x.instr_id, x.is_wrong_path);  
#endif
            	      std::cout <<std::flush;
                      //for (auto dreg : x.destination_registers) {
                      //  auto begin = std::begin(reg_producers.at(dreg));
                      //  auto end = std::end(reg_producers.at(dreg));
                      //  auto elem = std::find_if(begin, end, [id = x.instr_id](ooo_model_instr& y) { return y.instr_id == id; });
                      //  if(elem != end){
                      //    reg_producers.at(dreg).erase(elem);
                      //  }
                      //}
             	    } else {
		      
		      auto first_wp_inst = find_if(std::begin(x.registers_instrs_depend_on_me), std::end(x.registers_instrs_depend_on_me), [id = id](auto &y){ auto &z = y.get(); return z.instr_id > id; });

		      //assert(first_wp_inst.is_wrong_path && "This should wrong path instruction");

		      for_each(first_wp_inst, std::end(x.registers_instrs_depend_on_me), [](auto &y) { assert(y.get().is_wrong_path && "Should be wrong path inst\n"); });
		      x.registers_instrs_depend_on_me.erase(first_wp_inst, std::end(x.registers_instrs_depend_on_me));

		    } 
	} );
	auto sq_it_squash_begin = find_if(std::begin(SQ), std::end(SQ), [id = instr.instr_id](auto &x){ return x.instr_id > id;});
	if (sq_it_squash_begin != SQ.end()){
	    for_each(sq_it_squash_begin, std::end(SQ), [](auto &x) { 
			    //fmt::print("Erasing SQ entry instr_id {} \n", x.instr_id);
			    assert(x.is_wrong_path && "This should be wrong path entry");
			    });
	    SQ.erase(sq_it_squash_begin, std::end(SQ));
	}

	for_each(std::begin(LQ), std::end(LQ), [id = instr.instr_id](auto &x) { 
			if( x->instr_id > id) 
			{
			  //fmt::print("Erasing LQ entry instr_id {} \n", x->instr_id);
			  assert(x->is_wrong_path && "This should be wrong path entry\n");
			  x.reset();
			} 
		});
        auto rob_it = find_if(std::begin(ROB), std::end(ROB), [id = instr.instr_id, this](auto &x) { return x.instr_id > id;});
	
        if (rob_it != std::end(ROB)){
            ROB.erase(rob_it, std::end(ROB));
        }
        //if(std::size(IFETCH_BUFFER)){
        //    fmt::print("IFETCH_BUFFER flush begin\n");
        //    for_each(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER), [](auto &x){
        //        if(!x.executed && x.num_mem_ops()){
        //            fmt::print("IFETCH_BUFFER flushed instr_id: {} is_wrong_path: {} mem_ops: {} ", x.instr_id, x.is_wrong_path, x.num_mem_ops());
        //            for (auto& smem : x.source_memory) {
        //                fmt::print("{:#x} ",smem);
        //            }
        //            fmt::print("\n");
        //        }
        //        });
        //    fmt::print("IFETCH_BUFFER flush end\n");
        //}
        //flush ROB, DISPATH_BUFFER, DECODE_BUFFER, IFETCH_BUFFER
        DISPATCH_BUFFER.clear();
        DECODE_BUFFER.clear();
        IFETCH_BUFFER.clear();

        exec_instr_id = instr.instr_id;
  }
}

long O3_CPU::complete_inflight_instruction()
{
  // update ROB entries with completed executions
  auto complete_bw = EXEC_WIDTH;
  for (auto rob_it = std::begin(ROB); rob_it != std::end(ROB) && complete_bw > 0; ++rob_it) {
    if (rob_it->scheduled && rob_it->executed && !rob_it->completed && (rob_it->event_cycle <= current_cycle) && rob_it->completed_mem_ops == rob_it->num_mem_ops()) {
      do_complete_execution(*rob_it);
      if(!rob_it->is_wrong_path && !rob_it->is_prefetch)
        --complete_bw;
    }
  }

  return EXEC_WIDTH - complete_bw;
}

long O3_CPU::handle_memory_return()
{
  long progress{0};

  for (auto l1i_bw = FETCH_WIDTH, to_read = L1I_BANDWIDTH; l1i_bw > 0 && to_read > 0 && !L1I_bus.lower_level->returned.empty(); --to_read) {
    auto& l1i_entry = L1I_bus.lower_level->returned.front();

    //while (l1i_bw > 0) {
    //  auto fetched = std::find_if(std::begin(IFETCH_BUFFER), std::end(IFETCH_BUFFER),
    //                              [fid = l1i_entry.instr_id](const auto& x) { return x.fetch_instr_id == fid && !x.fetch_completed; });
    for (auto fetched = std::begin(IFETCH_BUFFER) ; fetched != std::end(IFETCH_BUFFER); fetched++){

      //if(fetched->fetch_instr_id > l1i_entry.instr_id)
      //  continue;

      if(fetched != std::end(IFETCH_BUFFER) && (fetched->ip >> LOG2_BLOCK_SIZE) == (l1i_entry.v_address >> LOG2_BLOCK_SIZE) && !fetched->fetch_issued){
        prev_fetch_block = 0;
      }
      if (fetched != std::end(IFETCH_BUFFER) && (fetched->ip >> LOG2_BLOCK_SIZE) == (l1i_entry.v_address >> LOG2_BLOCK_SIZE) && fetched->fetch_issued) {
        fetched->fetch_completed = true;
        //--l1i_bw;
        ++progress;

        if constexpr (champsim::debug_print) {
          fmt::print("[IFETCH] {} instr_id: {} fetch completed\n", __func__, fetched->instr_id);
        }
      }
    

      //l1i_entry.instr_depend_on_me.erase(std::begin(l1i_entry.instr_depend_on_me));
    }

    // remove this entry if we have serviced all of its instructions
    //if (l1i_entry.instr_depend_on_me.empty()) {
    //  L1I_bus.lower_level->returned.pop_front();
    //  ++progress;
    //}
      L1I_bus.lower_level->returned.pop_front();
      ++progress;

      if(!IFETCH_BUFFER.empty() && IFETCH_BUFFER.back().fetch_completed){
          prev_fetch_block = 0;
      }
  }

  auto l1d_it = std::begin(L1D_bus.lower_level->returned);
  for (auto l1d_bw = L1D_BANDWIDTH; l1d_bw > 0 && l1d_it != std::end(L1D_bus.lower_level->returned); --l1d_bw, ++l1d_it) {
    for (auto& lq_entry : LQ) {
      if (lq_entry.has_value() && lq_entry->fetch_issued && lq_entry->virtual_address >> LOG2_BLOCK_SIZE == l1d_it->v_address >> LOG2_BLOCK_SIZE) {
        lq_entry->finish(std::begin(ROB), std::end(ROB));
        lq_entry.reset();
        ++progress;
      }
    }
    ++progress;
  }
  L1D_bus.lower_level->returned.erase(std::begin(L1D_bus.lower_level->returned), l1d_it);

  return progress;
}

long O3_CPU::retire_rob()
{
  auto [retire_begin, retire_end] = champsim::get_span_p(std::cbegin(ROB), std::cend(ROB), RETIRE_WIDTH, [](const auto& x) { return x.completed; });
  if constexpr (champsim::debug_print) {
    std::for_each(retire_begin, retire_end, [](const auto& x) { fmt::print("[ROB] retire_rob instr_id: {} is retired\n", x.instr_id); });
  }

  //auto retire_count = std::distance(retire_begin, retire_end);
  auto retire_count = 0;
  auto it = retire_begin;
  while (it != retire_end){

    if(std::size(it->source_memory)){
        sim_stats.loads_retired++;
    }


      if(it->is_wrong_path){
         print_deadlock();
      }
    assert(!it->is_wrong_path && "ROB should not contain WP instrutions\n");

    if(it->branch_mispredicted || it->before_wrong_path){
	    if(!it->squashed){
	        print_deadlock();
	    }
        assert(it->squashed && "This should have been squashed\n");
    }
    //for (auto& sq_entry : SQ) {
    //  if (sq_entry.instr_id == it->instr_id) {
    //    //if(!it->is_wrong_path){
    //      sq_entry.event_cycle = current_cycle + 1;
    //    //}
    //  }
    //}
    if(!it->is_wrong_path){
        retire_instr_id = it->instr_id;
    }
    if(it->is_wrong_path){
	#ifdef BGODALA
        fmt::print("[RESTART]: ip:{:#x} wrong path: {}\n", it->ip, it->is_wrong_path); 
        #endif
        restart = true;
	prev_fetch_block = 0;
    }

    //if(it->branch_mispredicted && !it->is_wrong_path){
    //    #ifdef BGODALA
    //    fmt::print("[RESTEER]: ip:{:#x} wrong path: {}\n", it->ip, it->is_wrong_path); 
    //    #endif
    //    if(fetch_resume_cycle > current_cycle){
    //        fetch_resume_cycle = current_cycle;
    //        restart = true;
    //        prev_fetch_block = 0;
    //    }
    //}

    //if(it->is_branch){
    //  //fmt::print("[BTB]: Update ip:{:x} target:{:x} taken:{} type:{}\n", it->ip, it->branch_target, it->branch_taken, it->branch); 
    //  //impl_update_btb(it->ip, it->branch_target, it->branch_taken, it->branch);
    //  //impl_last_branch_result(it->ip, it->branch_target, it->branch_taken, it->branch);
    //}

    //if(it->is_squash_after ||
    //   it->is_serializing ||
    //   it->is_serialize_after ||
    //   it->is_serialize_before ||
    //   it->is_write_barrier ||
    //   it->is_read_barrier ||
    //   it->is_non_spec){
    //  //fmt::print("[COMMIT SQUASH AFTER] ip: {:#x}\n", it->ip);
    //  fetch_resume_cycle = current_cycle;
    //}

    if(prev_ip != it->ip){
      if(!it->is_wrong_path && !it->is_prefetch){
        retire_count++;
        prev_ip = it->ip;
      }
    }
    it++;
  }
  //std::for_each(retire_begin, retire_end, [](const auto& x) { fmt::print("[ROB] retire_rob instr_ip: {:x} is retired\n", x.ip); });

  num_retired += retire_count;
  ROB.erase(retire_begin, retire_end);

  if(retire_count == 0){
      sim_stats.rob_idle_cycles++;
  }

  return retire_count;
}

void O3_CPU::impl_initialize_branch_predictor() const { branch_module_pimpl->impl_initialize_branch_predictor(); }

void O3_CPU::impl_last_branch_result(uint64_t ip, uint64_t target, bool taken, uint8_t branch_type) const
{
  branch_module_pimpl->impl_last_branch_result(ip, target, taken, branch_type);
}

bool O3_CPU::impl_predict_branch(uint64_t ip, uint64_t predicted_target, bool always_taken, uint8_t branch_type) const
{
  return branch_module_pimpl->impl_predict_branch(ip, predicted_target, always_taken, branch_type);
}

void O3_CPU::impl_initialize_btb() const { btb_module_pimpl->impl_initialize_btb(); }

void O3_CPU::impl_update_btb(uint64_t ip, uint64_t predicted_target, bool taken, uint8_t branch_type) const
{
  btb_module_pimpl->impl_update_btb(ip, predicted_target, taken, branch_type);
}

std::pair<uint64_t, bool> O3_CPU::impl_btb_prediction(uint64_t ip, uint8_t branch_type) const { return btb_module_pimpl->impl_btb_prediction(ip, branch_type); }

// LCOV_EXCL_START Exclude the following function from LCOV
void O3_CPU::print_deadlock()
{
  fmt::print("DEADLOCK! CPU {} cycle {}\n", cpu, current_cycle);

  auto instr_pack = [](const auto& entry) {
    return std::tuple{entry.instr_id,   entry.ip, entry.fetch_issued, entry.fetch_completed,    entry.scheduled,
                      entry.executed,   entry.completed,    +entry.num_reg_dependent, entry.num_mem_ops() - entry.completed_mem_ops,
                      entry.event_cycle, entry.is_wrong_path};
  };
  std::string_view instr_fmt{
      "instr_id: {} ip: {:#x} fetch_issued: {} fetch_completed: {} scheduled: {} executed: {} completed: {} num_reg_dependent: {} num_mem_ops: {} event: {} wrong_path: {}"};
  champsim::range_print_deadlock(IFETCH_BUFFER, "cpu" + std::to_string(cpu) + "_IFETCH", instr_fmt, instr_pack);
  champsim::range_print_deadlock(DECODE_BUFFER, "cpu" + std::to_string(cpu) + "_DECODE", instr_fmt, instr_pack);
  champsim::range_print_deadlock(DISPATCH_BUFFER, "cpu" + std::to_string(cpu) + "_DISPATCH", instr_fmt, instr_pack);
  champsim::range_print_deadlock(ROB, "cpu" + std::to_string(cpu) + "_ROB", instr_fmt, instr_pack);

  // print LSQ entries
  auto lq_pack = [](const auto& entry) {
    std::string depend_id{"-"};
    if (entry->producer_id != std::numeric_limits<uint64_t>::max()) {
      depend_id = std::to_string(entry->producer_id);
    }
    return std::tuple{entry->instr_id, entry->virtual_address, entry->fetch_issued, entry->event_cycle, depend_id, entry->is_wrong_path};
  };
  std::string_view lq_fmt{"instr_id: {} address: {:#x} fetch_issued: {} event_cycle: {} waits on {} wrong_path: {}"};

  auto sq_pack = [](const auto& entry) {
    std::vector<uint64_t> depend_ids;
    std::transform(std::begin(entry.lq_depend_on_me), std::end(entry.lq_depend_on_me), std::back_inserter(depend_ids),
                   [](const std::optional<LSQ_ENTRY>& lq_entry) { return lq_entry->producer_id; });
    return std::tuple{entry.instr_id, entry.virtual_address, entry.fetch_issued, entry.event_cycle, depend_ids, entry.is_wrong_path};
  };
  std::string_view sq_fmt{"instr_id: {} address: {:#x} fetch_issued: {} event_cycle: {} LQ waiting: {} wrong_path: {}"};
  champsim::range_print_deadlock(LQ, "cpu" + std::to_string(cpu) + "_LQ", lq_fmt, lq_pack);
  champsim::range_print_deadlock(SQ, "cpu" + std::to_string(cpu) + "_SQ", sq_fmt, sq_pack);

  std::cout << std::flush;
}
// LCOV_EXCL_STOP

LSQ_ENTRY::LSQ_ENTRY(uint64_t id, uint64_t addr, uint64_t local_ip, std::array<uint8_t, 2> local_asid, bool is_wrong_path_)
    : instr_id(id), virtual_address(addr), ip(local_ip), asid(local_asid), is_wrong_path(is_wrong_path_)
{
}

void LSQ_ENTRY::finish(std::deque<ooo_model_instr>::iterator begin, std::deque<ooo_model_instr>::iterator end) const
{
  auto rob_entry = std::partition_point(begin, end, [id = this->instr_id](auto x) { return x.instr_id < id; });
  assert(rob_entry != end);
  finish(*rob_entry);
}

void LSQ_ENTRY::finish(ooo_model_instr& rob_entry) const
{
  assert(rob_entry.instr_id == this->instr_id);

  ++rob_entry.completed_mem_ops;
  assert(rob_entry.completed_mem_ops <= rob_entry.num_mem_ops());

  if constexpr (champsim::debug_print) {
    fmt::print("[LSQ] {} instr_id: {} full_address: {:#x} remain_mem_ops: {} event_cycle: {}\n", __func__, instr_id, virtual_address,
               rob_entry.num_mem_ops() - rob_entry.completed_mem_ops, event_cycle);
  }
}

bool CacheBus::issue_read(request_type data_packet)
{
  data_packet.address = data_packet.v_address;
  //data_packet.address = (data_packet.v_address >> LOG2_BLOCK_SIZE) << LOG2_BLOCK_SIZE;
  data_packet.is_translated = false;
  data_packet.cpu = cpu;
  data_packet.type = access_type::LOAD;

  return lower_level->add_rq(data_packet);
}

bool CacheBus::issue_write(request_type data_packet)
{
  data_packet.address = data_packet.v_address;
  data_packet.is_translated = false;
  data_packet.cpu = cpu;
  data_packet.type = access_type::WRITE;
  data_packet.response_requested = false;

  return lower_level->add_wq(data_packet);
}
