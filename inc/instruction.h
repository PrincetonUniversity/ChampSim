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

#ifndef INSTRUCTION_H
#define INSTRUCTION_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <string_view>
#include <vector>
#include <iostream>

#include "trace_instruction.h"
//#define PRINT_TRACE

// branch types
enum branch_type {
  BRANCH_DIRECT_JUMP = 0,
  BRANCH_INDIRECT,
  BRANCH_CONDITIONAL,
  BRANCH_DIRECT_CALL,
  BRANCH_INDIRECT_CALL,
  BRANCH_RETURN,
  BRANCH_OTHER,
  NOT_BRANCH
};

enum flags {
  NON_SPEC=0,
  SERIAL,
  SERIAL_AFTER,
  SERIAL_BEFORE,
  READ_BARRIER,
  WRITE_BARRIER,
  SQUASH_AFTER,
  SQUASHED
};

using namespace std::literals::string_view_literals;
inline constexpr std::array branch_type_names{"BRANCH_DIRECT_JUMP"sv, "BRANCH_INDIRECT"sv,      "BRANCH_CONDITIONAL"sv,
                                              "BRANCH_DIRECT_CALL"sv, "BRANCH_INDIRECT_CALL"sv, "BRANCH_RETURN"sv};

struct ooo_model_instr {
  uint64_t instr_id = 0;
  uint64_t ip = 0;
  uint64_t event_cycle = std::numeric_limits<uint64_t>::max();

  bool is_branch = false;
  bool branch_taken = false;
  bool branch_prediction = false;
  bool branch_mispredicted = false; // A branch can be mispredicted even if the direction prediction is correct when the predicted target is not correct
  bool before_wrong_path = false;
  bool squashed = false;

  bool is_non_spec = false;
  bool is_serializing = false;
  bool is_serialize_after = false;
  bool is_serialize_before = false;
  bool is_read_barrier = false;
  bool is_write_barrier = false;
  bool is_squash_after = false;
  bool is_wrong_path = false;

  bool is_prefetch = false;

  std::array<uint8_t, 2> asid = {std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()};

  branch_type branch{NOT_BRANCH};
  uint64_t branch_target = 0;

  bool dib_checked = false;
  bool fetch_issued = false;
  bool fetch_completed = false;
  bool decoded = false;
  bool scheduled = false;
  bool executed = false;
  bool completed = false;

  unsigned completed_mem_ops = 0;
  int num_reg_dependent = 0;

  std::vector<uint8_t> destination_registers = {}; // output registers
  std::vector<uint8_t> source_registers = {};      // input registers

  std::vector<uint64_t> destination_memory = {};
  std::vector<uint64_t> source_memory = {};

  // these are indices of instructions in the ROB that depend on me
  std::vector<std::reference_wrapper<ooo_model_instr>> registers_instrs_depend_on_me;

private:
  template <typename T>
  ooo_model_instr(T instr, std::array<uint8_t, 2> local_asid) : ip(instr.ip), is_branch(instr.is_branch), branch_taken(instr.branch_taken), asid(local_asid)
  {
    std::remove_copy(std::begin(instr.destination_registers), std::end(instr.destination_registers), std::back_inserter(this->destination_registers), 0);
    std::remove_copy(std::begin(instr.source_registers), std::end(instr.source_registers), std::back_inserter(this->source_registers), 0);
    std::remove_copy(std::begin(instr.destination_memory), std::end(instr.destination_memory), std::back_inserter(this->destination_memory), 0);
    std::remove_copy(std::begin(instr.source_memory), std::end(instr.source_memory), std::back_inserter(this->source_memory), 0);

    //bool writes_sp = std::count(std::begin(destination_registers), std::end(destination_registers), champsim::REG_STACK_POINTER);
    //bool writes_ip = std::count(std::begin(destination_registers), std::end(destination_registers), champsim::REG_INSTRUCTION_POINTER);
    //bool reads_sp = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_STACK_POINTER);
    //bool reads_flags = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_FLAGS);
    //bool reads_ip = std::count(std::begin(source_registers), std::end(source_registers), champsim::REG_INSTRUCTION_POINTER);
    //bool reads_other = std::count_if(std::begin(source_registers), std::end(source_registers), [](uint8_t r) {
    //  return r != champsim::REG_STACK_POINTER && r != champsim::REG_FLAGS && r != champsim::REG_INSTRUCTION_POINTER;
    //});

#ifdef PRINT_TRACE
    bool found = true;
    std::cout << "0x"<< std::hex << ip << " ";
    if(instr.is_branch){
        std::cout << "Br : ";
    }else if (std::size(source_memory) > 0){
        std::cout << "Ld : ";
    }else if (std::size(destination_memory) > 0){
        std::cout << "St : ";
    }else{
        std::cout << " : ";
    }
    for(auto it = std::begin(destination_registers); it != std::end(destination_registers); it++){
        std::cout<< std::dec << (int)*it <<" ";
        if((int)*it != 0){
          found = false;
        }
    }
    std::cout << " <- " ;
    for(auto it = std::begin(source_registers); it != std::end(source_registers); it++){
        std::cout<< std::dec << (int)*it <<" ";
        if((int)*it != 0){
          found = false;
        }
    }

    for(auto it = std::begin(source_memory); it != std::end(source_memory); it++){
        std::cout<< "0x"<< std::hex << (uint64_t)*it << std::dec << " ";
        if((int)*it != 0){
          found = false;
        }
    }

    if(found){
        std::cout<<"No source ";
    }

    std::cout << std::endl ;

#endif
    

    int flags = instr.flags;

    is_non_spec = (flags & 1 << NON_SPEC);
    is_serializing = (flags & 1 << SERIAL);
    is_serialize_after = (flags & 1 << SERIAL_AFTER);
    is_serialize_before = (flags & 1 << SERIAL_BEFORE);
    is_read_barrier = (flags & 1 << READ_BARRIER);
    is_write_barrier = (flags & 1 << WRITE_BARRIER);
    is_squash_after = (flags & 1 << SQUASH_AFTER);
    is_wrong_path = (flags & 1 << SQUASHED);

    is_prefetch = instr.pref;

    //std::cout << "0x"<< std::hex << ip << " is_branch: " << (instr.is_branch != 0)
    //    << " is_prefetch: " << is_prefetch << "is_wrong_path: "<< is_wrong_path << std::endl ;

    if(is_prefetch){
        is_wrong_path = true;
    }


    int brCode = instr.is_branch;

    if(brCode & 0x1){
        if(brCode & (1 << 1)){
		//Conditional
		if(brCode & ( 1 << 2)){
			branch = BRANCH_CONDITIONAL;
			//DIrect Ctrl
			if(brCode & (1<<3)){
				branch = BRANCH_DIRECT_CALL;
			}
		}else{
			std::cout << "This should not happen" << std::endl;
		}
	}else{
		//Unconditional
		if(brCode & ( 1 << 2)){
		        branch = BRANCH_DIRECT_JUMP;
			//DIrect Ctrl
			if(brCode & (1<<3)){
				branch = BRANCH_DIRECT_CALL;
			}
		}else{
			//Indirect Ctrl
		        branch = BRANCH_INDIRECT;

			if(brCode & (1 << 4)){
				//is REturn
				branch = BRANCH_RETURN;
			}

			// Indirest Call
			if(brCode & (1<<3)){
				branch = BRANCH_INDIRECT_CALL;
			}
		}
	}
    }
    //// determine what kind of branch this is, if any
    //if (!reads_sp && !reads_flags && writes_ip && !reads_other) {
    //  // direct jump
    //  is_branch = true;
    //  branch_taken = true;
    //  branch = BRANCH_DIRECT_JUMP;
    //} else if (!reads_sp && !reads_flags && writes_ip && reads_other) {
    //  // indirect branch
    //  is_branch = true;
    //  branch_taken = true;
    //  branch = BRANCH_INDIRECT;
    //} else if (!reads_sp && reads_ip && !writes_sp && writes_ip && reads_flags && !reads_other) {
    //  // conditional branch
    //  is_branch = true;
    //  branch_taken = instr.branch_taken; // don't change this
    //  branch = BRANCH_CONDITIONAL;
    //} else if (reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && !reads_other) {
    //  // direct call
    //  is_branch = true;
    //  branch_taken = true;
    //  branch = BRANCH_DIRECT_CALL;
    //} else if (reads_sp && reads_ip && writes_sp && writes_ip && !reads_flags && reads_other) {
    //  // indirect call
    //  is_branch = true;
    //  branch_taken = true;
    //  branch = BRANCH_INDIRECT_CALL;
    //} else if (reads_sp && !reads_ip && writes_sp && writes_ip) {
    //  // return
    //  is_branch = true;
    //  branch_taken = true;
    //  branch = BRANCH_RETURN;
    //} else if (writes_ip) {
    //  // some other branch type that doesn't fit the above categories
    //  is_branch = true;
    //  branch_taken = instr.branch_taken; // don't change this
    //  branch = BRANCH_OTHER;
    //} else {
    //  branch_taken = false;
    //}
  }

public:
  ooo_model_instr(uint8_t cpu, input_instr instr) : ooo_model_instr(instr, {cpu, cpu}) {}
  ooo_model_instr(uint8_t /*cpu*/, cloudsuite_instr instr) : ooo_model_instr(instr, {instr.asid[0], instr.asid[1]}) {}

  [[nodiscard]] std::size_t num_mem_ops() const { return std::size(destination_memory) + std::size(source_memory); }

  static bool program_order(const ooo_model_instr& lhs, const ooo_model_instr& rhs) { return lhs.instr_id < rhs.instr_id; }
};

#endif
