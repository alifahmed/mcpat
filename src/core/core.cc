/*****************************************************************************
 *                                McPAT
 *                      SOFTWARE LICENSE AGREEMENT
 *            Copyright 2012 Hewlett-Packard Development Company, L.P.
 *                          All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.”
 *
 ***************************************************************************/

#include "core.h"

#include "XML_Parse.h"
#include "basic_circuit.h"
#include "const.h"
#include "io.h"
#include "parameter.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <string>
//#include "globalvar.h"

void Core::set_params(const ParseXML *XML_interface,
                      int ithCore_,
                      InputParameter *interface_ip_,
                      bool cp) {
  /*
   * initialize, compute and optimize individual components.
   */

  XML = XML_interface;
  ithCore = ithCore_;
  interface_ip = *interface_ip_;

  bool exit_flag = true;

  set_core_param();

  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;

  exu.set_params(XML, ithCore, &interface_ip, lsu.lsq_height, coredynp, exit_flag);
  //undiffCore.set_params(XML, ithCore, &interface_ip, coredynp, exit_flag);
  corepipe.set_params(&interface_ip, coredynp);
}

void Core::set_stats(const ParseXML *XML_interface){
  exu.set_stats(XML);
  exu.computeStaticPower();
}

void Core::computeArea() {
  corepipe.computeArea();

  pipeline_area_per_unit = (corepipe.area.get_area() * coredynp.num_pipelines) / 4.0;

  exu.computeArea();
  exu.computeStaticPower();
  if (exu.exist) {
    exu.area.set_area(exu.area.get_area() + pipeline_area_per_unit);
    area.set_area(area.get_area() + exu.area.get_area());
  }

//  undiffCore.computeArea();
//  if (undiffCore.exist) {
//    area.set_area(area.get_area() + undiffCore.area.get_area());
//  }
}

void Core::computeDynamicPower(bool is_tdp) {
  /*
   * When computing TDP, power = energy_per_cycle (the value computed in this
   * function) * clock_rate (in the display_energy function) When computing
   * dyn_power; power = total energy (the value computed in this function) /
   * Total execution time (cycle count / clock rate)
   */
  // power_point_product_masks
  double pppm_t[4] = {1, 1, 1, 1};
  double rtp_pipeline_coe;
  double num_units = 4.0;
  if (is_tdp) {
    exu.computeDynamicPower(is_tdp);

    if (exu.exist) {
      set_pppm(pppm_t,
               coredynp.num_pipelines / num_units * coredynp.ALU_duty_cycle,
               coredynp.num_pipelines / num_units,
               coredynp.num_pipelines / num_units,
               coredynp.num_pipelines / num_units);
      exu.power = exu.power + corepipe.power * pppm_t;
      power = power + exu.power;
    }
    //power = power + undiffCore.power;
  } else {
    exu.computeDynamicPower(is_tdp);

    num_units = 4.0;

    if (exu.exist) {
      if (XML->sys.homogeneous_cores == 1) {
        rtp_pipeline_coe = coredynp.pipeline_duty_cycle *
                           coredynp.ALU_duty_cycle * XML->sys.total_cycles *
                           XML->sys.number_of_cores;
      } else {
        rtp_pipeline_coe = coredynp.pipeline_duty_cycle *
                           coredynp.ALU_duty_cycle * coredynp.total_cycles;
      }
      set_pppm(pppm_t,
               coredynp.num_pipelines * rtp_pipeline_coe / num_units,
               coredynp.num_pipelines / num_units,
               coredynp.num_pipelines / num_units,
               coredynp.num_pipelines / num_units);
      exu.rt_power = exu.rt_power + corepipe.power * pppm_t;
      rt_power = rt_power + exu.rt_power;
    }

    //rt_power = rt_power + undiffCore.power;
  }
}

void Core::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  if (is_tdp) {
    cout << "Core:" << endl;
    cout << indent_str << "Area = " << area.get_area() * 1e-6 << " mm^2"
         << endl;
    cout << indent_str << "Peak Dynamic = " << power.readOp.dynamic * clockRate
         << " W" << endl;
    cout << indent_str << "Subthreshold Leakage = "
         << (long_channel ? power.readOp.longer_channel_leakage
                          : power.readOp.leakage)
         << " W" << endl;
    if (power_gating) {
      cout << indent_str << "Subthreshold Leakage with power gating = "
           << (long_channel ? power.readOp.power_gated_with_long_channel_leakage
                            : power.readOp.power_gated_leakage)
           << " W" << endl;
    }
    cout << indent_str << "Gate Leakage = " << power.readOp.gate_leakage << " W"
         << endl;
    cout << indent_str
         << "Runtime Dynamic = " << rt_power.readOp.dynamic / executionTime
         << " W" << endl;
    cout << endl;

    if (exu.exist) {
      cout << indent_str << "Execution Unit:" << endl;
      cout << indent_str_next << "Area = " << exu.area.get_area() * 1e-6
           << " mm^2" << endl;
      cout << indent_str_next
           << "Peak Dynamic = " << exu.power.readOp.dynamic * clockRate << " W"
           << endl;
      cout << indent_str_next << "Subthreshold Leakage = "
           << (long_channel ? exu.power.readOp.longer_channel_leakage
                            : exu.power.readOp.leakage)
           << " W" << endl;
      if (power_gating)
        cout << indent_str_next << "Subthreshold Leakage with power gating = "
             << (long_channel
                     ? exu.power.readOp.power_gated_with_long_channel_leakage
                     : exu.power.readOp.power_gated_leakage)
             << " W" << endl;
      cout << indent_str_next << "Runtime Dynamic = "
           << exu.rt_power.readOp.dynamic / executionTime << " W" << endl;
      cout << endl;
      if (plevel > 2) {
        exu.displayEnergy(indent + 4, plevel, is_tdp);
      }
    }
  } else {
    //		cout << indent_str_next << "Instruction Fetch Unit    Peak Dynamic =
    //"
    //<< ifu.rt_power.readOp.dynamic*clockRate << " W" << endl; 		cout
    //<< indent_str_next << "Instruction Fetch Unit    Subthreshold Leakage = "
    // << ifu.rt_power.readOp.leakage <<" W" << endl; 		cout <<
    // indent_str_next << "Instruction Fetch Unit    Gate Leakage = " <<
    // ifu.rt_power.readOp.gate_leakage << " W" << endl; 		cout <<
    // indent_str_next
    //<< "Load Store Unit   Peak Dynamic = " <<
    // lsu.rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout
    // << indent_str_next << "Load Store Unit   Subthreshold Leakage = " <<
    // lsu.rt_power.readOp.leakage  << " W" << endl; 		cout <<
    // indent_str_next
    // << "Load Store Unit   Gate Leakage = " <<
    // lsu.rt_power.readOp.gate_leakage
    //<< " W" << endl; 		cout << indent_str_next << "Memory Management Unit
    // Peak Dynamic = " << mmu.rt_power.readOp.dynamic*clockRate  << " W" <<
    // endl; 		cout << indent_str_next << "Memory Management Unit Subthreshold
    // Leakage = " << mmu.rt_power.readOp.leakage  << " W" << endl; 		cout
    // << indent_str_next << "Memory Management Unit   Gate Leakage = " <<
    // mmu.rt_power.readOp.gate_leakage  << " W" << endl; 		cout <<
    // indent_str_next << "Execution Unit   Peak Dynamic = " <<
    // exu.rt_power.readOp.dynamic*clockRate  << " W" << endl; 		cout
    // << indent_str_next << "Execution Unit   Subthreshold Leakage = " <<
    // exu.rt_power.readOp.leakage  << " W" << endl; 		cout <<
    // indent_str_next
    // << "Execution Unit   Gate Leakage = " <<
    // exu.rt_power.readOp.gate_leakage
    //<< " W" << endl;
  }
}

Core ::~Core() {}

void Core::set_core_param() {
  coredynp.opt_local = XML->sys.core[ithCore].opt_local;
  coredynp.x86 = XML->sys.core[ithCore].x86;
  coredynp.Embedded = XML->sys.Embedded;
  coredynp.core_ty = (enum Core_type)XML->sys.core[ithCore].machine_type;
  coredynp.rm_ty = (enum Renaming_type)XML->sys.core[ithCore].rename_scheme;
  coredynp.fetchW = XML->sys.core[ithCore].fetch_width;
  coredynp.decodeW = XML->sys.core[ithCore].decode_width;
  coredynp.issueW = XML->sys.core[ithCore].issue_width;
  coredynp.peak_issueW = XML->sys.core[ithCore].peak_issue_width;
  coredynp.commitW = XML->sys.core[ithCore].commit_width;
  coredynp.peak_commitW = XML->sys.core[ithCore].peak_issue_width;
  coredynp.predictionW = XML->sys.core[ithCore].prediction_width;
  coredynp.fp_issueW = XML->sys.core[ithCore].fp_issue_width;
  coredynp.fp_decodeW = XML->sys.core[ithCore].fp_issue_width;
  coredynp.num_alus = XML->sys.core[ithCore].ALU_per_core;
  coredynp.num_fpus = XML->sys.core[ithCore].FPU_per_core;
  coredynp.num_muls = XML->sys.core[ithCore].MUL_per_core;
  coredynp.vdd = XML->sys.core[ithCore].vdd;
  coredynp.power_gating_vcc = XML->sys.core[ithCore].power_gating_vcc;

  coredynp.num_hthreads = XML->sys.core[ithCore].number_hardware_threads;
  coredynp.multithreaded = coredynp.num_hthreads > 1 ? true : false;
  coredynp.hthread_width =
      int(ceil(log2(XML->sys.core[ithCore].number_hardware_threads)));
  coredynp.instruction_length = XML->sys.core[ithCore].instruction_length;
  coredynp.pc_width = XML->sys.virtual_address_width;

  coredynp.opcode_length = XML->sys.core[ithCore].opcode_width;
  coredynp.micro_opcode_length = XML->sys.core[ithCore].micro_opcode_width;
  coredynp.num_pipelines = XML->sys.core[ithCore].pipelines_per_core[0];
  coredynp.pipeline_stages = XML->sys.core[ithCore].pipeline_depth[0];
  coredynp.num_fp_pipelines = XML->sys.core[ithCore].pipelines_per_core[1];
  coredynp.fp_pipeline_stages = XML->sys.core[ithCore].pipeline_depth[1];
  coredynp.int_data_width = int(ceil(XML->sys.machine_bits / 32.0)) * 32;
  coredynp.fp_data_width = coredynp.int_data_width;
  coredynp.v_address_width = XML->sys.virtual_address_width;
  coredynp.p_address_width = XML->sys.physical_address_width;

  coredynp.scheu_ty =
      (enum Scheduler_type)XML->sys.core[ithCore].instruction_window_scheme;
  coredynp.arch_ireg_width =
      int(ceil(log2(XML->sys.core[ithCore].archi_Regs_IRF_size)));
  coredynp.arch_freg_width =
      int(ceil(log2(XML->sys.core[ithCore].archi_Regs_FRF_size)));
  coredynp.num_IRF_entry = XML->sys.core[ithCore].archi_Regs_IRF_size;
  coredynp.num_FRF_entry = XML->sys.core[ithCore].archi_Regs_FRF_size;
  coredynp.pipeline_duty_cycle = XML->sys.core[ithCore].pipeline_duty_cycle;
  coredynp.total_cycles = XML->sys.core[ithCore].total_cycles;
  coredynp.busy_cycles = XML->sys.core[ithCore].busy_cycles;
  coredynp.idle_cycles = XML->sys.core[ithCore].idle_cycles;

  // Max power duty cycle for peak power estimation
  //	if (coredynp.core_ty==OOO)
  //	{
  //		coredynp.IFU_duty_cycle = 1;
  //		coredynp.LSU_duty_cycle = 1;
  //		coredynp.MemManU_I_duty_cycle =1;
  //		coredynp.MemManU_D_duty_cycle =1;
  //		coredynp.ALU_duty_cycle =1;
  //		coredynp.MUL_duty_cycle =1;
  //		coredynp.FPU_duty_cycle =1;
  //		coredynp.ALU_cdb_duty_cycle =1;
  //		coredynp.MUL_cdb_duty_cycle =1;
  //		coredynp.FPU_cdb_duty_cycle =1;
  //	}
  //	else
  //	{
  coredynp.IFU_duty_cycle = XML->sys.core[ithCore].IFU_duty_cycle;
  coredynp.BR_duty_cycle = XML->sys.core[ithCore].BR_duty_cycle;
  coredynp.LSU_duty_cycle = XML->sys.core[ithCore].LSU_duty_cycle;
  coredynp.MemManU_I_duty_cycle = XML->sys.core[ithCore].MemManU_I_duty_cycle;
  coredynp.MemManU_D_duty_cycle = XML->sys.core[ithCore].MemManU_D_duty_cycle;
  coredynp.ALU_duty_cycle = XML->sys.core[ithCore].ALU_duty_cycle;
  coredynp.MUL_duty_cycle = XML->sys.core[ithCore].MUL_duty_cycle;
  coredynp.FPU_duty_cycle = XML->sys.core[ithCore].FPU_duty_cycle;
  coredynp.ALU_cdb_duty_cycle = XML->sys.core[ithCore].ALU_cdb_duty_cycle;
  coredynp.MUL_cdb_duty_cycle = XML->sys.core[ithCore].MUL_cdb_duty_cycle;
  coredynp.FPU_cdb_duty_cycle = XML->sys.core[ithCore].FPU_cdb_duty_cycle;
  //	}

  if (!((coredynp.core_ty == OOO) || (coredynp.core_ty == Inorder))) {
    cout << "Invalid Core Type" << endl;
    exit(0);
  }
  //	if (coredynp.core_ty==OOO)
  //	{
  //		cout<<"OOO processor models are being updated and will be
  // available in next release"<<endl; 		exit(0);
  //	}
  if (!((coredynp.scheu_ty == PhysicalRegFile) ||
        (coredynp.scheu_ty == ReservationStation))) {
    cout << "Invalid OOO Scheduler Type" << endl;
    exit(0);
  }

  if (!((coredynp.rm_ty == RAMbased) || (coredynp.rm_ty == CAMbased))) {
    cout << "Invalid OOO Renaming Type" << endl;
    exit(0);
  }

  if (coredynp.core_ty == OOO) {
    if (coredynp.scheu_ty == PhysicalRegFile) {
      coredynp.phy_ireg_width =
          int(ceil(log2(XML->sys.core[ithCore].phy_Regs_IRF_size)));
      coredynp.phy_freg_width =
          int(ceil(log2(XML->sys.core[ithCore].phy_Regs_FRF_size)));
      coredynp.num_ifreelist_entries = coredynp.num_IRF_entry =
          XML->sys.core[ithCore].phy_Regs_IRF_size;
      coredynp.num_ffreelist_entries = coredynp.num_FRF_entry =
          XML->sys.core[ithCore].phy_Regs_FRF_size;
    } else if (coredynp.scheu_ty ==
               ReservationStation) { // ROB serves as Phy RF in RS based OOO
      coredynp.phy_ireg_width =
          int(ceil(log2(XML->sys.core[ithCore].ROB_size)));
      coredynp.phy_freg_width =
          int(ceil(log2(XML->sys.core[ithCore].ROB_size)));
      coredynp.num_ifreelist_entries = XML->sys.core[ithCore].ROB_size;
      coredynp.num_ffreelist_entries = XML->sys.core[ithCore].ROB_size;
    }
  }

  int GC_count =
      XML->sys.core[ithCore]
          .checkpoint_depth; // best check pointing entries for a 4~8 issue OOO
                             // should be 8~48;See TR for reference.
  if (coredynp.rm_ty == RAMbased) {
    coredynp.globalCheckpoint =
        GC_count > 4 ? 4 : GC_count; // RAM-based RAT cannot have more than 4
                                     // GCs; see "a power-aware hybrid ram-cam
                                     // renaming mechanism for fast recovery"
  } else if (coredynp.rm_ty == CAMbased) {
    coredynp.globalCheckpoint = GC_count < 1 ? 1 : GC_count;
  }

  coredynp.perThreadState = 8;
  coredynp.instruction_length = 32;
  coredynp.clockRate = XML->sys.core[ithCore].clock_rate;
  coredynp.clockRate *= 1e6;
  coredynp.regWindowing = (XML->sys.core[ithCore].register_windows_size > 0 &&
                           coredynp.core_ty == Inorder)
                              ? true
                              : false;
  coredynp.executionTime = XML->sys.total_cycles / coredynp.clockRate;
  set_pppm(coredynp.pppm_lkg_multhread,
           0,
           coredynp.num_hthreads,
           coredynp.num_hthreads,
           0);

  // does not care device types, since all core device types are set at sys.
  // level
  if (coredynp.vdd > 0) {
    interface_ip.specific_hp_vdd = true;
    interface_ip.specific_lop_vdd = true;
    interface_ip.specific_lstp_vdd = true;
    interface_ip.hp_Vdd = coredynp.vdd;
    interface_ip.lop_Vdd = coredynp.vdd;
    interface_ip.lstp_Vdd = coredynp.vdd;
  }

  if (coredynp.power_gating_vcc > -1) {
    interface_ip.specific_vcc_min = true;
    interface_ip.user_defined_vcc_min = coredynp.power_gating_vcc;
  }
}
