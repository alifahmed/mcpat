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

#include "exec_unit.h"

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

EXECU::EXECU() {
  init_params = false;
  init_stats = false;
}

void EXECU::set_params(const ParseXML *XML_interface,
                       int ithCore_,
                       InputParameter *interface_ip_,
                       double lsq_height_,
                       const CoreDynParam &dyn_p_,
                       bool exist_) {

  XML = XML_interface;
  ithCore = ithCore_;
  interface_ip = *interface_ip_;
  coredynp = dyn_p_;
  lsq_height = lsq_height_;
  exist = exist_;
  bool exist_flag = true;
  if (!exist) {
    return;
  }
  clockRate = coredynp.clockRate;
  executionTime = coredynp.executionTime;
  exeu.set_params(XML, ithCore, &interface_ip, coredynp, ALU);

  if (coredynp.num_fpus > 0) {
    fp_u.set_params(XML, ithCore, &interface_ip, coredynp, FPU);
  }
  if (coredynp.num_muls > 0) {
    mul.set_params(XML, ithCore, &interface_ip, coredynp, MUL);
  }
  /*
   * broadcast logic, including int-broadcast; int_tag-broadcast; fp-broadcast;
   * fp_tag-broadcast integer by pass has two paths and fp has 3 paths. on the
   * same bus there are multiple tri-state drivers and muxes that go to
   * different components on the same bus
   */

  init_params = true;
}

void EXECU::computeStaticPower() {
  // Doing nothing as of now, everything seems to be hapening inside set area
  // itself
}

void EXECU::set_stats(const ParseXML *XML) {
  exeu.set_stats(XML);
  if (coredynp.num_fpus > 0) {
    fp_u.set_stats(XML);
  }
  if (coredynp.num_muls > 0) {
    mul.set_stats(XML);
  }
  init_stats = true;
}

void EXECU::computeArea() {
  if (!init_params) {
    std::cerr << "[ EXECU ] Error: must set params before calling "
                 "computeArea()\n";
    exit(1);
  }
  exeu.computeArea();

  // all of the below interconnects depend ont he stats being set
  exeu.set_stats(XML);
  double fu_height = 0.0;

  area.set_area(area.get_area() + exeu.area.get_area());
  fu_height = exeu.FU_height;
  if (coredynp.num_fpus > 0) {
    fp_u.computeArea();
    area.set_area(area.get_area() + fp_u.area.get_area());
  }
  if (coredynp.num_muls > 0) {
    mul.computeArea();
    area.set_area(area.get_area() + mul.area.get_area());
    fu_height += mul.FU_height;
  }
  /*
   * broadcast logic, including int-broadcast; int_tag-broadcast; fp-broadcast;
   * fp_tag-broadcast integer by pass has two paths and fp has 3 paths. on the
   * same bus there are multiple tri-state drivers and muxes that go to
   * different components on the same bus
   */
  if (XML->sys.Embedded) {
    interface_ip.wt = Global_30;
    interface_ip.wire_is_mat_type = 0;
    interface_ip.wire_os_mat_type = 0;
    interface_ip.throughput = 1.0 / clockRate;
    interface_ip.latency = 1.0 / clockRate;
  } else {
    interface_ip.wt = Global;
    interface_ip.wire_is_mat_type =
        2; // start from semi-global since local wires are already used
    interface_ip.wire_os_mat_type = 2;
    interface_ip.throughput = 10.0 / clockRate; // Do not care
    interface_ip.latency = 10.0 / clockRate;
  }
  //area.set_area(area.get_area() + bypass.area.get_area());
}

void EXECU::computeDynamicPower(bool is_tdp) {
  if (!init_stats) {
    std::cerr << "[ EXECU ] Error: must set stats before calling "
                 "computeStaticPower()\n";
    exit(1);
  }
  if (!exist)
    return;
  double pppm_t[4] = {1, 1, 1, 1};
  //	rfu.power.reset();
  //	rfu.rt_power.reset();
  //	scheu.power.reset();
  //	scheu.rt_power.reset();
  //	exeu.power.reset();
  //	exeu.rt_power.reset();

  if (is_tdp) {
    exeu.computePower();
  } else {
    exeu.computeRuntimeDynamicPower();
  }
  if (coredynp.num_fpus > 0) {
    if (is_tdp) {
      fp_u.computePower();
    } else {
      fp_u.computeRuntimeDynamicPower();
    }
  }
  if (coredynp.num_muls > 0) {
    if (is_tdp) {
      mul.computePower();
    } else {
      mul.computeRuntimeDynamicPower();
    }
  }

  if (is_tdp) {
    set_pppm(
        pppm_t,
        2 * coredynp.ALU_cdb_duty_cycle,
        2,
        2,
        2 * coredynp
                .ALU_cdb_duty_cycle); // 2 means two source operands needs to be
                                      // passed for each int instruction.
    if (coredynp.num_muls > 0) {
      set_pppm(
          pppm_t,
          2 * coredynp.MUL_cdb_duty_cycle,
          2,
          2,
          2 * coredynp
                  .MUL_cdb_duty_cycle); // 2 means two source operands needs to
                                        // be passed for each int instruction.
      power = power + mul.power;
    }
    if (coredynp.num_fpus > 0) {
      set_pppm(
          pppm_t,
          3 * coredynp.FPU_cdb_duty_cycle,
          3,
          3,
          3 * coredynp
                  .FPU_cdb_duty_cycle); // 3 means three source operands needs
                                        // to be passed for each fp instruction.
      power = power + fp_u.power;
    }

    power = power + exeu.power;
  } else {
    set_pppm(pppm_t,
             XML->sys.core[ithCore].cdb_alu_accesses,
             2,
             2,
             XML->sys.core[ithCore].cdb_alu_accesses);

    if (coredynp.num_muls > 0) {
      set_pppm(pppm_t,
               XML->sys.core[ithCore].cdb_mul_accesses,
               2,
               2,
               XML->sys.core[ithCore]
                   .cdb_mul_accesses); // 2 means two source operands needs to
                                       // be passed for each int instruction.
      rt_power = rt_power + mul.rt_power;
    }

    if (coredynp.num_fpus > 0) {
      set_pppm(pppm_t,
               XML->sys.core[ithCore].cdb_fpu_accesses,
               3,
               3,
               XML->sys.core[ithCore].cdb_fpu_accesses);
      rt_power = rt_power + fp_u.rt_power;
    }
    rt_power = rt_power + exeu.rt_power;
  }
}

void EXECU::displayEnergy(uint32_t indent, int plevel, bool is_tdp) {
  if (!exist)
    return;
  string indent_str(indent, ' ');
  string indent_str_next(indent + 2, ' ');
  bool long_channel = XML->sys.longer_channel_device;
  bool power_gating = XML->sys.power_gating;

  //	cout << indent_str_next << "Results Broadcast Bus Area = " <<
  // bypass->area.get_area() *1e-6 << " mm^2" << endl;
  if (is_tdp) {
//    cout << indent_str << "Register Files:" << endl;
//    cout << indent_str_next << "Area = " << rfu.area.get_area() * 1e-6
//         << " mm^2" << endl;
//    cout << indent_str_next
//         << "Peak Dynamic = " << rfu.power.readOp.dynamic * clockRate << " W"
//         << endl;
//    cout << indent_str_next << "Subthreshold Leakage = "
//         << (long_channel ? rfu.power.readOp.longer_channel_leakage
//                          : rfu.power.readOp.leakage)
//         << " W" << endl;
//    if (power_gating)
//      cout << indent_str_next << "Subthreshold Leakage with power gating = "
//           << (long_channel
//                   ? rfu.power.readOp.power_gated_with_long_channel_leakage
//                   : rfu.power.readOp.power_gated_leakage)
//           << " W" << endl;
//    cout << indent_str_next
//         << "Gate Leakage = " << rfu.power.readOp.gate_leakage << " W" << endl;
//    cout << indent_str_next
//         << "Runtime Dynamic = " << rfu.rt_power.readOp.dynamic / executionTime
//         << " W" << endl;
//    cout << endl;
//    if (plevel > 3) {
//      rfu.displayEnergy(indent + 4, is_tdp);
//    }
//    cout << indent_str << "Instruction Scheduler:" << endl;
//    cout << indent_str_next << "Area = " << scheu.area.get_area() * 1e-6
//         << " mm^2" << endl;
//    cout << indent_str_next
//         << "Peak Dynamic = " << scheu.power.readOp.dynamic * clockRate << " W"
//         << endl;
//    cout << indent_str_next << "Subthreshold Leakage = "
//         << (long_channel ? scheu.power.readOp.longer_channel_leakage
//                          : scheu.power.readOp.leakage)
//         << " W" << endl;
//    if (power_gating)
//      cout << indent_str_next << "Subthreshold Leakage with power gating = "
//           << (long_channel
//                   ? scheu.power.readOp.power_gated_with_long_channel_leakage
//                   : scheu.power.readOp.power_gated_leakage)
//           << " W" << endl;
//    cout << indent_str_next
//         << "Gate Leakage = " << scheu.power.readOp.gate_leakage << " W"
//         << endl;
//    cout << indent_str_next << "Runtime Dynamic = "
//         << scheu.rt_power.readOp.dynamic / executionTime << " W" << endl;
//    cout << endl;
//    if (plevel > 3) {
//      scheu.displayEnergy(indent + 4, is_tdp);
//    }
    exeu.display(indent, is_tdp);
    if (coredynp.num_fpus > 0) {
      fp_u.display(indent, is_tdp);
    }
    if (coredynp.num_muls > 0) {
      mul.display(indent, is_tdp);
    }
//    cout << indent_str << "Results Broadcast Bus:" << endl;
//    cout << indent_str_next
//         << "Area Overhead = " << bypass.area.get_area() * 1e-6 << " mm^2"
//         << endl;
//    cout << indent_str_next
//         << "Peak Dynamic = " << bypass.power.readOp.dynamic * clockRate << " W"
//         << endl;
//    cout << indent_str_next << "Subthreshold Leakage = "
//         << (long_channel ? bypass.power.readOp.longer_channel_leakage
//                          : bypass.power.readOp.leakage)
//         << " W" << endl;
//    if (power_gating)
//      cout << indent_str_next << "Subthreshold Leakage with power gating = "
//           << (long_channel
//                   ? bypass.power.readOp.power_gated_with_long_channel_leakage
//                   : bypass.power.readOp.power_gated_leakage)
//           << " W" << endl;
//    cout << indent_str_next
//         << "Gate Leakage = " << bypass.power.readOp.gate_leakage << " W"
//         << endl;
//    cout << indent_str_next << "Runtime Dynamic = "
//         << bypass.rt_power.readOp.dynamic / executionTime << " W" << endl;
//    cout << endl;
  } else {
//    cout << indent_str_next << "Register Files    Peak Dynamic = "
//         << rfu.rt_power.readOp.dynamic * clockRate << " W" << endl;
//    cout << indent_str_next << "Register Files    Subthreshold Leakage = "
//         << rfu.rt_power.readOp.leakage << " W" << endl;
//    cout << indent_str_next << "Register Files    Gate Leakage = "
//         << rfu.rt_power.readOp.gate_leakage << " W" << endl;
//    cout << indent_str_next << "Instruction Sheduler   Peak Dynamic = "
//         << scheu.rt_power.readOp.dynamic * clockRate << " W" << endl;
//    cout << indent_str_next << "Instruction Sheduler   Subthreshold Leakage = "
//         << scheu.rt_power.readOp.leakage << " W" << endl;
//    cout << indent_str_next << "Instruction Sheduler   Gate Leakage = "
//         << scheu.rt_power.readOp.gate_leakage << " W" << endl;
    cout << indent_str_next << "Results Broadcast Bus   Peak Dynamic = "
         << bypass.rt_power.readOp.dynamic * clockRate << " W" << endl;
    cout << indent_str_next << "Results Broadcast Bus   Subthreshold Leakage = "
         << bypass.rt_power.readOp.leakage << " W" << endl;
    cout << indent_str_next << "Results Broadcast Bus   Gate Leakage = "
         << bypass.rt_power.readOp.gate_leakage << " W" << endl;
  }
}
