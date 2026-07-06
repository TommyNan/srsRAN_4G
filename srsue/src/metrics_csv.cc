/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "srsue/hdr/metrics_csv.h"

#include <chrono>
#include <float.h>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>

#include <stdio.h>

using namespace std;

namespace srsue {

metrics_csv::metrics_csv(std::string filename, bool append_)
{
  std::ios_base::openmode flags = std::ios_base::out;
  if (append_) {
    // check if file exists
    ifstream f(filename.c_str());
    if (f.good()) {
      file_exists = true;
      flags |= std::ios_base::app;
    }
  }
  file.open(filename.c_str(), flags);
}

metrics_csv::~metrics_csv()
{
  stop();
}

void metrics_csv::set_ue_handle(ue_metrics_interface* ue_)
{
  std::lock_guard<std::mutex> lock(mutex);
  ue = ue_;
}

void metrics_csv::set_flush_period(const uint32_t flush_period_sec_)
{
  std::lock_guard<std::mutex> lock(mutex);
  flush_period_sec = flush_period_sec_;
}

void metrics_csv::stop()
{
  std::unique_lock<std::mutex> lock(mutex);
  if (file.is_open()) {
    file << "#eof\n";
    file.flush();
    file.close();
  }
}

void metrics_csv::set_metrics_helper(const ue_metrics_t&  m,
                                     const phy_metrics_t& phy,
                                     const mac_metrics_t  mac[SRSRAN_MAX_CARRIERS],
                                     const bool           is_nr,
                                     const uint32_t       cc,
                                     const uint32_t       r)
{
  if (not file.is_open()) {
    return;
  }

  file << time_ms << ";";
  file << wallclock_ms << ";";

  // RAT, CC and PCI
  file << (is_nr ? "nr" : "lte") << ";";
  file << cc << ";";
  file << phy.info[r].dl_earfcn << ";";
  file << phy.info[r].pci << ";";

  // Signal quality metrics (RSRQ is not measured by the NR PHY and reads 0)
  file << float_to_string(phy.ch[r].rsrp, 2);
  file << float_to_string(phy.ch[r].rsrq, 2);
  file << float_to_string(phy.ch[r].cqi, 2);
  file << float_to_string(phy.ch[r].pathloss, 2);
  file << float_to_string(phy.sync[r].cfo, 2);

  // Find strongest neighbour for this EARFCN (cells are ordered). Only LTE neighbour cells are reported by the RRC.
  bool has_neighbour = false;
  if (not is_nr) {
    for (auto& c : m.stack.rrc.neighbour_cells) {
      if (c.earfcn == phy.info[r].dl_earfcn && c.pci != phy.info[r].pci) {
        file << c.pci << ";";
        file << float_to_string(c.rsrp, 2);
        file << float_to_string(c.cfo_hz, 2);
        has_neighbour = true;
        break;
      }
    }
  }
  if (!has_neighbour) {
    file << "n/a;";
    file << "n/a;";
    file << "n/a;";
  }

  file << float_to_string(phy.dl[r].mcs, 2);
  file << float_to_string(phy.dl[r].mimo_rank, 2);
  file << float_to_string(phy.ch[r].sinr, 2);
  file << float_to_string(phy.dl[r].fec_iters, 2);

  // L1 DL throughput (bit/s) as delivered by the MAC
  float dl_brate = 0;
  if (mac[r].rx_brate > 0 && mac[r].nof_tti > 0) {
    dl_brate = mac[r].rx_brate / (mac[r].nof_tti * 1e-3);
  }
  file << float_to_string(dl_brate, 2);

  // PDSCH BLER (%)
  int rx_pkts   = mac[r].rx_pkts;
  int rx_errors = mac[r].rx_errors;
  if (rx_pkts > 0) {
    file << float_to_string((float)100 * rx_errors / rx_pkts, 1);
  } else {
    file << float_to_string(0, 2);
  }

  // PDSCH allocation size, number of slots with PDSCH reception, carrier bandwidth and spectral efficiency
  file << float_to_string(phy.dl[r].nof_prb, 2);
  file << std::to_string(mac[r].rx_pkts) << ";";
  file << std::to_string(phy.info[r].nof_prb) << ";";
  file << float_to_string(phy.info[r].scs_hz / 1000.0f, 2);

  const float carrier_bw_hz = (float)phy.info[r].nof_prb * 12.0f * (float)phy.info[r].scs_hz;
  file << float_to_string(carrier_bw_hz > 0 ? dl_brate / carrier_bw_hz : 0, 2);

  file << float_to_string(phy.sync[r].ta_us, 2);
  file << float_to_string(phy.sync[r].distance_km, 2);
  file << float_to_string(phy.sync[r].speed_kmph, 2);
  file << float_to_string(phy.ul[r].mcs, 2);
  file << float_to_string((float)mac[r].ul_buffer, 2);

  if (mac[r].tx_brate > 0 && mac[r].nof_tti > 0) {
    file << float_to_string(mac[r].tx_brate / (mac[r].nof_tti * 1e-3), 2);
  } else {
    file << float_to_string(0, 2);
  }

  // Sum UL BLER for all CCs
  int tx_pkts   = mac[r].tx_pkts;
  int tx_errors = mac[r].tx_errors;
  if (tx_pkts > 0) {
    file << float_to_string((float)100 * tx_errors / tx_pkts, 1);
  } else {
    file << float_to_string(0, 2);
  }

  // Application (IP) level throughput measured at the GW (common for all carriers)
  file << float_to_string((float)m.gw.dl_tput_mbps, 2);
  file << float_to_string((float)m.gw.ul_tput_mbps, 2);

  file << float_to_string(m.rf.rf_o, 2);
  file << float_to_string(m.rf.rf_u, 2);
  file << float_to_string(m.rf.rf_l, 2);
  file << ((m.stack.rrc.state == RRC_STATE_CONNECTED || m.stack.rrc_nr.state == RRC_NR_STATE_CONNECTED) ? "1.0"
                                                                                                        : "0.0")
       << ";";

  // Write system metrics.
  const srsran::sys_metrics_t& sys = m.sys;
  file << float_to_string(sys.process_realmem, 2);
  file << std::to_string(sys.process_realmem_kB) << ";";
  file << std::to_string(sys.process_virtualmem_kB) << ";";
  file << float_to_string(sys.system_mem, 2);
  file << float_to_string(sys.process_cpu_usage, 2);
  file << std::to_string(sys.thread_count) << ";";

  // Write the cpu metrics.
  for (uint32_t i = 0, e = sys.cpu_count, last_cpu_index = e - 1; i != e; ++i) {
    file << float_to_string(sys.cpu_load[i], 2, (i != last_cpu_index));
  }

  file << "\n";
}

void metrics_csv::set_metrics(const ue_metrics_t& metrics, const uint32_t period_usec)
{
  std::unique_lock<std::mutex> lock(mutex);

  time_ms += period_usec / 1000;
  wallclock_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();

  if (file.is_open() && ue != NULL) {
    if (n_reports == 0 && !file_exists) {
      file << "time;timestamp_ms;rat;cc;earfcn;pci;rsrp;rsrq;cqi;pl;cfo;pci_neigh;rsrp_neigh;cfo_neigh;"
              "dl_mcs;dl_mimo_rank;dl_snr;dl_turbo;dl_brate;dl_bler;dl_nof_prb;dl_nof_slots;cell_nof_prb;scs_khz;"
              "dl_se_bps_hz;"
              "ul_ta;distance_km;speed_kmph;ul_mcs;ul_buff;ul_brate;ul_bler;"
              "gw_dl_tput_mbps;gw_ul_tput_mbps;"
              "rf_o;rf_u;rf_l;is_attached;"
              "proc_rmem;proc_rmem_kB;proc_vmem_kB;sys_mem;sys_load;thread_count";

      // Add the cores.
      for (uint32_t i = 0, e = metrics.sys.cpu_count; i != e; ++i) {
        file << ";cpu_" << std::to_string(i);
      }

      // Add the new line.
      file << "\n";
    }

    // Metrics for LTE carrier
    for (uint32_t r = 0; r < metrics.phy.nof_active_cc; r++) {
      set_metrics_helper(metrics, metrics.phy, metrics.stack.mac, false, r, r);
    }

    // Metrics for NR carrier
    for (uint32_t r = 0; r < metrics.phy_nr.nof_active_cc; r++) {
      set_metrics_helper(metrics,
                         metrics.phy_nr,
                         metrics.stack.mac_nr,
                         true,
                         metrics.phy.nof_active_cc + r, // NR carrier offset
                         r);
    }

    n_reports++;

    if (flush_period_sec > 0) {
      flush_time_ms += period_usec / 1000;
      if (flush_time_ms / 1000 >= flush_period_sec) {
        file.flush();
        flush_time_ms -= flush_period_sec * 1000;
      }
    }
  } else {
    std::cout << "couldn't write CSV file." << std::endl;
  }
}

std::string metrics_csv::float_to_string(float f, int digits, bool add_semicolon)
{
  std::ostringstream os;
  const int          precision = (f == 0.0) ? digits - 1 : digits - log10f(fabs(f)) - 2 * DBL_EPSILON;
  os << std::fixed << std::setprecision(precision) << f;
  if (add_semicolon)
    os << ';';
  return os.str();
}

} // namespace srsue
