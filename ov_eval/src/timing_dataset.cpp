/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Eigen>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

#include "utils/Loader.h"
#include "utils/Statistics.h"
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {

  sleep(10);

  // Verbosity setting
  ov_core::Printer::setPrintLevel("ALL");

  // Ensure we have a path
  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify a timing folder\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_dataset <timings_folder>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval timing_dataset <timings_folder>\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Get the algorithms we will process
  // Also create empty statistic objects for each of our datasets
  std::string path_algos(argv[1]);
  std::vector<boost::filesystem::path> path_algorithms;
  for (const auto &entry : boost::filesystem::directory_iterator(path_algos)) {
    if (boost::filesystem::is_directory(entry)) {
      path_algorithms.push_back(entry.path());
    }
  }
  std::sort(path_algorithms.begin(), path_algorithms.end());

  // Prepare export to file
  std::ofstream timing_ofstream;
  boost::filesystem::path path_statistics(argv[1]);
  path_statistics = path_statistics.parent_path().append("timing_dataset.txt");
  timing_ofstream.open(path_statistics.c_str(), std::ofstream::out | std::ofstream::trunc);

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Summary information (tracking,propagation,msckf update,re-tri & marg,total)
  std::map<std::string, std::tuple<ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics>> algo_timings;

  // Loop through each algorithm type
  for (size_t i = 0; i < path_algorithms.size(); i++) {
    std::string algo = path_algorithms.at(i).stem().string();
    std::vector<double> times_total;
    std::vector<Eigen::VectorXd> timing_values_total;

//    std::vector<ov_eval::Statistics> temp = {ov_eval::Statistics(), ov_eval::Statistics(), ov_eval::Statistics(), ov_eval::Statistics(), ov_eval::Statistics()};
    std::map<double, std::tuple<ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics>> local_statistics;

    // Debug print
    PRINT_DEBUG("======================================\n");
    PRINT_DEBUG("[COMP]: processing %s algorithm\n", path_algorithms.at(i).stem().c_str());

    size_t count_runs = 0;

    // Loop through each sub-directory in this folder
    for (auto &entry : boost::filesystem::recursive_directory_iterator(path_algorithms.at(i))) {

      // skip if not a directory
      if (boost::filesystem::is_directory(entry))
        continue;

      // skip if not a text file
      if (entry.path().extension() != ".txt")
        continue;

      // Load the data from file
      std::vector<std::string> names_temp;
      std::vector<double> times;
      std::vector<Eigen::VectorXd> timing_values;
      ov_eval::Loader::load_timing_flamegraph(entry.path().string(), names_temp, times, timing_values);

      for (size_t t = 0; t < times.size(); ++t) {
        std::get<0>(local_statistics[times[t]]).timestamps.push_back(times[t]);
        std::get<0>(local_statistics[times[t]]).values.push_back(timing_values.at(t)(0));
        std::get<1>(local_statistics[times[t]]).values.push_back(timing_values.at(t)(1));
        std::get<2>(local_statistics[times[t]]).values.push_back(timing_values.at(t)(2));
        std::get<3>(local_statistics[times[t]]).values.push_back(timing_values.at(t)(3));
        std::get<4>(local_statistics[times[t]]).values.push_back(timing_values.at(t)(4));
      }

      count_runs++;
    }

    // append to the map
    PRINT_DEBUG("[COMP]: Grouping timing information for %s\n", algo.c_str());
//    algo_timings.insert(std::make_pair(algo, std::tuple<ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics, ov_eval::Statistics>()));
    std::stringstream ss_t0, ss_v0, ss_v1, ss_v2, ss_v3, ss_v4;
    ss_t0 << algo<< " | t | " << std::fixed << std::setprecision(8);
    ss_v0 << algo<< " | tracking | " << std::fixed << std::setprecision(4);
    ss_v1 << algo<< " | prop | " << std::fixed << std::setprecision(4);
    ss_v2 << algo<< " | update | " << std::fixed << std::setprecision(4);
    ss_v3 << algo<< " | marg | " << std::fixed << std::setprecision(4);
    ss_v4 << algo<< " | total | " << std::fixed << std::setprecision(4);

    for (auto &elem : local_statistics) {
//      if (std::get<0>(elem.second).timestamps.size() == count_runs) {
      std::get<0>(elem.second).calculate();
      std::get<1>(elem.second).calculate();
      std::get<2>(elem.second).calculate();
      std::get<3>(elem.second).calculate();
      std::get<4>(elem.second).calculate();

      ss_t0 << elem.first << ",";
      ss_v0 << std::get<0>(elem.second).mean << ",";
      ss_v1 << std::get<1>(elem.second).mean << ",";
      ss_v2 << std::get<2>(elem.second).mean << ",";
      ss_v3 << std::get<3>(elem.second).mean << ",";
      ss_v4 << std::get<4>(elem.second).mean << ",";
//      }
    }
    timing_ofstream << ss_t0.rdbuf() << std::endl << ss_v0.rdbuf() << std::endl << ss_v1.rdbuf() << std::endl << ss_v2.rdbuf() << std::endl << ss_v3.rdbuf() << std::endl << ss_v4.rdbuf() << std::endl;
    timing_ofstream.flush();
  }

//  PRINT_DEBUG("======================================\n");
//  PRINT_DEBUG("[EXP]: Writing data to file\n");
//
//  for (size_t i = 0; i < path_algorithms.size(); i++) {
//    std::string algo = path_algorithms.at(i).stem().string();
//    PRINT_DEBUG("[EXP]: Exporting algo %s\n", algo.c_str());
//
//    PRINT_DEBUG("[EXP] Writing time and tracking\n");
//    std::stringstream ss_t0, ss_v0;
//    ss_t0 << algo<< " | t | " << std::fixed << std::setprecision(8);
//    ss_v0 << algo<< " | v_tracking | " << std::fixed << std::setprecision(4);
//    for (size_t t = 0; t < std::get<0>(algo_timings[algo]).timestamps.size(); ++i) {
//      ss_t0 << std::get<0>(algo_timings[algo]).timestamps[t] << ",";
//      ss_v0 << std::get<0>(algo_timings[algo]).values[t] << ",";
//    }
//    timing_ofstream << ss_t0.rdbuf() << std::endl << ss_v0.rdbuf() << std::endl;
//    timing_ofstream.flush();
//
//    PRINT_DEBUG("[EXP] Writing propagation\n");
//    std::stringstream ss_v1;
//    ss_v1 << algo<< " | v_prop | " << std::fixed << std::setprecision(4);
//    for (size_t t = 0; t < std::get<1>(algo_timings[algo]).values.size(); ++i) {
//      ss_v1 << std::get<1>(algo_timings[algo]).values[t] << ",";
//    }
//    timing_ofstream << ss_v1.rdbuf() << std::endl;
//    timing_ofstream.flush();
//
//    PRINT_DEBUG("[EXP] Writing update\n");
//    std::stringstream ss_v2;
//    ss_v2 << algo<< " | v_update | " << std::fixed << std::setprecision(4);
//    for (size_t t = 0; t < std::get<2>(algo_timings[algo]).values.size(); ++i) {
//      ss_v2 << std::get<2>(algo_timings[algo]).values[t] << ",";
//    }
//    timing_ofstream << ss_v2.rdbuf() << std::endl;
//    timing_ofstream.flush();
//
//    PRINT_DEBUG("[EXP] Writing marginalization\n");
//    std::stringstream ss_v3;
//    ss_v3 << algo<< " | v_marg | " << std::fixed << std::setprecision(4);
//    for (size_t t = 0; t < std::get<3>(algo_timings[algo]).values.size(); ++i) {
//      ss_v3 << std::get<3>(algo_timings[algo]).values[t] << ",";
//    }
//    timing_ofstream << ss_v3.rdbuf() << std::endl;
//    timing_ofstream.flush();
//
//    PRINT_DEBUG("[EXP] Writing total\n");
//    std::stringstream ss_v4;
//    ss_v4 << algo<< " | v_total | " << std::fixed << std::setprecision(4);
//    for (size_t t = 0; t < std::get<4>(algo_timings[algo]).values.size(); ++i) {
//      ss_v4 << std::get<4>(algo_timings[algo]).values[t] << ",";
//    }
//    timing_ofstream << ss_v4.rdbuf() << std::endl;
//    timing_ofstream.flush();
//  }

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Done!
  return EXIT_SUCCESS;
}
