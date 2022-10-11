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

  // Verbosity setting
  ov_core::Printer::setPrintLevel("ALL");

  // Ensure we have a path
  if (argc < 2) {
    PRINT_ERROR(RED "ERROR: Please specify a timing and memory percent folder\n" RESET);
    PRINT_ERROR(RED "ERROR: ./timing_percentages <timings_folder>\n" RESET);
    PRINT_ERROR(RED "ERROR: rosrun ov_eval timing_percentages <timings_folder>\n" RESET);
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
  path_statistics = path_statistics.parent_path().append("computational_load.txt");
  timing_ofstream.open(path_statistics.c_str(), std::ofstream::out | std::ofstream::trunc);

  //===============================================================================
  //===============================================================================
  //===============================================================================

  // Summary information (%cpu, %mem, threads)
  std::map<std::string, std::vector<ov_eval::Statistics>> algo_timings;
  for (const auto &p : path_algorithms) {
    std::vector<ov_eval::Statistics> temp = {ov_eval::Statistics(), ov_eval::Statistics(), ov_eval::Statistics()};
    algo_timings.insert({p.stem().string(), temp});
  }

  // Loop through each algorithm type
  for (size_t i = 0; i < path_algorithms.size(); i++) {

    // Debug print
    PRINT_DEBUG("======================================\n");
    PRINT_DEBUG("[COMP]: processing %s algorithm\n", path_algorithms.at(i).stem().c_str());

    // our total summed values
    std::vector<double> total_times;
    std::vector<Eigen::Vector3d> total_summed_values;

    // Loop through each sub-directory in this folder
    for (auto &entry : boost::filesystem::recursive_directory_iterator(path_algorithms.at(i))) {

      // skip if not a directory
      if (boost::filesystem::is_directory(entry))
        continue;

      // skip if not a text file
      if (entry.path().extension() != ".txt")
        continue;

      // Load the data from file
      std::vector<double> times;
      std::vector<Eigen::Vector3d> summed_values;
      std::vector<Eigen::VectorXd> node_values;
      ov_eval::Loader::load_timing_percent(entry.path().string(), times, summed_values, node_values);

      // Append to our summed values
      total_times.insert(total_times.end(), times.begin(), times.end());
      total_summed_values.insert(total_summed_values.end(), summed_values.begin(), summed_values.end());
    }

    // append to the map
    std::string algo = path_algorithms.at(i).stem().string();
    for (size_t j = 0; j < total_times.size(); j++) {
      algo_timings.at(algo).at(0).timestamps.push_back(total_times.at(j));
      algo_timings.at(algo).at(0).values.push_back(total_summed_values.at(j)(0));
      algo_timings.at(algo).at(1).timestamps.push_back(total_times.at(j));
      algo_timings.at(algo).at(1).values.push_back(total_summed_values.at(j)(1));
      algo_timings.at(algo).at(2).timestamps.push_back(total_times.at(j));
      algo_timings.at(algo).at(2).values.push_back(total_summed_values.at(j)(2));
    }

    // Display for the user
    PRINT_DEBUG("\tloaded %d timestamps from file!!\n", (int)algo_timings.at(algo).at(0).timestamps.size());
    algo_timings.at(algo).at(0).calculate();
    algo_timings.at(algo).at(1).calculate();
    algo_timings.at(algo).at(2).calculate();
    PRINT_DEBUG("\tPREC: mean_cpu = %.3f +- %.3f\n", algo_timings.at(algo).at(0).mean, algo_timings.at(algo).at(0).std);
    PRINT_DEBUG("\tPREC: mean_mem = %.3f +- %.3f\n", algo_timings.at(algo).at(1).mean, algo_timings.at(algo).at(1).std);
    PRINT_DEBUG("\tTHR: mean_threads = %.3f +- %.3f\n", algo_timings.at(algo).at(2).mean, algo_timings.at(algo).at(2).std);
    PRINT_DEBUG("======================================\n");
  }

  //===============================================================================
  //===============================================================================
  //===============================================================================

#ifdef HAVE_PYTHONLIBS

  // Plot line colors
  std::vector<std::string> colors = {"#005AA9", "#E6001A", "#99C000", "#EC6500", "cyan", "magenta"};
  std::vector<std::string> linestyle = {"-", "--", "-."};
  assert(algo_timings.size() <= colors.size() * linestyle.size());

  // Parameters
  std::map<std::string, std::string> params_rpe;
//  params_rpe.insert({"notch", "False"});
  params_rpe.insert({"sym", ""});
//  params_rpe.insert({"fontsize", "11"});
//  params_rpe.insert({"font.family", "serif"});

  std::map<std::string, std::string> text_data;
  text_data.insert({"fontfamily", "serif"});
  text_data.insert({"fontsize", "11"});

  std::map<std::string, std::string> font_manager_prop;
  font_manager_prop.insert({"family", "serif"});
  font_manager_prop.insert({"size", "11"});

  //============================================================
  //============================================================
  // Plot this figure
  float cm = 1/2.54; // 1cm in inches
  float px = 100; // 1inch in px
  matplotlibcpp::figure_size((size_t)(18*cm*px), (size_t)(10*cm*px));

  // Plot each RPE next to each other
  double width = 0.1 / (algo_timings.size() + 1);
  std::vector<double> yticks;
  std::vector<std::string> labels;
  int ct_algo = 0;
  double ct_pos = 0;
  PRINT_ALL("CPU Percent Usage\n");
  for (auto &algo : algo_timings) {
    // Start based on what algorithm we are doing
    ct_pos = 1 + 1.5 * ct_algo * width;
    yticks.push_back(ct_pos);
//    labels.push_back(algo.first);
    labels.push_back("");
    // Plot it!!!
    matplotlibcpp::boxplot(algo.second.at(0).values, ct_pos, width, colors.at(ct_algo % colors.size()),
                           linestyle.at(ct_algo / colors.size()), params_rpe, true);

    // Export to file
    std::stringstream ss_cpu;
    ss_cpu << algo.first << " | cpu | " << std::fixed << std::setprecision(4);
    for (auto &v : algo.second.at(0).values) {
      ss_cpu << v << ",";
    }
    timing_ofstream << ss_cpu.rdbuf() << std::endl;
    timing_ofstream.flush();
    // Move forward
    ct_algo++;
  }

  // Add "fake" plots for our legend
  ct_algo = 0;
  for (const auto &algo : algo_timings) {
    std::map<std::string, std::string> params_empty;
    params_empty.insert({"label", algo.first});
    params_empty.insert({"linestyle", linestyle.at(ct_algo / colors.size())});
    params_empty.insert({"color", colors.at(ct_algo % colors.size())});
    std::vector<double> vec_empty;
    matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
    ct_algo++;
  }

  // Display to the user
//  matplotlibcpp::ylim(1.0 - 1 * width, ct_pos + 1 * width);
  matplotlibcpp::xticks(yticks, labels, text_data);
  matplotlibcpp::yticks(std::vector<int>({0,100,200,300,400,500,600}), text_data);
  matplotlibcpp::ylim(0, 600);
  matplotlibcpp::ylabel("CPU Percent Usage", text_data);
  matplotlibcpp::tight_layout();
  matplotlibcpp::legend();
  matplotlibcpp::show(false);

  //============================================================
  //============================================================
  // Plot this figure
  matplotlibcpp::figure_size(1500, 400);

  // Plot each RPE next to each other
  width = 0.1 / (algo_timings.size() + 1);
  yticks.clear();
  labels.clear();
  ct_algo = 0;
  ct_pos = 0;
  PRINT_ALL("Memory Percent Usage\n");
  for (auto &algo : algo_timings) {
    // Start based on what algorithm we are doing
    ct_pos = 1 + 1.5 * ct_algo * width;
    yticks.push_back(ct_pos);
    labels.push_back(algo.first);
    // Plot it!!!
    matplotlibcpp::boxplot(algo.second.at(1).values, ct_pos, width, colors.at(ct_algo % colors.size()),
                           linestyle.at(ct_algo / colors.size()), params_rpe, false);

    // Export to file
    std::stringstream ss_mem;
    ss_mem << algo.first << " | mem | " << std::fixed << std::setprecision(4);
    for (auto &v : algo.second.at(1).values) {
      ss_mem << v << ",";
    }
    timing_ofstream << ss_mem.rdbuf() << std::endl;
    timing_ofstream.flush();

    // Move forward
    ct_algo++;
  }

  // Add "fake" plots for our legend
  ct_algo = 0;
  for (const auto &algo : algo_timings) {
    std::map<std::string, std::string> params_empty;
    params_empty.insert({"label", algo.first});
    params_empty.insert({"linestyle", linestyle.at(ct_algo / colors.size())});
    params_empty.insert({"color", colors.at(ct_algo % colors.size())});
    std::vector<double> vec_empty;
    matplotlibcpp::plot(vec_empty, vec_empty, params_empty);
    ct_algo++;
  }

  // Display to the user
  matplotlibcpp::ylim(1.0 - 1 * width, ct_pos + 1 * width);
  matplotlibcpp::yticks(yticks, labels);
  matplotlibcpp::xlabel("Memory Percent Usage");
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(true);

#endif

  // Done!
  return EXIT_SUCCESS;
}
