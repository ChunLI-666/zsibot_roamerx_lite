// SPDX-License-Identifier: Apache-2.0
// Optional failure-only experiment evidence; no planner or sensor policy lives here.
#pragma once
#include <chrono>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

namespace official_lattice_rpp_adapter::diagnostics {
inline std::string quote(const std::string & value) {
  std::ostringstream out;out << '"';
  for (const unsigned char c:value) {
    switch(c) {
      case '"':out << "\\\"";break;
      case '\\':out << "\\\\";break;
      case '\n':out << "\\n";break;
      case '\r':out << "\\r";break;
      case '\t':out << "\\t";break;
      default:
        if (c<0x20) {out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;}
        else {out << c;}
    }
  }
  out << '"';return out.str();
}
inline std::int64_t steadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::int64_t threadCpuNs() {
  timespec t{};
  return clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t)==0?
    std::int64_t(t.tv_sec)*1000000000LL+t.tv_nsec:-1;
}
inline std::string readFile(const std::string & path) {
  std::ifstream in(path,std::ios::binary);in.exceptions(std::ios::badbit);
  if (!in) {throw std::runtime_error("Cannot open diagnostic identity input: "+path);}
  return std::string(std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>());
}
template<class PoseStamped> std::string poseJson(const PoseStamped & p) {
  std::ostringstream out;out << std::setprecision(17)
    << "{\"header\":{\"frame_id\":" << quote(p.header.frame_id)
    << ",\"stamp\":{\"sec\":" << p.header.stamp.sec << ",\"nanosec\":" << p.header.stamp.nanosec
    << "}},\"position\":[" << p.pose.position.x << ',' << p.pose.position.y << ',' << p.pose.position.z
    << "],\"quaternion_xyzw\":[" << p.pose.orientation.x << ',' << p.pose.orientation.y
    << ',' << p.pose.orientation.z << ',' << p.pose.orientation.w << "]}";
  return out.str();
}
template<class Node> std::string parametersJson(Node & node) {
  std::ostringstream out;out << '[';bool comma=false;
  for (const auto & name:node.list_parameters({},0).names) {
    const auto parameter=node.get_parameter(name);
    if (comma) {out << ',';}comma=true;
    out << "{\"name\":" << quote(name) << ",\"type\":" << quote(parameter.get_type_name())
      << ",\"value_text\":" << quote(parameter.value_to_string()) << '}';
  }
  out << ']';return out.str();
}
struct FailureSnapshot {
  std::vector<unsigned char> grid;
  unsigned int width=0,height=0;
  double resolution=0.,origin_x=0.,origin_y=0.;
  std::string start_json,goal_json,footprint_json,parameters_json,costmap_parameters_json;
  std::string source_identity,primitive_path,primitive_bytes,planner_type;
  std::string phase="filled_start_goal_checks",base_outcome="not_called";
  std::int64_t capture_start_ns=0,capture_end_ns=0,base_start_ns=0,base_end_ns=0;
  std::int64_t base_cpu_start_ns=-1,base_cpu_end_ns=-1;
  std::uint64_t base_cancel_checks=0,base_cancel_true=0,raw_pose_count=0;
  void beginBase() {
    phase="official_planner";base_outcome="running";
    base_start_ns=steadyNs();base_cpu_start_ns=threadCpuNs();
  }
  void endBase(bool returned) {
    base_cpu_end_ns=threadCpuNs();base_end_ns=steadyNs();
    base_outcome=returned?"returned":"threw";
  }
  void save(const std::filesystem::path & directory,std::exception_ptr error) const {
    // Caller has already unwound all costmap/index guards and released request mutex.
    std::string failure_type="unknown_non_std_exception",failure_message;
    try {std::rethrow_exception(error);}
    catch (const std::exception & e) {failure_type=typeid(e).name();failure_message=e.what();}
    catch (...) {}
    const auto write_started=steadyNs();
    // A fresh numbered directory makes any collision explicit; nothing is truncated.
    if (!std::filesystem::create_directory(directory)) {throw std::runtime_error("Diagnostic output exists: "+directory.string());}
    auto write=[&](const std::string & name,const std::string & bytes) {
      std::ofstream file(directory/name,std::ios::binary);file.exceptions(std::ios::failbit|std::ios::badbit);
      file.write(bytes.data(),static_cast<std::streamsize>(bytes.size()));file.close();
    };
    write("costmap.raw",std::string(reinterpret_cast<const char *>(grid.data()),grid.size()));
    write("primitive.input",primitive_bytes);
    write("source_identity.input",source_identity);
    const auto input_write_finished=steadyNs();
    std::ostringstream meta;meta << std::setprecision(17)
      << "{\"schema\":\"checked_lattice_failure_snapshot_v1\",\"phase\":" << quote(phase)
      << ",\"failure_dynamic_type\":" << quote(failure_type) << ",\"failure_message\":" << quote(failure_message)
      << ",\"planner_type\":" << quote(planner_type) << ",\"width\":" << width << ",\"height\":" << height
      << ",\"resolution\":" << resolution << ",\"origin_x\":" << origin_x << ",\"origin_y\":" << origin_y
      << ",\"grid_encoding\":\"row-major uint8 native Costmap2D bytes, x fastest\",\"grid_bytes\":" << grid.size()
      << ",\"start\":" << start_json << ",\"goal\":" << goal_json << ",\"footprint\":" << footprint_json
      << ",\"parameters\":" << parameters_json << ",\"costmap_parameters\":" << costmap_parameters_json
      << ",\"primitive_path\":" << quote(primitive_path)
      << ",\"capture_start_steady_ns\":" << capture_start_ns << ",\"capture_end_steady_ns\":" << capture_end_ns
      << ",\"capture_wall_ns\":" << capture_end_ns-capture_start_ns
      << ",\"base_outcome\":" << quote(base_outcome) << ",\"base_start_steady_ns\":" << base_start_ns
      << ",\"base_end_steady_ns\":" << base_end_ns << ",\"base_wall_ns\":" << base_end_ns-base_start_ns
      << ",\"base_thread_cpu_ns\":";
    if (base_cpu_start_ns<0||base_cpu_end_ns<0) {meta << "null";}
    else {meta << base_cpu_end_ns-base_cpu_start_ns;}
    meta << ",\"base_cancel_checks\":" << base_cancel_checks << ",\"base_cancel_true\":" << base_cancel_true
      << ",\"base_raw_pose_count\":" << raw_pose_count
      << ",\"actual_iterations\":null,\"iterations_boundary\":\"Installed GlobalPlanner interface does not expose actual iterations; configured max_iterations/max_planning_time are in parameters\""
      << ",\"input_write_wall_ns\":" << input_write_finished-write_started
      << ",\"write_boundary\":\"Failure-only synchronous writes after both request and costmap mutex release; disk latency delays exception delivery, outside original search timer; see write_timing.json for complete save duration\"}\n";
    write("snapshot.json",meta.str());
    write("write_timing.json","{\"save_wall_ns_before_timing_file\":"+std::to_string(steadyNs()-write_started)+"}\n");
    // Completion marker is last; incomplete or failed writes remain explicit evidence gaps.
    write("COMPLETE","checked_lattice_failure_snapshot_v1\n");
  }
};
}  // namespace official_lattice_rpp_adapter::diagnostics
