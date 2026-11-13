// Copyright (c) 2026 Paul N. Beuchat. MIT License.
// Offline numeric replay only. No SDK, ROS, image pixels or camera access.
#include "mapper.hpp"
#include "rgbd_camera/imu_combiner.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace clock_experiment;
using namespace rgbd_camera;
namespace fs = std::filesystem;
constexpr int64_t max_age = 500000000;
struct ClockRow {
  int64_t observed, aseq, ahost, adev, gseq, ghost, gdev;
  uint64_t object, batch, poll, snapshot, packet, packets, queued;
};
struct ImageRow { int64_t observed, kind, sequence, host, device; };
static_assert(sizeof(ClockRow) == 112 && sizeof(ImageRow) == 40);
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);
void check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
template<class T> std::vector<T> read(const fs::path& path) {
  const auto bytes = fs::file_size(path);
  check(bytes % sizeof(T) == 0, "partial binary record");
  std::vector<T> rows(bytes / sizeof(T));
  std::ifstream file(path, std::ios::binary);
  check(bool(file.read(reinterpret_cast<char*>(rows.data()), bytes)), "binary read failed");
  return rows;
}
struct Event { int64_t receipt; int kind; size_t row; }; // 1 dequeue, 2 image.
struct Stats {
  uint64_t seen = 0, accepted = 0, unavailable = 0, invalid = 0, stale_or_duplicate = 0;
  uint64_t backwards = 0, future = 0;
  int64_t previous = 0, min_delta = INT64_MAX, max_delta = INT64_MIN;
  int64_t min_age = INT64_MAX, max_age = INT64_MIN, max_accepted_gap = 0, last_accepted = 0;
  int64_t min_sdk_difference = INT64_MAX, max_sdk_difference = INT64_MIN;
  void sample(int64_t stamp, int64_t receipt, int64_t sdk, int64_t tolerance) {
    if (previous) {
      min_delta = std::min(min_delta, stamp - previous);
      max_delta = std::max(max_delta, stamp - previous);
      backwards += stamp < previous;
    }
    previous = stamp;
    min_age = std::min(min_age, receipt - stamp);
    max_age = std::max(max_age, receipt - stamp);
    future += stamp - receipt > tolerance;
    min_sdk_difference = std::min(min_sdk_difference, stamp - sdk);
    max_sdk_difference = std::max(max_sdk_difference, stamp - sdk);
  }
  void accept(int64_t receipt) {
    if (last_accepted) max_accepted_gap = std::max(max_accepted_gap, receipt - last_accepted);
    last_accepted = receipt;
    ++accepted;
  }
  void print() const {
    std::cout << "{\"seen\":" << seen << ",\"accepted\":" << accepted
      << ",\"unavailable\":" << unavailable << ",\"invalid\":" << invalid
      << ",\"stale_or_duplicate\":" << stale_or_duplicate << ",\"backwards\":" << backwards
      << ",\"future\":" << future << ",\"min_delta_ns\":" << min_delta
      << ",\"max_delta_ns\":" << max_delta << ",\"min_age_ns\":" << min_age
      << ",\"max_age_ns\":" << max_age << ",\"max_accepted_receipt_gap_ns\":" << max_accepted_gap
      << ",\"min_difference_from_sdk_ns\":" << min_sdk_difference
      << ",\"max_difference_from_sdk_ns\":" << max_sdk_difference << '}';
  }
};

void replay(const fs::path& trace, ClockSource clock, FaultPolicy policy) {
  const auto arrivals = read<ClockRow>(trace / "arrival.bin");
  const auto dequeues = read<ClockRow>(trace / "dequeue.bin");
  const auto images = read<ImageRow>(trace / "images.bin");
  check(!arrivals.empty() && !dequeues.empty(), "empty trace");
  std::vector<Event> events;
  for (size_t i = 0; i < dequeues.size(); ++i) events.push_back({dequeues[i].observed, 1, i});
  for (size_t i = 0; i < images.size(); ++i) events.push_back({images[i].observed, 2, i});
  // Causal merge of independently recorded stages; order within a stage is retained.
  std::stable_sort(events.begin(), events.end(), [](const auto& a, const auto& b) {
    return a.receipt < b.receipt || (a.receipt == b.receipt && a.kind < b.kind);
  });
  Mapper mapper;
  RawClockGuard raw_guard;
  MotionGate gate(5000000);
  ImuCombiner combiner(10000000);
  std::array<Stats, 6> stats;
  uint64_t combined = 0, faults = 0, raw_faults = 0;
  int64_t first_fault_at = 0, combined_previous = 0, combined_max_gap = 0, observed_end = 0;
  std::array<int64_t, 4> image_receipt{}, image_stamp{}, image_sdk{};
  std::array<uint64_t, 2> paired{};
  std::array<int64_t, 2> max_pair_skew{}, max_pair_skew_change{};
  bool stopped = false;
  for (const auto& event : events) {
    observed_end = event.receipt;
    if (event.kind == 1) {
      const auto& row = dequeues[event.row];
      try {
        raw_guard.observe(0, row.adev, uint32_t(row.aseq));
        raw_guard.observe(1, row.gdev, uint32_t(row.gseq));
      } catch (const std::exception&) { ++raw_faults; stopped = true; break; }
      if (clock == ClockSource::Local) mapper.observe(std::max(row.adev, row.gdev), row.observed);
      for (size_t sensor = 0; sensor < 2; ++sensor) {
        auto& s = stats[sensor];
        ++s.seen;
        const auto sdk = sensor ? row.ghost : row.ahost;
        const auto device = sensor ? row.gdev : row.adev;
        const auto stamp = clock == ClockSource::Sdk ? std::optional<int64_t>(sdk) : mapper.map(device);
        if (!stamp) { ++s.unavailable; continue; }
        s.sample(*stamp, row.observed, sdk, 5000000);
        Motion motion{sensor ? MotionKind::Gyroscope : MotionKind::Accelerometer,
                      uint32_t(sensor ? row.gseq : row.aseq), *stamp, row.observed, {0, 0, 0}};
        bool admitted = false;
        try { admitted = gate.admit(motion, row.observed, max_age); }
        catch (const std::exception&) {
          ++s.invalid; ++faults;
          if (!first_fault_at) first_fault_at = row.observed - arrivals.front().observed;
          if (policy == FaultPolicy::Stop) { stopped = true; break; }
          continue;
        }
        if (!admitted) { ++s.stale_or_duplicate; continue; }
        s.accept(row.observed);
        combiner.push(motion);
      }
      if (stopped) break;
      // Match the retained provider-poll groups, not one drain per sensor.
      if (event.row + 1 == dequeues.size() || dequeues[event.row + 1].poll != row.poll) {
        for (const auto& value : combiner.drain(row.observed, max_age)) {
          ++combined;
          if (combined_previous) combined_max_gap = std::max(combined_max_gap, value.gyro.acquisition_ns - combined_previous);
          combined_previous = value.gyro.acquisition_ns;
        }
      }
    } else {
      const auto& row = images[event.row];
      check(row.kind >= 0 && row.kind < 4, "invalid image kind");
      auto& s = stats[size_t(row.kind) + 2];
      ++s.seen;
      const auto stamp = clock == ClockSource::Sdk ? std::optional<int64_t>(row.host) : mapper.map(row.device);
      if (!stamp) { ++s.unavailable; continue; }
      s.sample(*stamp, row.observed, row.host, 0);
      image_receipt[row.kind] = row.observed;
      image_stamp[row.kind] = *stamp;
      image_sdk[row.kind] = row.host;
      if (row.kind % 2 == 1 && image_receipt[row.kind - 1] == row.observed) {
        const auto pair = size_t(row.kind / 2);
        ++paired[pair];
        const auto skew = *stamp - image_stamp[row.kind - 1];
        const auto sdk_skew = row.host - image_sdk[row.kind - 1];
        max_pair_skew[pair] = std::max(max_pair_skew[pair], std::abs(skew));
        max_pair_skew_change[pair] = std::max(max_pair_skew_change[pair], std::abs(skew - sdk_skew));
      }
      // Event diagnostics only: the trace lacks exposure midpoints/full Frame.
      if (*stamp > row.observed) ++s.invalid;
      else if (row.observed - *stamp > max_age) ++s.stale_or_duplicate;
      else s.accept(row.observed);
    }
  }
  std::cout << "{\"clock\":\"" << (clock == ClockSource::Sdk ? "sdk" : "local")
    << "\",\"fault_policy\":\"" << (policy == FaultPolicy::Stop ? "stop" : "drop")
    << "\",\"stopped\":" << (stopped ? "true" : "false") << ",\"clock_faults\":" << faults
    << ",\"raw_epoch_faults\":" << raw_faults << ",\"first_fault_relative_ns\":" << first_fault_at
    << ",\"processed_span_ns\":" << observed_end - arrivals.front().observed
    << ",\"combined\":" << combined << ",\"combined_rejected\":" << combiner.rejected()
    << ",\"combined_overflows\":" << combiner.overflows() << ",\"combined_max_gap_ns\":" << combined_max_gap
    << ",\"min_rate_ppb\":" << mapper.min_rate_ppb() << ",\"max_rate_ppb\":" << mapper.max_rate_ppb()
    << ",\"image_pair_counts\":[" << paired[0] << ',' << paired[1] << ']'
    << ",\"max_image_pair_skew_ns\":[" << max_pair_skew[0] << ',' << max_pair_skew[1] << ']'
    << ",\"max_image_pair_skew_change_from_sdk_ns\":[" << max_pair_skew_change[0] << ',' << max_pair_skew_change[1] << ']'
    << ",\"streams\":[";
  for (size_t i = 0; i < stats.size(); ++i) { if (i) std::cout << ','; stats[i].print(); }
  std::cout << "]}\n";
}

void self_test() {
  Mapper mapper;
  constexpr int64_t base = 1000000000000;
  for (int64_t i = 1; i <= 201; ++i) mapper.observe(i * 5000000, base + i * 5000000 + 2000000);
  check(mapper.ready(), "startup never ready");
  const auto old = mapper.map(1005000000);
  check(bool(old), "ready stamp absent");
  int64_t previous = *old;
  // A receipt-delay step must not rewrite old maps or introduce clock steps.
  for (int64_t i = 202; i <= 1000; ++i) {
    const auto device = i * 5000000;
    mapper.observe(device, base + device + 2000000 + (i > 500 ? 120000000 : 0));
    const auto stamp = mapper.map(device);
    check(stamp && *stamp > previous, "non-increasing local map");
    check(*stamp - previous <= 5025001, "slew bound exceeded");
    previous = *stamp;
  }
  check(mapper.map(1005000000) == old, "past mapping changed");
  check(!mapper.map(5000000), "pre-start event invented");
  const auto middle = mapper.map(4900000000), left = mapper.map(4890000000), right = mapper.map(4910000000);
  check(left && middle && right && *left < *middle && *middle < *right, "exposure order lost");
  for (int64_t i = 1001; i <= 5000; ++i) mapper.observe(i * 5000000, base + i * 5000000 + 122000000);
  check(!mapper.map(1005000000), "expired history retained without bound");
  RawClockGuard raw;
  raw.observe(0, 100, 1);
  bool reset = false;
  try { raw.observe(0, 90, 2); } catch (const std::runtime_error&) { reset = true; }
  check(reset, "raw reset hidden");
  // Graceful SDK policy keeps the gate's high-water mark and recovers by dropping.
  MotionGate gate(5000000);
  auto motion = [](uint64_t sequence, int64_t acquired, int64_t received) {
    return Motion{MotionKind::Accelerometer, sequence, acquired, received, {0, 0, 0}};
  };
  check(gate.admit(motion(1, base, base + 1000000), base + 1000000, max_age), "initial rejected");
  unsigned drops = 0;
  for (const auto sample : {motion(2, base - 1000, base + 2000000), motion(3, base + 20000000, base + 3000000)})
    try { gate.admit(sample, sample.received_ns, max_age); } catch (const std::runtime_error&) { ++drops; }
  check(drops == 2 && gate.admit(motion(4, base + 15000000, base + 16000000), base + 16000000, max_age), "graceful recovery failed");
  std::cout << "{\"self_test_passed\":true,\"mapper_bytes\":" << sizeof(Mapper) << "}\n";
}

void benchmark() {
  using Clock = std::chrono::steady_clock;
  Mapper mapper;
  std::vector<int64_t> timings;
  timings.reserve(100000);
  int64_t checksum = 0;
  const auto start = Clock::now();
  for (int64_t i = 1; i <= 100000; ++i) {
    const int64_t device = i * 5000000;
    const auto begin = Clock::now();
    mapper.observe(device, 1000000000000 + device + 2000000 + (i % 7) * 100000);
    checksum ^= mapper.map(device).value_or(0);
    checksum ^= mapper.map(device - 10000000).value_or(0);
    timings.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - begin).count());
  }
  const auto total = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
  std::sort(timings.begin(), timings.end());
  std::cout << "{\"iterations\":100000,\"work_per_iteration\":\"observe_plus_two_conversions\",\"mapper_bytes\":" << sizeof(Mapper)
    << ",\"p50_ns\":" << timings[50000] << ",\"p95_ns\":" << timings[95000] << ",\"p99_ns\":" << timings[99000]
    << ",\"max_ns\":" << timings.back() << ",\"total_ns\":" << total << ",\"checksum\":" << checksum << "}\n";
}

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") self_test();
    else if (argc == 2 && std::string(argv[1]) == "--benchmark") benchmark();
    else if (argc == 4) {
      const std::string clock = argv[2], policy = argv[3];
      check(clock == "sdk" || clock == "local", "clock must be sdk or local");
      check(policy == "stop" || policy == "drop", "policy must be stop or drop");
      replay(argv[1], clock == "sdk" ? ClockSource::Sdk : ClockSource::Local,
             policy == "stop" ? FaultPolicy::Stop : FaultPolicy::Drop);
    } else throw std::runtime_error("usage: replay TRACE sdk|local stop|drop; --self-test; --benchmark");
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
