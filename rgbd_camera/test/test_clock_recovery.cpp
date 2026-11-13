// Copyright (c) 2026 Paul N. Beuchat. MIT License.
#include "rgbd_camera/clock_recovery.hpp"
#include "rgbd_camera/imu_combiner.hpp"
#include <iostream>
#include <stdexcept>
using namespace rgbd_camera;
void check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
template<class F> void throws(F f) { bool failed=false; try { f(); } catch (const std::exception&) { failed=true; } check(failed,"expected terminal fault"); }
Products products(int64_t raw, int64_t offset, uint64_t seq, int64_t received) {
  Products p; p.frame = std::make_unique<Frame>(); auto& f=*p.frame;
  f.sequence=f.depth_sequence=seq; f.acquisition_ns=f.depth_acquisition_ns=raw+offset; f.received_ns=received;
  f.clocks={{{raw,raw+offset},{raw,raw+offset}}};
  p.motion.push_back({MotionKind::Accelerometer,seq,raw+offset,received,{0,0,9.81},{raw,raw+offset}});
  p.motion.push_back({MotionKind::Gyroscope,seq,raw+offset,received,{0,0,0},{raw,raw+offset}});
  return p;
}
int main() {
  try {
    constexpr int64_t offset=10000000000LL;
    MotionGate gate(5000000,true);
    Motion good{MotionKind::Accelerometer,1,1000000000,1001000000,{0,0,1},{}};
    check(gate.admit(good,1001000000,500000000),"initial report rejected");
    auto bad=good; bad.sequence=2; bad.acquisition_ns=2000000000;
    check(!gate.admit(bad,1001000000,500000000),"future report admitted");
    good.sequence=3; good.acquisition_ns+=5000000; good.received_ns+=5000000;
    check(gate.admit(good,good.received_ns,500000000),"future report poisoned watermark");
    bad=good; bad.sequence=4; bad.acquisition_ns-=1000000;
    check(!gate.admit(bad,bad.received_ns,500000000),"backward report admitted");
    good.sequence=5; good.acquisition_ns+=5000000; good.received_ns+=5000000;
    check(gate.admit(good,good.received_ns,500000000) && gate.clock_rejected()==2,"drop count/recovery wrong");
    bad=good; bad.received_ns=good.received_ns+1;
    throws([&] {gate.admit(bad,good.received_ns,500000000);});
    Config config; config.enable_imu=true;
    ClockRecovery clock(config,5000000);
    int64_t raw=1000000000; uint64_t seq=1;
    auto p=products(raw,offset,seq,raw+offset+250000000);
    check(!clock.inspect(p,p.frame->received_ns),"normal poll held");
    // True transport loss advances both clocks; it is not a mapping jump.
    raw+=1000000000; seq+=200;
    p=products(raw,offset,seq,raw+offset+250000000);
    check(!clock.inspect(p,p.frame->received_ns),"true gap treated as clock jump");
    raw+=5000000; ++seq;
    p=products(raw,offset+182000000,seq,raw+offset+250000000);
    // Images still in the old clock correction when the IMU batch changes.
    p.frame->acquisition_ns=p.frame->depth_acquisition_ns=raw+offset;
    p.frame->clocks={{{raw,raw+offset},{raw,raw+offset}}};
    check(clock.inspect(p,p.frame->received_ns) && clock.recovering() && clock.discontinuities()==1,"shared jump missed");
    for(int i=0;i<110;++i) {
      raw+=20000000; ++seq;
      p=products(raw,offset+182000000,seq,raw+offset+250000000);
      const bool held=clock.inspect(p,p.frame->received_ns);
      if(i<100) check(held,"recovery admitted mixed/unsettled timing");
    }
    check(!clock.recovering() && clock.discontinuities()==1,"did not settle once");
    raw+=20000000; ++seq;
    p=products(raw,offset+182000000,seq,raw+offset+250000000);
    check(!clock.inspect(p,p.frame->received_ns),"valid continuation held");
    p.motion[0].clock.device_ns-=100000000;
    throws([&] {clock.inspect(p,p.frame->received_ns);});
    config.timestamp_policy="strict"; ClockRecovery strict(config,5000000);
    p=products(raw,offset,seq,raw+offset+250000000); strict.inspect(p,p.frame->received_ns);
    p=products(raw+5000000,offset+182000000,seq+1,raw+offset+255000000);
    throws([&] {strict.inspect(p,p.frame->received_ns);});
    // No acceleration bracket survives a timing-generation boundary.
    ImuCombiner combiner(10000000);
    combiner.push({MotionKind::Accelerometer,1,1000000000,1100000000,{0,0,1},{}});
    combiner.push({MotionKind::Gyroscope,1,1005000000,1100000000,{0,0,0},{}});
    combiner.reset_timing();
    combiner.push({MotionKind::Accelerometer,2,1007000000,1100000000,{0,0,2},{}});
    check(combiner.drain(1100000000,500000000).empty() && combiner.rejected()==1,"old gyro crossed boundary");
    std::cout << "clock recovery contract passed\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
