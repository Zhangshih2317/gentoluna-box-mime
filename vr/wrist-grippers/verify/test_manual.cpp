#include "../manual_motion.hpp"
#include "fake_drive.hpp"
#include <functional>
#include <iostream>

using namespace wrist;

struct ManualBus {
    std::array<FakeDrive,2> drives;
    std::function<void()> before_start;
    bool retry_idle_reads=false;
    bool lose_second_ack=false, alarm_after_start=false;
    unsigned exchanges=0;
    Bytes operator()(int side, const Bytes &frame, size_t) {
        ++exchanges;
        const bool start=frame[1]==6 && frame[5]==1;
        if (start) before_start();
        auto reply=drives[side].exchange(frame);
        if (start && side==1 && lose_second_ack) return {};
        if (start && alarm_after_start) drives[side].alarm=0x1004;
        return reply;
    }
};

template<class F> void rejected(F f) {
    try { f(); } catch(const std::exception &) { return; }
    throw std::logic_error("expected command rejection");
}

int main() {
    ManualBus bus;
    auto run=[&](std::vector<int> sides,int target,int speed=60,int torque=60){
        run_manual(bus,sides,target,speed,torque,[]{});
    };
    for (int bad : {59,101}) {
        rejected([&]{run({0,1},9000,bad,60);});
        rejected([&]{run({0,1},9000,60,bad);});
    }
    assert(!bus.exchanges);
    run({0,1},-1);
    assert(bus.drives[0].params.empty() && bus.drives[1].params.empty());
    run({0},9000);
    assert(bus.drives[0].params==Words({0,9000,60,60,60,60}) && bus.drives[0].position==9000);
    assert(bus.drives[1].starts==0 && bus.drives[0].alarm==drop_alarm_b);
    run({1},0,100,75);
    assert(bus.drives[1].params==Words({0,0,100,75,60,60}));
    run({0,1},0);
    for(auto &d:bus.drives) assert(d.position==0 && d.stops==0);
    bus={};
    bus.drives[1].mismatch=true;
    rejected([&]{run({0,1},9000);});
    assert(bus.drives[0].starts==0 && bus.drives[1].starts==0);
    bus={};
    bus.drives[0].fault=1;
    rejected([&]{run({0},9000);});
    assert(bus.drives[0].starts==0);
    bus={};
    bus.alarm_after_start=true;
    rejected([&]{run({0,1},9000);});
    for(auto &d:bus.drives) assert(d.alarm==0x1004 && d.starts==1 && d.stops==1);
    bus={};
    bus.lose_second_ack=true;
    rejected([&]{run({0,1},9000);});
    for(auto &d:bus.drives) assert(d.starts==1 && d.stops==1);
    std::cout<<"PASS manual selection, limits, drop alarm, mixed alarms, readback and lost ACK\n";
}
