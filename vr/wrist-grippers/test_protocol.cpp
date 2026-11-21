#include "wrist_protocol.hpp"
#include "verify/fake_drive.hpp"
#include <cassert>
#include <iostream>

using namespace wrist;
struct Fake : FakeDrive {
    int expected_side = 0;
    std::optional<Bytes> stage_reply;
    unsigned stage_writes = 0, stage_reads = 0;
    Bytes operator()(int side, const Bytes &frame, size_t) {
        assert(side == expected_side);
        auto reply = exchange(frame);
        if (frame[1]==16) {
            ++stage_writes;
            if (stage_reply) return *stage_reply;
        }
        if (frame[1]==3 && frame[2]==1 && frame[3]==2) ++stage_reads;
        return reply;
    }
};
int main() {
    Trigger t;
    auto now=Clock::now();
    t.update(1,now);assert(t.target==-1);
    t.update(0,now);assert(t.target==-1);
    t.update(.85f,now);assert(t.target==9000);
    auto rev=t.revision;t.update(.6f,now);assert(t.revision==rev);
    t.update(.15f,now);assert(t.target==0);
    t.update(1,now+500ms);assert(t.target==-1);
    t.update(NAN,now);assert(!t.fresh(now));
    assert(with_crc({1,3,0,0,0,1})==Bytes({1,3,0,0,0,1,0x84,0x0a}));
    Fake f;
    for(int side=0;side<2;side++) {
        f.expected_side=side;Motor<Fake> m(f,side);m.inspect();
        m.prepare(9000);assert(f.params==Words({0,9000,60,60,60,60}));m.command(false);
        m.prepare(0);m.command(false);m.command(true);
        f.mismatch=true;bool rejected=false;
        try{m.prepare(0);}catch(const std::runtime_error&){rejected=true;}
        assert(rejected);f.mismatch=false;
        f.corrupt=true;rejected=false;
        try{m.feedback();}catch(const std::runtime_error&){rejected=true;}
        assert(rejected);f.corrupt=false;
    }
    assert(f.starts==4 && f.stops==2);
    Fake lost;
    Motor<Fake> motor(lost,0);
    for (const Bytes &fragment : std::vector<Bytes>{{},{1},{1,16,1},{6,0xe0,0x37},
                                                   {1,16,1,6,0xe0,0x37},{1,16,1,2,0,6,0,0}}) {
        lost.stage_reply=fragment;
        const auto before=lost.stage_writes;
        lost.mismatch=false;
        motor.prepare(9000);
        assert(lost.stage_writes==before+1 && lost.stage_reads==before+1 && lost.starts==0);
        lost.mismatch=true;
        bool cancelled=false;
        try {motor.prepare(0);} catch(const RegisterTransportError &) {cancelled=true;}
        assert(cancelled && lost.stage_writes==before+2 && lost.stage_reads==before+2 && lost.starts==0);
    }
    const auto reads=lost.stage_reads;
    for (const Bytes &bad : std::vector<Bytes>{with_crc({1,0x90,2}),with_crc({2,16,1,2,0,6}),
                                              with_crc({1,16,1,3,0,6})}) {
        lost.stage_reply=bad;
        bool rejected=false;
        try {motor.prepare(0);} catch(const std::runtime_error &) {rejected=true;}
        assert(rejected && lost.stage_reads==reads && lost.starts==0);
    }
    std::cout<<"passed: independent arms, preflight, release-before-press, timeout, CRC, target readback, start and stop\n";
}
