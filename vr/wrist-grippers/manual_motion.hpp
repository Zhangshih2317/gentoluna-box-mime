#pragma once

#include "wrist_protocol.hpp"
#include <iostream>

namespace wrist {
inline void show_feedback(int side, const Feedback &f) {
    std::cout << "ARM" << side << "/485A position=" << f.position << " speed=" << f.speed
              << " status=0x" << std::hex << f.status << " fault_A=0x" << f.fault
              << " alarm_B=0x" << f.alarm << std::dec << std::endl;
}

template<class Transport, class Check>
void run_manual(Transport &bus, const std::vector<int> &sides, int target,
                int speed, int torque, Check check) {
    // -1 is the read-only command. The launcher accepts open, close or status.
    require(target == -1 || target == 0 || target == 9000, "invalid target");
    require(speed >= minimum_motion_percent && speed <= 100, "speed must be 60..100 percent");
    require(torque >= minimum_motion_percent && torque <= 100, "torque must be 60..100 percent");
    std::array<Motor<Transport>,2> motors{{{bus,0},{bus,1}}};
    std::array<bool,2> pending{};
    std::array<unsigned,2> stable{};
    int side = -1;
    try {
        bus.retry_idle_reads=true;
        for (int i : sides) {
            side=i;
            check();
            auto &motor=motors.at(i);
            motor.inspect();
            auto f=motor.feedback();
            show_feedback(i,f);
            f.check();
            require(f.stationary(),"drive moved during preflight");
        }
        if (target == -1) return;
        for (int i : sides) {
            side=i;
            check();
            motors[i].prepare(target,speed,torque);
        }
        bus.retry_idle_reads=false;
        bus.before_start=check;
        auto deadline=Clock::now()+12s;
        for (int i : sides) {
            side=i;
            check();
            pending[i]=true; // A missing ACK does not prove that the write failed.
            motors[i].command(false);
        }
        while (std::any_of(pending.begin(),pending.end(),[](bool v){return v;})) {
            for (int i : sides) {
                if (!pending[i]) continue;
                side=i;
                check();
                require(Clock::now()<deadline,"target timeout");
                const auto f=motors[i].feedback();
                f.check();
                const bool reached=f.stationary() && (f.status & 1) && std::abs(f.position-target)<=20;
                stable[i]=reached ? stable[i]+1 : 0;
                if (stable[i]>=3) {
                    pending[i]=false;
                    show_feedback(i,f);
                }
            }
        }
    } catch (const std::exception &error) {
        std::cerr << "ARM" << side << "/485A: " << error.what() << std::endl;
        bus.retry_idle_reads=false;
        // Issue both stops before waiting for either drive's feedback.
        for (int i : sides) if (pending[i]) {
            try { motors[i].command(true); }
            catch (const std::exception &e) {
                std::cerr << "ARM" << i << " stop acknowledgement missing: " << e.what() << std::endl;
            }
        }
        for (int i : sides) if (pending[i]) {
            try {
                auto end=Clock::now()+2s;
                do {
                    const auto f=motors[i].feedback();
                    if (f.stationary()) { pending[i]=false; break; }
                } while (Clock::now()<end);
            } catch (const std::exception &e) {
                std::cerr << "ARM" << i << " stop feedback: " << e.what() << std::endl;
            }
            std::cerr << "ARM" << i << (pending[i] ? " STOP UNCONFIRMED" : " stationary confirmed") << std::endl;
        }
        throw;
    }
}
}
