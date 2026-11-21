#pragma once

#include "wrist_protocol.hpp"
#include "L1Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace wrist {
struct CancelledBeforeStart {};

struct TerminalApi {
    decltype(&FX_L1_Terminal_GetData) receive = &FX_L1_Terminal_GetData;
};

struct SharedTerminal {
    TerminalApi api{};
    // Each wrist has its own RX queue. Serialize individual calls into the shared SDK.
    std::mutex *sdk_mutex=nullptr;
    std::function<void()> before_start;
    bool retry_idle_reads = false;
    bool retry_quiet_reads = false;
    unsigned retried_empty_reads = 0;

    Bytes operator()(int side, const Bytes &request, std::size_t expected) {
        if ((!retry_idle_reads && !retry_quiet_reads) || request.at(1) != 3)
            return exchange_once(side, request, expected);
        const unsigned attempts = retry_idle_reads ? 5 : 3;
        for (unsigned attempt = 1; ; ++attempt) {
            if (retry_idle_reads || attempt > 1) std::this_thread::sleep_for(150ms);
            auto reply = exchange_once(side, request, expected);
            // An intact exception or an unexpected device/function goes to the protocol checker.
            if (attempt == attempts || (reply.size() >= 5 && crc(reply) == 0)) return reply;
            if (reply.empty()) ++retried_empty_reads;
        }
    }
private:
    template<class Call>
    int sdk(Call call) {
        if (!sdk_mutex) return call();
        std::lock_guard<std::mutex> lock(*sdk_mutex);
        return call();
    }
    Bytes exchange_once(int side, const Bytes &request, std::size_t expected) {
        const auto arm = side == 0 ? FX_TERMINAL_ARM0 : FX_TERMINAL_ARM1;
        std::array<unsigned char,64> wire{};
        require(request.size() <= wire.size(), "oversize wrist request");
        std::copy(request.begin(), request.end(), wire.begin());
        require(sdk([&]{return FX_L1_Terminal_ClearData(arm);}) == 0, "terminal clear failed");
        std::this_thread::sleep_for(100ms);
        unsigned stamp = 0;
        require(sdk([&]{
            if (request.size()==8 && request[1]==6 && request[2]==1 && request[3]==8 &&
                request[4]==0 && request[5]==1) {
                require(static_cast<bool>(before_start), "missing drive trigger interlock");
                before_start();
            }
            return FX_L1_Terminal_SetData(arm, FX_CHN_485A, 250, wire.data(),request.size(),&stamp);
        }) == 0, "terminal send failed");
        Bytes reply;
        auto deadline = Clock::now() + 700ms;
        while (Clock::now() < deadline) {
            FXChnType channel = FX_CHN_485A;
            int n = sdk([&]{return api.receive(arm,sdk_mutex ? 5 : 30,&channel,wire.data(),&stamp);});
            if (n == FUNC_RET_OPERATION_TIMEOUT || n == 0) {
                if (sdk_mutex) std::this_thread::sleep_for(1ms);
                continue;
            }
            require(n > 0 && n <= 64 && channel == FX_CHN_485A, "unexpected wrist channel or receive error");
            reply.insert(reply.end(), wire.begin(), wire.begin()+n);
            if (reply.size() >= expected || (reply.size() >= 5 && (reply[1] & 0x80))) break;
        }
        return reply;
    }
};

class Adapter {
    using Bool = std_msgs::msg::Bool;
    using Float = std_msgs::msg::Float32;
    using State = std_msgs::msg::Int16MultiArray;
    using Status = std_msgs::msg::Int32MultiArray;
    rclcpp::Node &node;
    const bool actuate;
    const TerminalApi api;
    const std::function<bool()> tool_active;
    std::atomic_bool running{true};
    std::mutex mutex;
    std::mutex sdk_mutex;
    std::condition_variable wake;
    std::array<bool,2> ready{};
    Clock::time_point heartbeat{}, state_seen{};
    std::array<bool,2> movable{};
    std::array<Clock::time_point,2> side_enable{};
    std::array<Trigger,2> inputs{};
    rclcpp::Subscription<Bool>::SharedPtr connected;
    rclcpp::Subscription<State>::SharedPtr state;
    std::array<rclcpp::Subscription<Float>::SharedPtr,2> triggers;
    std::array<rclcpp::Subscription<Bool>::SharedPtr,2> enables;
    std::array<rclcpp::Publisher<Status>::SharedPtr,2> feedback_pub;
    std::array<std::thread,2> workers;
    std::once_flag shutdown;
    bool allowed(int side, Clock::time_point now) const {
        return actuate && running && ready[side] && movable[side] && now-state_seen < 500ms &&
               side_enable[side]!=Clock::time_point{} && now-side_enable[side]<300ms &&
               heartbeat != Clock::time_point{} && now-heartbeat < 1500ms;
    }
    Trigger intent(int side) {
        std::lock_guard<std::mutex> lock(mutex);
        auto now=Clock::now();
        auto &input=inputs[side];
        if ((!allowed(side,now) || !input.fresh(now)) && input.seen!=Clock::time_point{}) input.reset();
        return input;
    }
    void publish(int side,const Feedback &f) {
        Status msg;
        msg.data={f.position,f.speed,f.status,f.fault,f.alarm};
        feedback_pub[side]->publish(msg);
    }
    void set_ready(int side,bool value) {
        std::lock_guard<std::mutex> lock(mutex);
        inputs[side].reset();
        ready[side]=value;
    }
    void run_side(int i) {
        SharedTerminal bus;
        bus.api=api;
        bus.sdk_mutex=&sdk_mutex;
        Motor<SharedTerminal> motor(bus,i);
        bool inspected=false, moving=false, stop_sent=false;
        int target=0;
        uint64_t handled=0;
        Clock::time_point deadline{}, next_poll{};
        auto send_stop = [&] {
            bus.retry_quiet_reads=false;
            if (stop_sent) return;
            stop_sent=true;
            motor.command(true);
        };
        auto stationary = [&] {
            const auto f=motor.feedback();
            publish(i,f);
            f.check();
            require(f.stationary(),"wrist not stationary");
            return f.position;
        };
        auto confirm_stop = [&](bool reversing) {
            const auto end=Clock::now()+2s;
            bool previous_zero_speed=false;
            do {
                if (reversing) {
                    const bool zero_speed=(motor.checked_status()[0]&4)!=0;
                    if (zero_speed && previous_zero_speed) {moving=false; return;}
                    previous_zero_speed=zero_speed;
                } else {
                    const auto f=motor.feedback();
                    publish(i,f);
                    f.check();
                    if (f.stationary()) {
                        moving=false;
                        RCLCPP_INFO(node.get_logger(),"ARM%d stopped at position %d",i,f.position);
                        return;
                    }
                }
            } while (Clock::now()<end);
            throw std::runtime_error("wrist stop not confirmed");
        };
        while (running && rclcpp::ok()) {
            try {
                if (!inspected) {
                    bus.retry_idle_reads=true;
                    motor.inspect();
                    inspected=true;
                    set_ready(i,true);
                    RCLCPP_INFO(node.get_logger(),"CTAG ARM%d/485A passed read-only preflight",i);
                }
                bus.retry_idle_reads=!actuate;
                bus.retry_quiet_reads=false;
                const auto action=intent(i);
                if (moving && (action.target<0 || action.target!=target)) {
                    send_stop();
                    confirm_stop(action.target>=0);
                }
                if (action.target>=0 && action.revision!=handled) {
                    bus.retry_quiet_reads=!moving;
                    motor.prepare(action.target,minimum_motion_percent,minimum_motion_percent,false);
                    bus.retry_quiet_reads=false;
                    handled=action.revision;
                    bus.before_start=[&,action] {
                        auto current=intent(i);
                        if (current.revision!=action.revision || current.target!=action.target ||
                            (tool_active && !tool_active()))
                            throw CancelledBeforeStart{};
                        moving=true;
                        stop_sent=false;
                        target=action.target;
                        deadline=Clock::now()+12s;
                    };
                    motor.command(false);
                    RCLCPP_INFO(node.get_logger(),"ARM%d %s command acknowledged (target %d)",i,
                                target==0?"open":"close",target);
                    bus.before_start={};
                } else if (moving || Clock::now()>=next_poll) {
                    next_poll=Clock::now()+800ms;
                    uint16_t status=0;
                    const auto f=motor.observe(status);
                    if (f) {
                        publish(i,*f);
                        f->check();
                        if (moving && f->stationary() && (f->status&1) && std::abs(f->position-target)<=20)
                            moving=false;
                    } else if (moving && (status&5)==5 && (motor.checked_status()[0]&5)==5) {
                        moving=false;
                    }
                    if (moving) require(Clock::now()<deadline,"wrist motion supervision timed out");
                }
            } catch (const CancelledBeforeStart &) {
                bus.before_start={};
            } catch (const std::exception &error) {
                const auto since=Clock::now();
                set_ready(i,false);
                bus.before_start={};
                bus.retry_idle_reads=bus.retry_quiet_reads=false;
                RCLCPP_WARN(node.get_logger(),"ARM%d suspended: %s; first check in 3s",i,error.what());
                if (moving) {
                    try { send_stop(); }
                    catch (const std::exception &e) { RCLCPP_ERROR(node.get_logger(),"ARM%d stop: %s",i,e.what()); }
                }
                for (int round=1; round<=5; ++round) {
                    {
                        std::unique_lock<std::mutex> lock(mutex);
                        wake.wait_until(lock,since+round*3s,[&]{return !running.load();});
                    }
                    if (!running || !rclcpp::ok()) break;
                    try {
                        if (!inspected) {
                            motor.inspect();
                            inspected=true;
                            RCLCPP_INFO(node.get_logger(),"CTAG ARM%d/485A passed read-only preflight",i);
                        } else {
                            motor.check_mode();
                            auto position=stationary();
                            if (moving) require(stationary()==position,"wrist position not stable after stop");
                        }
                        moving=false;
                        set_ready(i,true);
                        RCLCPP_INFO(node.get_logger(),"ARM%d recovered on check %d/5; fresh gesture required",i,round);
                        break;
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(node.get_logger(),"ARM%d check %d/5 failed: %s",i,round,e.what());
                        if (round==5) {
                            RCLCPP_ERROR(node.get_logger(),"ARM%d latched off after five failed checks; %s",i,
                                         moving?"stop not confirmed":"commands inhibited");
                            return;
                        }
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        if (moving) {
            try {send_stop();}
            catch(const std::exception &e) { RCLCPP_ERROR(node.get_logger(),"ARM%d stop command failed: %s",i,e.what()); }
            try {confirm_stop(false);}
            catch(const std::exception &e) { RCLCPP_ERROR(node.get_logger(),"ARM%d stop unconfirmed: %s",i,e.what()); }
        }
    }
public:
    Adapter(rclcpp::Node &owner,bool enable_motion,TerminalApi api_={},
            std::function<bool()> tool_active_={}):node(owner),actuate(enable_motion),
            api(api_),tool_active(std::move(tool_active_)) {
        connected=node.create_subscription<Bool>("info/vr_connected",1,[this](Bool::ConstSharedPtr msg){
            std::lock_guard<std::mutex> lock(mutex);
            auto now=Clock::now();
            if (!msg->data || now-heartbeat>=1500ms) for(auto &input:inputs) input.reset();
            heartbeat=msg->data ? now : Clock::time_point{};
        });
        state=node.create_subscription<State>("info/robot_state",rclcpp::SensorDataQoS().keep_last(1),
            [this](State::ConstSharedPtr msg){
                std::lock_guard<std::mutex> lock(mutex);
                auto now=Clock::now();
                for (int i=0;i<2;i++) {
                    movable[i]=msg->data.size()>=2 && (msg->data[i]==1 || msg->data[i]==2);
                    if (!movable[i] || now-state_seen>=500ms) inputs[i].reset();
                }
                state_seen=now;
            });
        for (int i=0;i<2;i++) {
            std::string suffix=i==0?"L":"R";
            feedback_pub[i]=node.create_publisher<Status>("info/wrist485_"+suffix,1);
            enables[i]=node.create_subscription<Bool>("control/enable"+suffix,
                rclcpp::QoS(1).best_effort(),[this,i](Bool::ConstSharedPtr msg){
                    std::lock_guard<std::mutex> lock(mutex);
                    auto now=Clock::now();
                    if (!msg->data || now-side_enable[i]>=300ms) inputs[i].reset();
                    side_enable[i]=msg->data ? now : Clock::time_point{};
                });
            triggers[i]=node.create_subscription<Float>("control/gripperValue"+suffix,
                rclcpp::QoS(1).best_effort(),[this,i](Float::ConstSharedPtr msg){
                    std::lock_guard<std::mutex> lock(mutex);
                    auto now=Clock::now();
                    if (allowed(i,now)) {
                        const auto revision=inputs[i].revision;
                        inputs[i].update(msg->data,now);
                        if (inputs[i].target>=0 && inputs[i].revision!=revision)
                            RCLCPP_INFO(node.get_logger(),"ARM%d trigger %.2f requested %s",i,msg->data,
                                inputs[i].target==0?"open":"close");
                    } else inputs[i].reset();
                });
        }
        try {
            for (int i=0;i<2;++i) workers[i]=std::thread(&Adapter::run_side,this,i);
        } catch (...) { stop(); throw; }
    }
    void stop() {
        std::call_once(shutdown,[this]{
            { std::lock_guard<std::mutex> lock(mutex); running=false; }
            wake.notify_all();
            for (auto &worker:workers) if (worker.joinable()) worker.join();
        });
    }
    ~Adapter() { stop(); }
};
}
