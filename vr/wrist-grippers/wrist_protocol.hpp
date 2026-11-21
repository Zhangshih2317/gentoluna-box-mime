#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace wrist {
using Bytes = std::vector<uint8_t>;
using Words = std::vector<uint16_t>;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr int minimum_motion_percent = 60;
constexpr uint16_t drop_alarm_b = 0x1000;

inline void require(bool ok, const char *why) {
    if (!ok) throw std::runtime_error(why);
}

inline std::string transaction_error(int side, const Bytes &request, std::size_t received,
                                     const char *reason) {
    char text[160];
    std::snprintf(text,sizeof(text),"ARM%d/485A function=0x%02x register=0x%02x%02x received=%zu: %s",
                  side,unsigned(request.at(1)),unsigned(request.at(2)),unsigned(request.at(3)),
                  received,reason);
    return text;
}

struct RegisterTransportError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct EmptyRegisterReply : RegisterTransportError {
    using RegisterTransportError::RegisterTransportError;
};

struct MissingStartAck : std::runtime_error {
    using std::runtime_error::runtime_error;
};

inline uint16_t crc(const Bytes &msg) {
    uint16_t value = 0xffff;
    for (auto byte : msg) {
        value ^= byte;
        for (int bit = 0; bit < 8; ++bit)
            value = (value >> 1) ^ ((value & 1) ? 0xa001 : 0);
    }
    return value;
}

inline void append_word(Bytes &bytes, uint16_t word) {
    bytes.push_back(static_cast<uint8_t>(word >> 8));
    bytes.push_back(static_cast<uint8_t>(word));
}

inline Bytes with_crc(Bytes msg) {
    auto sum = crc(msg);
    msg.push_back(static_cast<uint8_t>(sum));
    msg.push_back(static_cast<uint8_t>(sum >> 8));
    return msg;
}

// Each side needs a fresh release after a disconnect. A neutral sample alone
// must never open a gripper that is already holding something.
struct Trigger {
    Clock::time_point seen{};
    bool released = false;
    bool pressed = false;
    int target = -1;
    uint64_t revision = 0;

    void reset() {
        released = pressed = false;
        target = -1;
        seen = {};
        ++revision;
    }
    void update(float value, Clock::time_point now) {
        if (!std::isfinite(value) || value < 0 || value > 1) {
            reset();
            return;
        }
        if (seen != Clock::time_point{} && now - seen > 300ms) reset();
        seen = now;
        if (value <= 0.2f) {
            released = true;
            if (pressed) {
                pressed = false;
                target = 0;
                ++revision;
            }
        } else if (released && !pressed && value >= 0.8f) {
            pressed = true;
            target = 9000;
            ++revision;
        }
    }
    bool fresh(Clock::time_point now) const {
        return seen != Clock::time_point{} && now - seen <= 300ms;
    }
};

struct Feedback {
    int32_t position;
    int16_t speed;
    uint16_t status, fault, alarm;
    bool stationary() const { return speed == 0 && (status & 4); }
    static void check_drive(uint16_t fault, uint16_t alarm) {
        if (fault || (alarm & ~drop_alarm_b))
            throw std::runtime_error("drive fault A=" + std::to_string(fault) +
                                     " alarm B=" + std::to_string(alarm));
    }
    void check() const {
        check_drive(fault,alarm);
        require(position >= -20 && position <= 9020, "position outside verified travel");
    }
};

// Exchange supplies one *complete* RTU reply on the selected arm's 485A port.
template<class Exchange> class Motor {
    Exchange &bus;
    int side;
    Bytes request(Bytes frame, std::size_t expected, bool verify_stage_by_readback=false) {
        auto reply = bus(side, with_crc(frame), expected);
        const bool damaged=reply.size()<5 || crc(reply)!=0;
        // Only the non-actuating parameter write has an independent six-word readback.
        if (verify_stage_by_readback && damaged) return {};
        if (reply.empty() && frame[1]==6 && frame[2]==1 && frame[3]==8) {
            if (frame[5]==1)
                throw MissingStartAck(transaction_error(side,frame,0,"start acknowledgement missing"));
            throw std::runtime_error(transaction_error(side,frame,0,"stop acknowledgement missing"));
        }
        if (reply.empty() && frame[1]==3)
            throw EmptyRegisterReply(transaction_error(side,frame,0,"empty wrist register reply"));
        if (frame[1]==3 && damaged)
            throw RegisterTransportError(transaction_error(side,frame,reply.size(),"damaged wrist register reply"));
        if (damaged || reply[0]!=1)
            throw std::runtime_error(transaction_error(side,frame,reply.size(),"invalid RTU reply"));
        if (reply[1]!=frame[1] || reply.size()!=expected)
            throw std::runtime_error(transaction_error(side,frame,reply.size(),"RTU exception or bad length"));
        return reply;
    }
public:
    Motor(Exchange &transport, int arm) : bus(transport), side(arm) {}
    Words read(uint16_t address, uint16_t count) {
        Bytes frame{1, 3};
        append_word(frame, address);
        append_word(frame, count);
        auto reply = request(frame, 5 + 2 * count);
        require(reply[2] == count * 2, "unexpected register count");
        Words result;
        for (unsigned i = 0; i < count; ++i)
            result.push_back((reply[3 + 2*i] << 8) | reply[4 + 2*i]);
        return result;
    }
private:
    Feedback read_position(const Words &v) {
        auto p = read(0x0418, 3);
        return {static_cast<int32_t>((uint32_t(p[0]) << 16) | p[1]),
                static_cast<int16_t>(p[2]), v[0], v[1], v[2]};
    }
public:
    Feedback feedback() { return read_position(read(0x0401,3)); }
    Words checked_status() {
        const auto v=read(0x0401,3);
        Feedback::check_drive(v[1],v[2]);
        return v;
    }
    std::optional<Feedback> observe(uint16_t &status) {
        const auto v=checked_status();
        status=v[0];
        try { return read_position(v); }
        catch (const RegisterTransportError &) { return std::nullopt; }
    }
    void inspect() {
        std::string model;
        for (auto w : read(0x0b00, 16)) {
            model.push_back(static_cast<char>(w >> 8));
            model.push_back(static_cast<char>(w));
        }
        while (!model.empty() && (model.back() == '\0' || model.back() == ' ')) model.pop_back();
        require(model == "CTAG2F90D", "unexpected gripper model");
        require(read(0x0504,4) == Words{0,0,0,9000}, "unexpected travel limits");
        require(read(0x0116,1) == Words{0} && read(0x0119,1) == Words{1}, "unexpected position units or motion mode");
        require(read(0x0a00,3) == Words{1,6,1}, "unexpected RTU communication settings");
        check_mode();
        auto state = feedback();
        state.check();
        require(state.stationary(), "gripper not stationary at preflight");
    }
    void check_mode() {
        require(read(0x0100,2) == Words{1,0}, "drive not enabled in absolute position mode");
    }
    void prepare(int target, int speed=minimum_motion_percent, int torque=minimum_motion_percent,
                 bool require_position=true) {
        require(target == 0 || target == 9000, "invalid target");
        require(speed >= minimum_motion_percent && speed <= 100, "speed must be 60..100 percent");
        require(torque >= minimum_motion_percent && torque <= 100, "torque must be 60..100 percent");
        check_mode();
        if (require_position) feedback().check();
        else checked_status();
        Words params{0, static_cast<uint16_t>(target), static_cast<uint16_t>(speed),
                     static_cast<uint16_t>(torque), 60, 60};
        Bytes frame{1,16,1,2,0,6,12};
        for (auto w : params) append_word(frame,w);
        const auto reply = request(frame,8,true);
        if (!reply.empty())
            require(std::equal(reply.begin(),reply.begin()+6,frame.begin()), "target acknowledgement mismatch");
        const auto applied = read(0x0102,6);
        if (reply.empty() && applied!=params)
            throw RegisterTransportError(transaction_error(side,frame,0,"unacknowledged target failed readback"));
        require(applied == params, "target readback mismatch");
    }
    void command(bool stop) {
        Bytes frame{1,6,1,8,0,static_cast<uint8_t>(stop ? 3 : 1)};
        require(request(frame,8) == with_crc(frame), "trigger or stop acknowledgement mismatch");
    }
};
}
