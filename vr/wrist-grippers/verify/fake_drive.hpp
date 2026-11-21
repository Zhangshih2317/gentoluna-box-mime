#pragma once

#include "../wrist_protocol.hpp"
#include <cassert>

namespace wrist {
struct FakeDrive {
    bool corrupt = false, mismatch = false;
    bool hold_motion = false, moving = false;
    uint16_t position = 4200;
    uint16_t fault = 0, alarm = 0x1000;
    Words params;
    unsigned starts = 0, stops = 0;

    Bytes exchange(const Bytes &frame) {
        assert(frame.size() >= 8 && crc(frame) == 0 && frame[0] == 1);
        Bytes reply;
        if (frame[1] == 3) {
            unsigned address = (frame[2] << 8) | frame[3];
            Words values;
            switch (address) {
                case 0x0b00: values = {0x4354,0x4147,0x3246,0x3930,0x4400}; values.resize(16); break;
                case 0x0504: values = {0,0,0,9000}; break;
                case 0x0116: values = {0}; break;
                case 0x0119: values = {1}; break;
                case 0x0a00: values = {1,6,1}; break;
                case 0x0100: values = {1,0}; break;
                case 0x0401: values = {static_cast<uint16_t>(moving ? 0x8000 : 0x8005),fault,alarm}; break;
                case 0x0418: values = {0,position,static_cast<uint16_t>(moving ? 10 : 0)}; break;
                case 0x0102: values = params; if (mismatch) values.at(1) ^= 1; break;
                default: throw std::logic_error("unexpected register");
            }
            assert(values.size() == frame[5]);
            reply = {1,3,static_cast<uint8_t>(values.size()*2)};
            for (auto word : values) append_word(reply, word);
        } else if (frame[1] == 16) {
            assert(frame.size() == 21 && frame[2] == 1 && frame[3] == 2 && frame[5] == 6 && frame[6] == 12);
            params.clear();
            for (unsigned n=7; n<19; n+=2) params.push_back((frame[n]<<8)|frame[n+1]);
            reply.assign(frame.begin(),frame.begin()+6);
        } else {
            assert(frame[1]==6 && frame[2]==1 && frame[3]==8);
            if (frame[5]==3) {
                ++stops;
                moving=false;
            } else {
                assert(frame[5]==1);
                ++starts;
                moving=hold_motion;
                if (!moving) position=params.at(1);
            }
            return frame;
        }
        reply=with_crc(reply);
        if (corrupt) reply.back()^=1;
        return reply;
    }
};
}
