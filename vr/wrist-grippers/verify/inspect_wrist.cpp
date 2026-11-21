#include "wrist_adapter.hpp"

#include <iostream>

namespace {
struct ReadTerminal {
    wrist::SharedTerminal transport;
    bool paced = false;
    wrist::Bytes operator()(int side, const wrist::Bytes &frame, std::size_t size) {
        wrist::require(frame.size() == 8 && frame[0] == 1 && frame[1] == 3 &&
                       wrist::crc(frame) == 0, "maintenance probe accepts only RTU reads");
        if (paced) std::this_thread::sleep_for(std::chrono::milliseconds(150));
        auto reply = transport(side, frame, size);
        if (reply.size() != size || wrist::crc(reply) != 0 || reply[0] != 1 || reply[1] != 3)
            std::cerr << "ARM" << side << " read reg=0x" << std::hex
                      << ((frame[2]<<8)|frame[3]) << std::dec << " expected_bytes=" << size
                      << " actual_bytes=" << reply.size() << std::endl;
        return reply;
    }
};
}

int main(int argc, char **argv) {
    if ((argc != 2 && argc != 3) || std::string(argv[1]) != "--released-controller") return 64;
    const std::string mode = argc == 3 ? argv[2] : "";
    if (argc == 3 && mode != "--paced" && mode != "--preflight-retry") return 64;
    if (FX_L1_System_Link(6,6,7,190,0) < 0) {
        std::cerr << "Could not link to the released controller\n";
        return 1;
    }
    struct Disconnect { ~Disconnect() { FX_L1_System_Unlink(); } } disconnect;
    ReadTerminal terminal;
    terminal.paced = mode == "--paced";
    terminal.transport.retry_idle_reads = mode == "--preflight-retry";
    bool passed = true;
    for (int side = 0; side < 2; ++side) {
        const auto before = terminal.transport.retried_empty_reads;
        try {
            wrist::Motor<ReadTerminal> motor(terminal, side);
            motor.inspect();
            const auto f = motor.feedback();
            f.check();
            wrist::require(f.stationary(), "gripper no longer stationary");
            std::cout << "PASS ARM" << side << "/485A CTAG2F90D"
                      << " position=" << f.position << " speed=" << f.speed
                      << " status=" << f.status << " fault=" << f.fault
                      << " alarm=" << f.alarm << " empty_read_retries="
                      << terminal.transport.retried_empty_reads-before << std::endl;
        } catch (const std::exception &error) {
            passed = false;
            std::cerr << "FAIL ARM" << side << "/485A " << error.what() << std::endl;
        }
    }
    return passed ? 0 : 1;
}
