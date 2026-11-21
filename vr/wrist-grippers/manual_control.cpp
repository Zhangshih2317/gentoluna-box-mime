#include "manual_motion.hpp"
#include "wrist_adapter.hpp"
#include <charconv>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted=1; }
void check_session() {
    pollfd input{STDIN_FILENO,POLLIN,0};
    const int result=poll(&input,1,0);
    // The launcher keeps stdin open, and requests cancellation by closing it.
    if (interrupted || result<0 || (result>0 && input.revents))
        throw std::runtime_error("manual command interrupted");
}
int percent(const std::string &s) {
    int value=0;
    const auto end=s.data()+s.size();
    const auto parsed=std::from_chars(s.data(),end,value);
    wrist::require(parsed.ec==std::errc{} && parsed.ptr==end && value>=60 && value<=100,
                   "speed and torque must be integer percentages in [60,100]");
    return value;
}
class Lock {
    int fd;
public:
    explicit Lock(const char *path):fd(open(path,O_WRONLY|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0660)) {
        if (fd<0 || flock(fd,LOCK_EX|LOCK_NB)!=0) {
            if (fd>=0) close(fd);
            throw std::runtime_error("controller or gripper lock unavailable");
        }
    }
    ~Lock(){close(fd);}
};
}

int main(int argc,char **argv) {
    try {
        wrist::require(argc==5,"usage: wrist_manual status|open|close left|right|both SPEED TORQUE");
        const std::string action=argv[1], side=argv[2];
        wrist::require(action=="status" || action=="open" || action=="close","invalid action");
        wrist::require(side=="left" || side=="right" || side=="both","invalid side");
        const int speed=percent(argv[3]), torque=percent(argv[4]);
        std::vector<int> sides;
        if (side!="right") sides.push_back(0);
        if (side!="left") sides.push_back(1);
        Lock controller("/tmp/gento_sdk_6_6_7_190.lock"), grippers("/tmp/luna-gripper-sdk.lock");
        wrist::require(std::system(
            "for unit in apex-robot apex-teleop apex-tool; do "
            "test \"$(systemctl show \"$unit\" -p ActiveState --value)\" = inactive || exit 1; done; "
            "ports=$(ss -Huln) || exit 1; if printf '%s\\n' \"$ports\" | grep -Eq ':372[0-4][[:space:]]'; "
            "then exit 1; fi")==0,"Apex or controller ports in use; release the controller first");
        for (int sig : {SIGINT,SIGTERM,SIGHUP}) std::signal(sig,interrupt);
        check_session();
        wrist::require(FX_L1_System_Link(6,6,7,190,0)>=0,"could not link controller");
        struct Disconnect { ~Disconnect(){ FX_L1_System_Unlink(); } } disconnect;
        wrist::SharedTerminal bus;
        const int target=action=="close" ? 9000 : action=="open" ? 0 : -1;
        wrist::run_manual(bus,sides,target,speed,torque,check_session);
        std::cout << "Completed " << action << " " << side << std::endl;
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
