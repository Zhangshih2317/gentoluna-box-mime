#include "wrist_adapter.hpp"
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdio>

// Built into the private, source-built SDK for apex-robot.service only.
int Wrist485Receive(FXTerminalType, unsigned, FXChnType *, unsigned char *, unsigned *);
namespace {
using namespace std::chrono_literals;
constexpr char lease_path[]="/run/luna-apex-wrist/enabled";

struct Lease {
    ino_t inode{};
    pid_t pid{};
    bool operator==(const Lease &other) const { return inode==other.inode && pid==other.pid; }
};
Lease selected_tool() {
    int fd=open(lease_path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);
    if (fd<0) return {};
    struct stat file{};
    struct flock test{};
    test.l_type=F_WRLCK;
    test.l_whence=SEEK_SET;
    const bool selected=fstat(fd,&file)==0 && S_ISREG(file.st_mode) &&
        file.st_uid==geteuid() && fcntl(fd,F_GETLK,&test)==0 && test.l_type==F_WRLCK;
    close(fd);
    return selected ? Lease{file.st_ino,test.l_pid} : Lease{};
}

class WristRuntime {
    std::atomic_bool linked{false};
    std::thread manager;
    void run() noexcept {
        try {
            rclcpp::NodeOptions options;
            options.use_global_arguments(false);
            auto node=std::make_shared<rclcpp::Node>("apex_ctag_wrist",options);
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(node);
            std::unique_ptr<wrist::Adapter> adapter;
            Lease active;
            while (linked && rclcpp::ok()) {
                const auto selection=selected_tool();
                if (adapter && !(selection==active)) {
                    adapter.reset();
                    RCLCPP_INFO(node->get_logger(),"Apex CTAG Tool detached");
                }
                if (selection.pid>0 && !adapter) {
                    active=selection;
                    const char *mode=std::getenv("LUNA_APEX_WRIST_MODE");
                    const bool motion=mode && std::string(mode)=="vr";
                    adapter=std::make_unique<wrist::Adapter>(*node,motion,
                        wrist::TerminalApi{&Wrist485Receive},[selection]{return selected_tool()==selection;});
                    RCLCPP_INFO(node->get_logger(),"Apex CTAG Tool attached (%s)",motion?"vr":"inspect");
                }
                executor.spin_some(5ms);
                std::this_thread::sleep_for(10ms);
            }
            adapter.reset();
            executor.remove_node(node);
        } catch (const std::exception &error) {
            std::fprintf(stderr,"Apex CTAG Tool disabled: %s\n",error.what());
        }
    }
public:
    void start() noexcept {
        if (!rclcpp::ok() || linked.exchange(true)) return;
        try { manager=std::thread(&WristRuntime::run,this); }
        catch (const std::exception &error) {
            linked=false;
            std::fprintf(stderr,"Apex CTAG Tool startup failed: %s\n",error.what());
        }
    }
    void stop() {
        linked=false;
        if (manager.joinable()) manager.join();
    }
    ~WristRuntime() { stop(); }
};
WristRuntime &runtime() { static WristRuntime value; return value; }
}

void Wrist485Linked() { runtime().start(); }
void Wrist485Unlinking() { runtime().stop(); }
