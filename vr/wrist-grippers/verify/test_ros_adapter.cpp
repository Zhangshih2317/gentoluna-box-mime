#include "wrist_adapter.hpp"
#include "fake_drive.hpp"
#include <cstdlib>
#include <iostream>
#include <unistd.h>

using namespace wrist;

namespace {
// Only this executable defines the three vendor functions. It never links
// libGentoSDK and cannot acquire a controller connection.
std::mutex sim_mutex;
std::array<FakeDrive,2> drives;
std::array<Bytes,2> replies;
std::array<bool,2> expect_trigger{};
std::array<unsigned,2> trigger_settles{};
std::array<bool,2> empty_next_status{};
bool corrupt_next_mode_read = false;
unsigned mode_replies_corrupted = 0;
unsigned drop_mode_reads = 0;
bool drop_motion_status = false;
unsigned drop_stopped_status_reads = 0;
bool drop_right_stage_ack = false;
bool truncate_left_stage_ack = false;
bool discard_left_stage_write = false;
unsigned left_stage_writes = 0;
unsigned left_model_reads = 0;
bool lose_left_start_ack = false, lose_left_stop_ack = false;
unsigned right_stage_ack_drops = 0, right_stage_writes = 0;
bool drop_position = false;
std::array<unsigned,2> missing_positions{};
bool trace_left_stop=false, trace_left_checks=false;
unsigned bad_recovery_modes=0;
Clock::time_point left_stop_time{};
std::vector<Clock::time_point> left_checks;

size_t arm_index(FXTerminalType arm) {
    if (arm == FX_TERMINAL_ARM0) return 0;
    if (arm == FX_TERMINAL_ARM1) return 1;
    throw std::logic_error("unexpected terminal ID");
}

FakeDrive snapshot(size_t arm) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    return drives.at(arm);
}
}

int FX_L1_Terminal_ClearData(FXTerminalType arm) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    auto i = arm_index(arm);
    replies[i].clear();
    if (expect_trigger[i]) ++trigger_settles[i];
    expect_trigger[i] = false;
    return 0;
}

int FX_L1_Terminal_SetData(FXTerminalType arm, FXChnType channel, unsigned,
                           unsigned char data[64], unsigned len, unsigned *elapsed) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    assert(channel == FX_CHN_485A && len <= 64);
    *elapsed = 0;
    auto i = arm_index(arm);
    const auto old_params=drives[i].params;
    replies[i] = drives[i].exchange(Bytes(data, data + len));
    if (i==0 && data[1]==6 && data[5]==3 && trace_left_stop) {
        left_stop_time=Clock::now();
        left_checks.clear();
        trace_left_stop=false;
        trace_left_checks=true;
    }
    if (i==0 && data[1]==3 && data[2]==1 && data[3]==0 && trace_left_checks) {
        left_checks.push_back(Clock::now());
        if (bad_recovery_modes) {
            --bad_recovery_modes;
            replies[i]=with_crc({1,3,4,0,0,0,0});
        }
    }
    if (drop_position && data[1]==3 && data[2]==4 && data[3]==0x18) {
        replies[i].clear();
        ++missing_positions[i];
    }
    if (i==0 && data[1]==6 && data[2]==1 && data[3]==8) {
        auto &lose_ack = data[5]==1 ? lose_left_start_ack : lose_left_stop_ack;
        if (lose_ack) {
            replies[i].clear();
            lose_ack=false;
        }
    }
    if (i==0 && data[1]==3 && data[2]==0x0b && data[3]==0 && ++left_model_reads==1) {
        require(replies[i].size()==37,"unexpected model response size");
        replies[i].resize(35);
    }
    if (i==0 && data[1]==16) {
        ++left_stage_writes;
        if (truncate_left_stage_ack) {
            replies[i]={0x06,0xe0,0x37};
            truncate_left_stage_ack=false;
            if (discard_left_stage_write) {
                drives[i].params=old_params;
                discard_left_stage_write=false;
            }
        }
    }
    if (i==1 && data[1]==16) {
        ++right_stage_writes;
        if (drop_right_stage_ack) {
            replies[i].clear();
            drop_right_stage_ack=false;
            ++right_stage_ack_drops;
        }
    }
    if (i==0 && data[1]==3 && data[2]==1 && data[3]==0 && drop_mode_reads) {
        --drop_mode_reads;
        replies[i].clear();
    }
    if (i==1 && data[1]==3 && data[2]==4 && data[3]==1) {
        if (drives[i].moving && drop_motion_status) {
            replies[i].clear();
            drop_motion_status=false;
        } else if (!drives[i].moving && drives[i].stops && drop_stopped_status_reads) {
            replies[i].clear();
            --drop_stopped_status_reads;
        }
    }
    if (i==0 && corrupt_next_mode_read && data[1]==3 && data[2]==1 && data[3]==0) {
        replies[i].back()^=1;
        corrupt_next_mode_read=false;
        ++mode_replies_corrupted;
    }
    if (empty_next_status[i] && data[1]==3 && data[2]==4 && data[3]==1) {
        replies[i].clear();
        empty_next_status[i]=false;
    }
    expect_trigger[i] = data[1] == 3 && data[2] == 1 && data[3] == 2;
    return 0;
}

int FX_L1_Terminal_GetData(FXTerminalType arm, unsigned, FXChnType *channel,
                           unsigned char data[64], unsigned *elapsed) {
    std::lock_guard<std::mutex> lock(sim_mutex);
    auto &reply = replies[arm_index(arm)];
    if (reply.empty()) return FUNC_RET_OPERATION_TIMEOUT;
    *channel = FX_CHN_485A;
    *elapsed = 0;
    // A real terminal can hand back a reply in multiple reads.
    auto n = std::min<size_t>(5, reply.size());
    std::copy_n(reply.begin(), n, data);
    reply.erase(reply.begin(), reply.begin() + n);
    return static_cast<int>(n);
}

namespace {
using Bool = std_msgs::msg::Bool;
using Float = std_msgs::msg::Float32;
using State = std_msgs::msg::Int16MultiArray;
using Status = std_msgs::msg::Int32MultiArray;

class Input {
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr source;
    rclcpp::Publisher<Bool>::SharedPtr heartbeat;
    rclcpp::Publisher<State>::SharedPtr state;
    std::array<rclcpp::Publisher<Bool>::SharedPtr,2> enable;
    std::array<rclcpp::Publisher<Float>::SharedPtr,2> trigger;
    std::array<rclcpp::Subscription<Status>::SharedPtr,2> feedback;
public:
    bool connected = true;
    std::array<bool,2> grip{true,true};
    std::array<float,2> value{1,1};
    std::array<int,2> position{-1,-1};

    explicit Input(rclcpp::Node::SharedPtr owner) {
        source = std::make_shared<rclcpp::Node>("test_inputs", owner->get_namespace());
        executor.add_node(owner);
        executor.add_node(source);
        heartbeat = source->create_publisher<Bool>("info/vr_connected", 1);
        state = source->create_publisher<State>("info/robot_state", 1);
        for (int i=0; i<2; ++i) {
            std::string suffix = i ? "R" : "L";
            enable[i] = source->create_publisher<Bool>("control/enable"+suffix, 1);
            trigger[i] = source->create_publisher<Float>("control/gripperValue"+suffix, 1);
            feedback[i] = source->create_subscription<Status>("info/wrist485_"+suffix, 1,
                [this,i](Status::ConstSharedPtr msg) {
                    require(msg->data.size()==5 && msg->data[4]==0x1000,"bad feedback");
                    position[i]=msg->data[0];
                });
        }
    }
    void tick() {
        Bool flag;
        flag.data = connected;
        heartbeat->publish(flag);
        State mode;
        mode.data = {2,2};
        state->publish(mode);
        for (int i=0; i<2; ++i) {
            flag.data = grip[i];
            enable[i]->publish(flag);
            Float msg;
            msg.data = value[i];
            trigger[i]->publish(msg);
        }
        executor.spin_some();
        std::this_thread::sleep_for(10ms);
    }
    void dwell(std::chrono::milliseconds duration) {
        auto end=Clock::now()+duration;
        while (Clock::now()<end) tick();
    }
    template<typename Predicate>
    void until(const char *description, Predicate predicate,std::chrono::seconds timeout=10s) {
        auto end=Clock::now()+timeout;
        while (!predicate() && Clock::now()<end) tick();
        require(predicate(),description);
        std::cout << "PASS " << description << std::endl;
    }
};

void verify() {
    auto owner=std::make_shared<rclcpp::Node>("test_owner","/wrist_test_"+std::to_string(getpid()));
    Input input(owner);
    std::atomic_bool tool_enabled{true};
    Adapter adapter(*owner,true,{},[&]{return tool_enabled.load();});
    input.until("both preflights and ROS feedback", [&]{return input.position[0]==4200 && input.position[1]==4200;});
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(left_model_reads==2 && drives[0].starts==0 && drives[1].starts==0,
                "short model reply was not recovered with one read-only retry");
    }
    input.dwell(800ms);
    require(snapshot(0).starts==0 && snapshot(1).starts==0,"startup press actuated");
    input.value={0,0};
    input.dwell(600ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drop_mode_reads=3;
    }
    input.value[0]=1;
    input.until("three missing mode replies before start",[]{
        std::lock_guard<std::mutex> lock(sim_mutex); return drop_mode_reads==0;
    });
    input.dwell(4900ms);
    require(snapshot(0).starts==0,"held press replayed after missing mode reads");
    input.value[0]=0;
    input.dwell(500ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        empty_next_status[0]=true;
    }
    input.until("empty reply on idle ARM0",[]{
        std::lock_guard<std::mutex> lock(sim_mutex); return !empty_next_status[0];
    });
    input.dwell(4900ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        truncate_left_stage_ack=true;
        discard_left_stage_write=true;
        drives[0].params={0,0,60,60,60,60};
    }
    input.value[0]=1;
    input.until("truncated left staging reply injected",[]{
        std::lock_guard<std::mutex> lock(sim_mutex); return !truncate_left_stage_ack;
    });
    input.dwell(4900ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(left_stage_writes==1 && drives[0].starts==0,
                "truncated staging reply repeated the write or moved the drive");
    }
    input.value[0]=0;
    input.dwell(600ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        corrupt_next_mode_read=true;
        truncate_left_stage_ack=true;
    }
    input.value[0]=1;
    input.until("left closes on ARM0", [&]{return input.position[0]==9000;});
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(mode_replies_corrupted==1 && left_stage_writes==2 && drives[0].starts==1,
                "corrupt preparation read did not recover with one start");
    }
    require(snapshot(1).starts==0,"left press sent to ARM1");
    input.value[0]=0;
    input.until("left release opens ARM0", [&]{return input.position[0]==0;});
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drop_right_stage_ack=true;
    }
    input.value[1]=1;
    input.until("right closes on ARM1", [&]{return input.position[1]==9000;});
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(right_stage_ack_drops==1 && right_stage_writes==1 && drives[1].starts==1,
                "missing staging ACK duplicated a write or start");
    }
    require(snapshot(0).starts==2,"right press changed ARM0");
    input.value[1]=0;
    input.until("right release opens ARM1", [&]{return input.position[1]==0;});

    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drives[0].hold_motion=true;
    }
    input.value[0]=1;
    input.until("left motion in progress", []{return snapshot(0).moving;});
    auto disconnected=Clock::now();
    input.connected=false;
    input.until("disconnect stops ARM0", []{auto d=snapshot(0);return !d.moving && d.stops==1;});
    std::cout << "simulated disconnect to stop acknowledgement: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-disconnected).count()
              << " ms" << std::endl;
    input.connected=true;
    input.dwell(1400ms);
    require(snapshot(0).starts==3 && snapshot(1).starts==2,"held input retriggered after reconnect");
    input.value[0]=0;
    input.dwell(600ms);
    input.value[1]=1;
    input.until("fresh gesture accepted on reconnect", [&]{
        return snapshot(1).starts==3 && input.position[1]==9000;
    });

    // Release the side grip specifically inside the last 100 ms clear-to-send
    // interval. No ambiguous write has happened at this point.
    input.value[1]=0;
    input.until("right opened for cancellation test", [&]{
        return snapshot(1).starts==4 && input.position[1]==0;
    });
    unsigned baseline;
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        baseline=trigger_settles[1];
    }
    input.value[1]=1;
    input.until("right prepared target, trigger pending", [&]{
        std::lock_guard<std::mutex> lock(sim_mutex);
        return trigger_settles[1]>baseline;
    });
    input.grip[1]=false;
    input.dwell(900ms);
    require(snapshot(1).starts==4,"stale right trigger moved gripper");
    require(snapshot(1).stops==0,"stopped an idle gripper after cancellation");
    input.value[1]=0;
    input.grip[1]=true;
    input.dwell(500ms);
    input.value[1]=1;
    input.until("adapter accepts fresh gesture after cancellation", [&]{return snapshot(1).starts==5;});
    input.until("right reaches closed position before feedback-loss test",[&]{return input.position[1]==9000;});
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drives[1].hold_motion=true;
        drop_motion_status=true;
        drop_stopped_status_reads=1;
    }
    input.value[1]=0;
    input.until("lost moving feedback sends stop",[]{return snapshot(1).stops==1;});
    input.value[1]=1;
    input.until("first recovery round missing a post-stop reply",[]{
        std::lock_guard<std::mutex> lock(sim_mutex); return drop_stopped_status_reads==0;
    });
    input.dwell(4500ms);
    require(snapshot(1).starts==6 && snapshot(1).stops==1,"repeated ambiguous motion or stop");
    {
        std::lock_guard<std::mutex> lock(sim_mutex); drives[1].hold_motion=false;
    }
    input.value[1]=0;
    input.dwell(550ms);
    input.value[1]=1;
    input.until("fresh press accepted after verified stop",[]{return snapshot(1).starts==7;});
    tool_enabled=false;
    input.value[1]=0;
    input.dwell(1800ms);
    require(snapshot(1).starts==7,"revoked Tool session triggered a new movement");
    input.grip[1]=false;
    input.dwell(450ms);
    input.grip[1]=true;
    input.dwell(550ms);
    tool_enabled=true;
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drives[1].hold_motion=true;
        drop_motion_status=true;
    }
    input.value[1]=1;
    input.until("new simulated motion before sustained loss",[]{return snapshot(1).moving;});
    const auto before_sustained_loss=snapshot(1);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drop_stopped_status_reads=30;
    }
    input.until("stop sent once on sustained loss",[&]{
        return snapshot(1).stops==before_sustained_loss.stops+1;
    });
    input.dwell(17000ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drop_stopped_status_reads=0;
    }
    input.value[1]=0;
    input.dwell(700ms);
    input.value[1]=1;
    input.dwell(1400ms);
    require(snapshot(1).starts==before_sustained_loss.starts &&
            snapshot(1).stops==before_sustained_loss.stops+1,
            "sustained loss unlocked the wrist or repeated its stop");
    adapter.stop();
}

void verify_missing_start_ack() {
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drives={};
        drives[0].hold_motion=true;
        lose_left_start_ack=true;
        lose_left_stop_ack=true;
        trace_left_stop=true;
    }
    auto owner=std::make_shared<rclcpp::Node>("ack_owner","/ack_test_"+std::to_string(getpid()));
    Input input(owner);
    input.value={0,0};
    Adapter adapter(*owner,true);
    input.until("both drives ready for lost start ACK", [&]{return input.position[0]==4200 && input.position[1]==4200;});
    input.dwell(350ms);
    input.value[0]=1;
    input.until("ambiguous start followed by one protective stop",[]{return snapshot(0).stops==1;});
    input.value[1]=1;
    input.until("right closes while left is suspended",[]{return snapshot(1).starts==1;});
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(left_checks.empty(),"first recovery check occurred too early");
    }
    input.dwell(4000ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(left_checks.size()==1 && left_checks.front()-left_stop_time>=2850ms &&
                left_checks.front()-left_stop_time<3600ms,"wrong first check schedule");
        trace_left_checks=false;
    }
    require(snapshot(0).starts==1 && !snapshot(0).moving,"held input repeated an ambiguous start");
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        lose_left_start_ack=true;
        lose_left_stop_ack=true;
        trace_left_stop=true;
        bad_recovery_modes=4;
    }
    input.value[0]=0;
    input.dwell(500ms);
    input.value[0]=1;
    input.until("fresh gesture accepted only after confirmed stop",[]{return snapshot(0).stops==2;});
    input.value[1]=0;
    input.until("peer opens during five-round recovery",[]{return snapshot(1).starts==2;});
    input.until("fifth left check reached",[&]{
        std::lock_guard<std::mutex> lock(sim_mutex); return left_checks.size()==5;
    },17s);
    input.dwell(1000ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        for (unsigned k=0;k<left_checks.size();++k) {
            const auto elapsed=left_checks[k]-left_stop_time;
            require(elapsed>=3s*(k+1)-150ms && elapsed<3s*(k+1)+600ms,"bad recovery round cadence");
        }
        require(drives[0].starts==2 && drives[0].stops==2,"held trigger replayed during recovery");
        trace_left_checks=false;
        lose_left_start_ack=true;
        trace_left_stop=true;
        bad_recovery_modes=5;
    }
    input.value[0]=0;
    input.dwell(600ms);
    input.value[0]=1;
    input.until("fifth healthy round allowed new input",[]{return snapshot(0).stops==3;});
    input.until("five consecutive bad rounds observed",[]{
        std::lock_guard<std::mutex> lock(sim_mutex);return left_checks.size()==5;
    },17s);
    input.dwell(3400ms);
    input.value={0,1};
    input.until("peer responds with left latched",[]{return snapshot(1).starts==3;});
    input.value[0]=1;
    input.dwell(1300ms);
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        require(left_checks.size()==5 && drives[0].starts==3 && drives[0].stops==3,
                "fifth failed check did not latch just the affected wrist");
    }
    adapter.stop();
}
}

void verify_trigger_without_position() {
    auto owner=std::make_shared<rclcpp::Node>("trigger_owner","/trigger_test_"+std::to_string(getpid()));
    Input input(owner);
    input.value={0,0};
    Adapter adapter(*owner,true);
    input.until("initial feedback before removing position telemetry",[&]{
        return input.position[0]==4200 && input.position[1]==4200;
    });
    {
        std::lock_guard<std::mutex> lock(sim_mutex);
        drop_position=true;
    }
    input.until("position telemetry absent on both wrists",[]{
        std::lock_guard<std::mutex> lock(sim_mutex);
        return missing_positions[0] && missing_positions[1];
    });
    for (unsigned edge=1; edge<=12; ++edge) {
        const bool close=(edge%2)==1;
        input.value.fill(close ? 1.0f : 0.0f);
        input.until("one command per wrist for this trigger edge",[&]{
            const auto l=snapshot(0),r=snapshot(1);
            const auto expected=close ? 9000 : 0;
            return l.starts==edge && r.starts==edge && l.position==expected && r.position==expected;
        });
    }
    input.dwell(1800ms);
    require(snapshot(0).starts==12 && snapshot(1).starts==12,"held trigger repeated a command");
    require(input.position[0]==4200 && input.position[1]==4200,"position telemetry unexpectedly delivered");
    std::cout<<"PASS six open-close cycles per hand without ordinary position feedback"<<std::endl;
    adapter.stop();
}

int main(int argc,char **argv) {
    auto env=[](const char *name){
        auto *value=std::getenv(name);
        return std::string(value ? value : "");
    };
    require(env("ROS_DOMAIN_ID")=="93" && env("ROS_LOCALHOST_ONLY")=="1","use isolated test domain");
    rclcpp::init(argc,argv);
    try {
        if (argc==2 && std::string(argv[1])=="--no-position") verify_trigger_without_position();
        else if (argc==2 && std::string(argv[1])=="--recovery") verify_missing_start_ack();
        else verify();
    }
    catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
