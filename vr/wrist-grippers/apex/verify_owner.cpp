#include "L1Robot.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

int main(int argc,char **argv) {
    using namespace std::chrono_literals;
    const auto env=[](const char *key){const char *val=std::getenv(key);return std::string(val?val:"");};
    if (env("LUNA_APEX_WRIST_MODE")!="inspect" || env("ROS_DOMAIN_ID")!="93" ||
        env("ROS_LOCALHOST_ONLY")!="1") return 64;
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("inspect_apex_wrist_connection");
    constexpr int required_samples=30;
    std::array<int,2> samples{};
    bool stationary_and_healthy=true;
    using Msg=std_msgs::msg::Int32MultiArray;
    std::array<rclcpp::Subscription<Msg>::SharedPtr,2> sub;
    for (int i=0;i<2;i++) {
        sub[i]=node->create_subscription<Msg>(i?"info/wrist485_R":"info/wrist485_L",1,
            [&,i](Msg::ConstSharedPtr m){
                if(m->data.size()!=5) return;
                stationary_and_healthy &= m->data[1]==0 && (m->data[2]&4)!=0 &&
                    m->data[3]==0 && (m->data[4]&~0x1000)==0;
                if(samples[i]++==0) std::cout<<"ARM"<<i<<"/485A position="<<m->data[0]
                    <<" speed="<<m->data[1]<<" fault="<<m->data[3]<<" alarm="<<m->data[4]<<std::endl;
            });
    }
    if (FX_L1_System_Link(6,6,7,190,0)<0) return 1;
    const auto end=std::chrono::steady_clock::now()+100s;
    bool poll_isolated=true;
    do {
        for (auto arm:{FX_TERMINAL_ARM0,FX_TERMINAL_ARM1}) {
            FXChnType ch=FX_CHN_CANFD;
            unsigned char data[64]{};
            unsigned stamp=0;
            poll_isolated &= FX_L1_Terminal_GetData(arm,0,&ch,data,&stamp)==FUNC_RET_OPERATION_TIMEOUT;
        }
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(5ms);
    } while (stationary_and_healthy && (samples[0]<required_samples || samples[1]<required_samples)
             && std::chrono::steady_clock::now()<end);
    FX_L1_System_Unlink();
    rclcpp::shutdown();
    std::cout<<"Feedback samples L/R "<<samples[0]<<"/"<<samples[1]
             <<", factory CAN poll isolated="<<poll_isolated<<std::endl;
    return poll_isolated && stationary_and_healthy &&
        samples[0]>=required_samples && samples[1]>=required_samples ? 0 : 1;
}
