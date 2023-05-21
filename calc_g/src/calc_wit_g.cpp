#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>

class CalcWitImuG : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    CalcWitImuG(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s node start!!", name.c_str());
        count_ = 0;
        g_sum_ = 0;
        g_avg_ = 0;
        wit_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data",10,
                                            std::bind(&CalcWitImuG::IMUCallback,this,std::placeholders::_1));
    }

private:
    void IMUCallback(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg){
        count_ ++ ;
        auto x = imu_msg->linear_acceleration.x;
        auto y = imu_msg->linear_acceleration.y;
        auto z = imu_msg->linear_acceleration.z;
        auto temp_g = sqrt(x*x + y*y + z*z);
        g_sum_ += temp_g;
        g_avg_ = g_sum_/count_;
        RCLCPP_INFO(this->get_logger(),"average g:%.6f",g_avg_);
    }
    uint32_t count_;
    double g_sum_;
    double g_avg_;

    // 声明节点
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr wit_imu_sub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个的节点*/
    auto node = std::make_shared<CalcWitImuG>("calc_wit_g");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}