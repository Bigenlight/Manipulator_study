#include <rclcpp/rclcpp.hpp> // 로스2 C++ 가져오기
#include <std_msgs/msg/string.hpp>

#include <chrono>
using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node // 클래스 생성, 상속
{
    public:
        SimplePublisher() : Node("simple_publisher"), counter_(0) // 생성자, 노드명, counter 변수는 0
        {
            pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback,this));

            RCLCPP_INFO(get_logger(), "퍼블리싱 시작: 1 Hz");
        }

        void timerCallback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello ROS2 - counter: " + std::to_string(counter_++);
            pub_-> publish(message);
        }

    private:
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;


};
    int main(int argc, char* argv[])
    {
        rclcpp::init(argc,argv);
        auto node = std::make_shared<SimplePublisher>();
        rclcpp::spin(node);

        rclcpp::shutdown();
        return 0;
    }
    // 이거 쓰고 CMake 수정