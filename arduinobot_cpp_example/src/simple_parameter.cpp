#include <rclcpp/rclcpp.hpp> // 로스2 연결
#include <rcl_interfaces/msg/set_parameters_result.hpp> // 파라미터 설정 함수를 가져옴

#include <vector>
#include <string>
#include <memory>


using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node // 파리미터 클래스
{
public:
    SimpleParameter() : Node("simple_parameter") // 생성자
    {
        // 파라미터 2개 생성
        //정수형 파라미터
        declare_parameter<int>("simple_int_param", 28); // <파라미터 종류> , ()"파라미터 이름",파라미터 초기값)
        // 스트링형 파라미터
        declare_parameter<std::string>("simple_string_param", "Antonio");

        // 노드 실행중에 파람이 바뀔 수 있도록 함수 추가 => 콜백함수
        // 여기서는 std 클래스에서 가져온 bind 함수를 실행, &으로 paramChangeCallback을 SimpleParameter클래스로 선언
        // this로 현재 클래스라 지정, _1은 input 파라미터가 하나라는거
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

private:
    // 포인터 생성
    // 여기에 param_callback_handle_을 저장
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 따로 위해서 가져온 함수 이용해서 파라미터 바꿈
    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for(const auto& param : parameters)
        {
            // 파라미터가 바뀌면 터미널 출력
            // 파라미터가 제목이 같은건지, 파라미터가 같은 타입인 같은지
            if(param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param changed! New value is " << param.as_int());
                result.successful = true;
            }
            if(param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_string_param changed! New value is " << param.as_string());
                result.successful = true;
            }
        }
        
        return result;
    }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // 노드와 클래스 선언
  auto node = std::make_shared<SimpleParameter>();
  rclcpp::spin(node); // 노드 지속

  rclcpp::shutdown();
  return 0;
}
// 이거 하고 패키지