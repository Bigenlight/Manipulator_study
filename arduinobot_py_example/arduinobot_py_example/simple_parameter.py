import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self): # 생성자
        super().__init__("simple_parameter") 
        # 노드 클래스에서 가져온 함수
        self.declare_parameter("simple_int_param", 28) #파라미터 지정 가능, 여기서는 그냥 28을 지정
        self.declare_parameter("simple_string", "Metheo")# 글자도 집어넣기 가능
        
        # 파미터들이 바뀔 때마다 발동
        self.add_on_set_parameters_callback(self.paramChangeCallback)
        
        # 파라미터가 변결 될 때마다 실행 되는 함수
    def paramChangeCallback(self, params):
        result = SetParametersResult() # 가져온 파라미터 결과 클래스
        
        # 파라미터가 바뀌면 터미널에 출력
        for param in params:
            if param.name == "simple_int_parma" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("simple_int_param 파라이미터가 바뀌었습니다: %d" % param.value)
                result.successful = True
            if param.name == "simple_string" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("simple_string 파라이미터가 바뀌었습니다: %d" % param.value)
                result.successful = True
                
        return result.successful


def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    
    simple_parameter.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()

#쓰고 setup으로