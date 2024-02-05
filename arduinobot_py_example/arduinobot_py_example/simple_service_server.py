import sys
import rclpy # 로스2랑 연결
from rclpy.node import Node # 노드 가져오기
# 우리가만든 메시지 인터페이스 가져오기
from arduinobot_msgs.srv import AddTwoInts


class SimpleServiceServer(Node):

    def __init__(self):
        # 노드 생성자, 노드명
        super().__init__("simple_service_server")
        # 노드 클래스에 멤버 변수 새로 생성, 인터페이스명, 서비스명, 메시지가(요청) 있을 때마다 발동되는 함수
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)
        # 이걸 위한 새로운 통신 인터페이스를 만들어야함
        # 통상적으로 다른 패키지에 그것을 만듬!
        
        self.get_logger().info("Service add_two_ints Ready") # 출력

    # 함수 정의, 요청 인풋은 req, 대답 아웃풋은 res
    def serviceCallback(self, req, res):
        # 요청 인풋
        self.get_logger().info("New Request Received a: %d, b: %d" % (req.a, req.b)) # req에서 a, b를 빼옴
        # 계산식
        res.sum = req.a + req.b # 으으음... 이렇게 res.sum에 바로 간단하게 집어넣수도 있구나, 나는 따로 res을 지정해야하는 줄...
        
        self.get_logger().info("Returning sum: %d" % res.sum) # 출력
        return res


def main():
    rclpy.init() # 로스 시작

    simple_service_server = SimpleServiceServer() # 객체 생성
    rclpy.spin(simple_service_server) # 노드 유지
    
    simple_service_server.destroy_node() # 노드 자폭
    rclpy.shutdown() # 로스 연결해제


if __name__ == '__main__':
    main()
    
    # 쓰고 setup.py로
