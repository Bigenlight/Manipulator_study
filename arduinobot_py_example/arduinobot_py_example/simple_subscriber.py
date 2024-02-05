import rclpy # 이 코드 상에서 로스2의 모든 기능을 사용하게 함
from rclpy.node import Node # 거기서 노드 클래스 가져오기
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self): 
        super().__init__("simple_subscriber") # 노드 생성
        self.pub_ = self.create_subscription(String, "chatter", self.msgCallback,10)
        

        
    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)

def main():
    rclpy.init() # 로스2와 연결
    simple_subscriber = SimpleSubscriber() # 우리가 만든 노드 클래스 객체선언
    rclpy.spin(simple_subscriber) # 노드가 계속 실행되게 만듬
    
    simple_subscriber.destroy_node() # 끝나면 노드 파괴
    rclpy.shutdown() # 로스2랑 연결해제

#  메인, 스크립트 실행시 실행 되는 것
if __name__ == '__main__':
    main()

# 이거 쓰고 setup.py로