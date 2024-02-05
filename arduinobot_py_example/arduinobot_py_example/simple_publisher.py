import rclpy # 이 코드 상에서 로스2의 모든 기능을 사용하게 함
from rclpy.node import Node # 거기서 노드 클래스 가져오기
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self): 
        super().__init__("simple_publisher") # 노드 생성
        self.pub_ = self.create_publisher(String, "chatter", 10) # 퍼블리셔 기능 가져오기, "토픽명", 10은 데이터 크기
        self.counter_ = 0 # 보낼 토픽수
        self.frequency_ = 1.0 # 1초에 한번씩 전송
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)# 터미널에 출력, 이거는 한번만
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback) # (타이머 freq, 실행 함수)
        
    def timerCallback(self): # 1초마다 이 함수 실행
        msg = String() # 스트링 변수 생성
        msg.data = "Hello Ros2 - counter: %d" % self.counter_
        
        self.pub_.publish(msg) # 퍼블리시!
        
        self.counter_ += 1

def main():
    rclpy.init() # 로스2와 연결
    simple_publisher = SimplePublisher() # 우리가 만든 노드 클래스 객체선언
    rclpy.spin(simple_publisher) # 노드가 계속 실행되게 만듬
    
    simple_publisher.destroy_node() # 끝나면 노드 파괴
    rclpy.shutdown() # 로스2랑 연결해제

#  메인, 스크립트 실행시 실행 되는 것
if __name__ == '__main__':
    main()

# 이거 쓰고 setup.py로