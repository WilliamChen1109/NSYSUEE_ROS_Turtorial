import rclpy 

def main(args=None): 
	rclpy.init(args=args) # 初始化ROS 
	
	# 創建一個叫做hello_world_py的Node 
	node = rclpy.create_node('hello_world_py') 
	
	# 用Node的get_logger() function來print出Hello World! 
	node.get_logger().info('Hello World!') 
	
	# 指定rate為1Hz 
	rate = node.create_rate(1) 
	# 讓Node持續運行直到ROS關閉(Ctrl+C) 
	
	while rclpy.ok(): 
		node.get_logger().info('Hello World in Loop!') 
	
		rclpy.spin_once(node) # 讓Node執行一次，會等待1/rate秒
		 
	# 關閉ROS 
	rclpy.shutdown()

if __name__ == '__main__': 
	main()