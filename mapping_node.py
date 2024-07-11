# This class subscribes to a topic called module_x/current_state and publish 
#the inverse message to a topic mapping_desired_state_inv(x)
# example module_0/current_state = [0,0,0] send to mapping_desired_state_1 = [40,40,40]

import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import re
import argparse

class MySubscriber(Node):
    def __init__(self, mapping_direction):

        # Subscribe to module_x/current_state topic with x the corresponding module number as user_command_pub
        super().__init__('my_subscriber')
        subscripiton_name = "module_" + str(mapping_direction) + "/current_state"
        self.subscription = self.create_subscription(String,subscripiton_name,  self.listener_callback,10)
        self.subscription  # Prevent unused variable warning

        #Publish to the second module 
        inverse_mapping_direction = int(not mapping_direction)
        #inverse_mapping_direction = int(mapping_direction)
        publisher_name = "mapping_desired_state_" + str(inverse_mapping_direction)
        self.publisher_ = self.create_publisher(String, publisher_name, 10)

        ## Example : Subscrible to module_0/current_state to get the current state of module 0 and publish the value
        ## of the module 0 current state to module_1 through the topic mapping_desired_state_1

    def listener_callback(self, msg):
        new_msg = String()
        desired_state = self.edit_data(msg)
        new_msg.data= 'Desired state mapping:%s' % desired_state
        self.publisher_.publish(new_msg)
        self.get_logger().info(new_msg.data)

    # Extract the positions from the message and inverse it : From msg = "Current state: [0.03, 15.94, 11.95]" to numbers = [-0.03, -15.94, -11.95]
        
    def edit_data(self, msg):
        pattern = r"[-+]?\d*\.\d+|[-+]?\d+"
        # Extract numerical values using regex
        numbers = re.findall(pattern, msg.data)
        
        desired_state = []

        limit_pos_sup = [40, 40,40]
        limit_pos_inf = [0, 0,0]

        
        for i in range(3):
            if float(numbers[i]) > limit_pos_sup[i] :
                desired_state.append(limit_pos_inf[i])
            elif  float(numbers[i]) < limit_pos_inf[i]:
                desired_state.append(limit_pos_sup[i])
            else :
                
                desired_state.append(abs(limit_pos_sup[i]-float(numbers[i])))

 

        return desired_state



def main(args=None):
    rclpy.init(args=args)

    # Take as argument an integer to choose on which topic he should subscribe to 
    parser = argparse.ArgumentParser(description='Haptic Joystick User Command')
    parser.add_argument('mapping_direction', type=int, help='')
    args = parser.parse_args(args)


    my_subscriber = MySubscriber(args.mapping_direction)

    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
