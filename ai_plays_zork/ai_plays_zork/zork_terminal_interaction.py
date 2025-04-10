import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class ZorkPlayerNode(Node):
    def __init__(self):
        super().__init__('zork_player')
        self.subscription = self.create_subscription(
            String,
            'zork_input',
            self.command_callback,
            10
        )

        # Publisher to send game output
        self.publisher = self.create_publisher(String, '/zork_output', 10)

        self.get_logger().info("ZorkPlayerNode is ready and waiting for commands on 'zork_input'")
        # Print initial game state
        initial_output = self.read_screen_output()
        self.get_logger().info(f"Initial game state:\n{initial_output}")

    def command_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Received command: {command}")
        self.send_command_to_screen(command)
        time.sleep(0.1)
        output = self.read_screen_output()
        self.get_logger().info(f"Zork output:\n{output}")

    def send_command_to_screen(self, command):
        subprocess.run(['screen', '-S', 'zork_session', '-p', '0', '-X', 'stuff', f'{command}\n'])

    def publish_game_output(self, game_output):
        # Create a message with the game output
        msg = String()
        msg.data = game_output
        
        # Publish the message to 'zork_output' topic
        self.publisher.publish(msg)
        self.get_logger().info(f"Published game output: {game_output}")


    def read_screen_output(self):
        output = subprocess.check_output(['screen', '-S', 'zork_session', '-p', '0', '-X', 'hardcopy', '/tmp/zork_output.txt'])
        with open('/tmp/zork_output.txt', 'r') as file:
            output_text = file.read()

        if output_text[-2] == '>':
            # print('the last character is >')
            output_text = output_text[:-2]
        else:
            self.read_screen_output()
            return

        output_text = '\n'.join(output_text.split('>')[-1].split('\n')[1:]).strip()  # Remove the command prompt
        self.publish_game_output(output_text)
        return output_text

def main(args=None):
    rclpy.init(args=args)
    node = ZorkPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
