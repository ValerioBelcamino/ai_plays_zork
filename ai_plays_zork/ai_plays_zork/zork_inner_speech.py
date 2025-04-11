import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate
import os


class ZorkReasonerNode(Node):
    def __init__(self):
        super().__init__('zork_reasoner')

        # Initialize the GroqLLM with your Groq API key and endpoint
        # self.groq_api_key = "your_groq_api_key"  # Replace with your actual API key
        # self.groq_api_url = "https://api.groq.com/v1/your-endpoint"  # Replace with the correct Groq API URL

        self.llm = init_chat_model("llama3-8b-8192", model_provider="groq")

        self.system_template = """Given the current state of the text-based game Zork, analyze the game description and provide the following reasoning: 
        - Current Environment: Describe the surroundings and setting of the character in this specific part of the game.
        - Key Objects and Interactions: Identify any notable objects, characters, or elements that are present in the environment. Explain how these might affect the player's decisions or actions.
        - Potential Actions: Based on the environment and objects, suggest a few possible actions the player could take next.
        - Game Progression: How does this current state relate to the broader narrative or the progression of the game?

        ### Context:
        - **Game Output**: The latest game description or terminal output received from Zork.
        - **Current Memory**: The accumulated history of important facts and events that have been stored over time.
        
        Your response should be structured, detailed, and demonstrate logical reasoning based on the game’s context. Please make sure to reason clearly, considering the game’s mechanics and storyline."""

        self.human_template = "Past Interactions:\n{past_interactions}\n\nCurrent Memory:\n{current_memory}"

        self.prompt_template = ChatPromptTemplate.from_messages(
            [("system", self.system_template), ("user", self.human_template)]
)

        # Subscriber to receive game output
        self.zork_output_subscriber = self.create_subscription(
            String,
            '/zork_output',
            self.game_output_callback,
            10
        )

        # Subscriber to receive game output
        self.zork_input_subscriber = self.create_subscription(
            String,
            '/zork_input',
            self.game_input_callback,
            10
        )

        self.reasoner_activation_subscriber = self.create_subscription(
            Bool,
            '/zork_reasoner_activation',
            self.reasoning_callback,
            10
        )

        # Publisher to send reasoned output
        self.publisher = self.create_publisher(String, '/zork_reasoner_output', 10)
        
        print("\033[95mTZorkReasonerNode is ready and listening on '/zork_output'\033[0m\n")
        self.package_path = "/home/belca/Desktop/AI_Plays_ZORK/src/ai_plays_zork/ai_plays_zork"

        self.memory = ""
        self.memory_file_path = os.path.join(self.package_path, 'memory.txt')

        self.last_game_state = ""
        self.last_game_input = ""

        self.last_prompts = {}


    def game_output_callback(self, msg):
        # self.get_logger().info(f"Received Zork game state:\n{self.last_game_state}")
        self.last_game_state = msg.data.strip()
        self.last_prompts[self.last_game_input] = self.last_game_state

    def game_input_callback(self, msg):
        # self.get_logger().info(f"Received Zork game state:\n{self.last_game_state}")
        self.last_game_input = msg.data.strip()


    def reasoning_callback(self, msg):
        # self.get_logger().info(f"Reasoning")

        # Reason about the game state using LangChain (Groq API)
        reasoner_output = self.reason_about_game_state()

        # Publish the reasoned output
        self.publish_reasoner_output(reasoner_output)


    def reason_about_game_state(self):
        # Load the memory from file
        with open(self.memory_file_path, 'r') as file:
            self.memory = file.readlines()

        print(f'\033[92m{str(self.last_prompts)}\033[0m')
        print(f'\033[93m{self.memory}\033[0m')
            
        # Use LangChain to call Groq API through the custom LLM
        input_dict = {
            "past_interactions": str(self.last_prompts),
            "current_memory": "\n".join(self.memory)
        }
        prompt = self.prompt_template.invoke(input_dict)
        result = self.llm.invoke(prompt)
        
        # self.get_logger().info(f"Reasoned output from LangChain/Groq: {result}")
        print(f"\033[34m{result.content}\033[0m\n")
        return result.content
    

    def publish_reasoner_output(self, reasoner_output):
        # Create a message with the reasoned output
        msg = String()
        msg.data = reasoner_output

        # Publish the message to 'zork_reasoned_output' topic
        self.publisher.publish(msg)
        # self.get_logger().info(f"Published reasoner output!")


def main(args=None):
    rclpy.init(args=args)
    node = ZorkReasonerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
