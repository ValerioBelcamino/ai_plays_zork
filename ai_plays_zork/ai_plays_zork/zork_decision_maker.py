import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate
import os


class ZorkReasonerNode(Node):
    def __init__(self):
        super().__init__('zork_decision_maker')

        # Initialize the GroqLLM with your Groq API key and endpoint
        # self.groq_api_key = "your_groq_api_key"  # Replace with your actual API key
        # self.groq_api_url = "https://api.groq.com/v1/your-endpoint"  # Replace with the correct Groq API URL

        self.llm = init_chat_model("llama-3.3-70b-versatile", model_provider="groq")

        self.system_template = """You are an expert player of the classic text-based adventure game Zork. Based on detailed reasoning from another agent, your task is to decide the single best next action for the player to take.

Your response should:
- Be a valid and concise game command (e.g., "open the mailbox", "go north", "take lantern").
- Reflect strategic thinking based on the current environment, key objects, and game progression.
- Avoid repeating reasoning or describing the environment—just choose the best action.
- Be grounded in the reasoning and context provided.

Only return the action as a plain string. Do not include explanations, formatting, or extra characters.
"""

        self.human_template = "Game Output:\n{game_output}\n\nReasoning Output:\n{reasoning_output}"

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

        self.reasoner_activation_subscriber = self.create_subscription(
            String,
            '/zork_reasoner_output',
            self.choose_action_callback,
            10
        )

        # Publisher to send reasoned output
        self.publisher = self.create_publisher(String, '/zork_input', 10)
        
        print("\033[95mTZorkReasonerNode is ready and listening on 'zork_output'\033[0m\n")

        self.last_game_state = ""


    def game_output_callback(self, msg):
        # self.get_logger().info(f"Received Zork game state:\n{self.last_game_state}")
        self.last_game_state = msg.data.strip()


    def choose_action_callback(self, msg):
        # self.get_logger().info(f"Let's choose an action")
        reasoning_output = msg.data.strip()


        # Reason about the game state using LangChain (Groq API)
        game_input = self.choose_an_action(reasoning_output)

        # Publish the reasoned output
        self.publish_game_input(game_input)


    def choose_an_action(self, reasoning_output):
        # Use LangChain to call Groq API through the custom LLM
        print(f'\033[92m{self.last_game_state}\033[0m')
        print(f'\033[93m{reasoning_output}\033[0m')
        input_dict = {
            "game_output": self.last_game_state,
            "reasoning_output": reasoning_output
        }
        prompt = self.prompt_template.invoke(input_dict)
        result = self.llm.invoke(prompt)
        
        # self.get_logger().info(f"Reasoned output from LangChain/Groq: {result}")
        print(f"\033[34m{result.content}\033[0m\n")
        return result.content
    

    def publish_game_input(self, game_input):
        # Create a message with the reasoned output
        msg = String()
        msg.data = game_input

        # Publish the message to 'zork_reasoned_output' topic
        self.publisher.publish(msg)
        # self.get_logger().info(f"Published reasoned output: {game_input}")


def main(args=None):
    rclpy.init(args=args)
    node = ZorkReasonerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
