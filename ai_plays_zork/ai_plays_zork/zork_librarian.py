import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from langchain.chat_models import init_chat_model
from langchain_core.prompts import ChatPromptTemplate
import os

class ZorkLibrarianNode(Node):
    def __init__(self):
        super().__init__('zork_lbrarian')

        # Initialize the GroqLLM with your Groq API key and endpoint
        # self.groq_api_key = "your_groq_api_key"  # Replace with your actual API key
        # self.groq_api_url = "https://api.groq.com/v1/your-endpoint"  # Replace with the correct Groq API URL

        self.package_path = "/home/belca/Desktop/AI_Plays_ZORK/src/ai_plays_zork/ai_plays_zork"

        self.llm = init_chat_model("llama-3.3-70b-versatile", model_provider="groq")

        self.system_template = """You are the memory manager of an AI system collaboratively playing the text-based game Zork. Your task is to maintain a concise, useful, and up-to-date list of memories that track important facts about the world, including:

- Discovered **places**, their descriptions, and connections.
- Observed or obtained **objects** and their current known locations.
- Important **characters**, creatures, or entities encountered.
- Key **events**, outcomes, or player decisions that affect game progression.

### Input:
- `Game Output`: This is the game's latest textual description of the world.
- `Player Action`: This is the most recent action taken by the player.
- `Current Memory`: This is a list of previously stored memory items (natural language sentences).

### Your Tasks:
1. **Extract** new relevant information from the game output and player action. Express each item as a clear, brief sentence.
2. **Add** any new useful memory items that are not already in memory.
3. **Remove** outdated, contradicted, or redundant memories.
4. **Summarize or rewrite** older memory items if they have become too verbose or if multiple entries can be consolidated.
5. **Preserve** important long-term facts that are still valid.

### Output:
Return ONLY the updated memory list as a list of natural language sentences. Keep it concise but informative and avoid any other texts."""

        self.human_template = "Game Output:\n{game_output}\n\nPlayer Action:\n{player_action}\n\nCurrent Memory:\n{current_memory}"

        self.prompt_template = ChatPromptTemplate.from_messages(
            [("system", self.system_template), ("user", self.human_template)]
        )

        # Subscriber to receive game output
        self.subscription = self.create_subscription(
            String,
            'zork_output',
            self.update_memory_callback,
            10
        )

        # Subscriber to receive game input
        self.subscription = self.create_subscription(
            String,
            'zork_input',
            self.update_memory_callback,
            10
        )

        # Publisher to send reasoned output
        self.publisher = self.create_publisher(Bool, '/zork_reasoner_activation', 10)

        # Publisher to send reasoned output
        
        self.get_logger().info("ZorkLibrarianNode is ready and listening on 'zork_output'")
        self.get_logger().info("ZorkLibrarianNode is ready and listening on 'zork_input'")

        self.memory_file_path = os.path.join(self.package_path, 'memory.txt')
        with open(self.memory_file_path, 'r') as file:
            self.memory = file.readlines()
            self.get_logger().info(f"Loaded memory")

        self.last_decision = ''


    def update_memory_callback(self, msg):
        game_state = msg.data.strip()
        self.get_logger().info(f"Received:\n{game_state}")
        self.get_logger().info(f"last decision:\n{self.last_decision}")

        input_dict = {
            "game_output": game_state,
            "player_action": self.last_decision,
            "current_memory": "\n".join(self.memory)
        }

        prompt = self.prompt_template.invoke(input_dict)
        result = self.llm.invoke(prompt)
        updated = result.content.strip().splitlines()

        # Remove list markers like "- "
        updated_memory = [line.lstrip("- ").lstrip("* ").strip() for line in updated if line.strip()]
        self.memory = updated_memory

        # Pretty print updated memory in blue
        print("\033[94m" + "\n[🧠 Updated Memory]\n" + "\n".join(self.memory) + "\033[0m")

        with open(self.memory_file_path, 'w') as file:
            for line in self.memory:
                file.write(line + '\n')

        # Publishing updated memory for the reasoner to process
        msg_to_reasoner = Bool()
        self.get_logger().info(f"Activating Reasoner: {msg_to_reasoner}")

    def destroy_node(self):
        # Save memory to file
        with open(self.memory_file_path, 'w') as file:
            for line in self.memory:
                file.write(line + '\n')
        self.get_logger().info("Memory saved to memory.txt")
        super().destroy_node()

        

def main(args=None):
    rclpy.init(args=args)
    node = ZorkLibrarianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
