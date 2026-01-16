#!/usr/bin/env python3
"""Technician ROS - Robot Operating System Agent for Ranger Garden Assistant

Installation:
    git clone https://github.com/anh0001/ros-technician-cli
    cd ros-technician-cli
    pip install -e .
"""

import os
import sys
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI

try:
    from rosa import ROSA
except ImportError:
    print("Error: rosa module not found.")
    print("Please install it by running:")
    print("  git clone https://github.com/anh0001/ros-technician-cli")
    print("  cd ros-technician-cli")
    print("  pip install -e .")
    sys.exit(1)

# Load environment variables from .env
load_dotenv()

def main():
    # Create LLM using OpenAI
    llm = ChatOpenAI(
        model="gpt-4",
        api_key=os.getenv("OPENAI_API_KEY"),
        temperature=0
    )

    # Create ROSA agent for ROS 2
    agent = ROSA(ros_version=2, llm=llm)

    print("Technician ROS Agent initialized for ROS 2")
    print("Type your queries or 'quit' to exit.\n")

    while True:
        try:
            query = input("You: ").strip()
            if query.lower() in ['quit', 'exit', 'q']:
                print("Goodbye!")
                break
            if not query:
                continue

            response = agent.invoke(query)
            print(f"\nTechnician: {response}\n")
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except Exception as e:
            print(f"Error: {e}\n")

if __name__ == "__main__":
    main()
