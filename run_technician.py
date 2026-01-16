#!/usr/bin/env python3
"""ROSA - Robot Operating System Agent for Ranger Garden Assistant"""

import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rosa import ROSA

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

    print("ROSA Agent initialized for ROS 2")
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
            print(f"\nROSA: {response}\n")
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except Exception as e:
            print(f"Error: {e}\n")

if __name__ == "__main__":
    main()
