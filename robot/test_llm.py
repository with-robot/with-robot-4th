#!/usr/bin/env python3
"""
Quick test script for LLM robot control
Tests code generation without running the actual robot
"""

import os
import sys
from llm_agent import LLMRobotAgent


def test_llm_agent():
    """Test LLM agent with various commands"""

    # Check API key
    if not os.getenv("OPENAI_API_KEY"):
        print("❌ Error: OPENAI_API_KEY not set!")
        print("\nPlease set your API key:")
        print("  export OPENAI_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'")
        return False

    print("="*60)
    print("🤖 LLM Robot Agent Test")
    print("="*60)

    agent = LLMRobotAgent()

    test_commands = [
        "정사각형으로 움직여줘",
        "원형으로 천천히 돌아줘",
        "삼각형을 그려줘",
        "Move in a square pattern with 2 meter sides",
        "제자리에서 3바퀴 회전"
    ]

    print("\n📝 Test Commands:")
    for i, cmd in enumerate(test_commands, 1):
        print(f"  {i}. {cmd}")

    print("\n" + "="*60)

    for i, command in enumerate(test_commands, 1):
        print(f"\n[Test {i}/{len(test_commands)}]")
        print(f"Command: {command}")
        print("-"*60)

        code = agent.generate_code(command)

        if code.startswith("# ERROR"):
            print(f"❌ Error: {code}")
            return False
        else:
            print("✅ Generated Code:")
            print(code)
            print("-"*60)

        # Reset for next test
        agent.reset_conversation()

    print("\n" + "="*60)
    print("✅ All tests passed!")
    print("="*60)
    return True


def interactive_test():
    """Interactive testing mode"""

    if not os.getenv("OPENAI_API_KEY"):
        print("❌ Error: OPENAI_API_KEY not set!")
        return

    print("="*60)
    print("🤖 Interactive LLM Robot Control")
    print("="*60)
    print("Enter commands (or 'quit' to exit):\n")

    agent = LLMRobotAgent()

    while True:
        try:
            command = input("Command: ").strip()

            if command.lower() in ['quit', 'exit', 'q']:
                print("👋 Goodbye!")
                break

            if not command:
                continue

            print("\nGenerating code...")
            code = agent.generate_code(command)

            if code.startswith("# ERROR"):
                print(f"❌ Error: {code}")
            else:
                print("\n✅ Generated Code:")
                print("-"*60)
                print(code)
                print("-"*60)

            print()

        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "-i":
        # Interactive mode
        interactive_test()
    else:
        # Batch test mode
        success = test_llm_agent()
        sys.exit(0 if success else 1)
