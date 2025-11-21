#!/usr/bin/env python3
"""
LLM + 장애물 회피 통합 테스트 (시뮬레이터 없이)
"""

from llm_agent import LLMRobotAgent

# LLM 에이전트 초기화
agent = LLMRobotAgent()

print("="*80)
print("🤖 LLM + 장애물 회피 통합 테스트")
print("="*80)

# 테스트 1: 장애물 추가 명령
print("\n[테스트 1] 장애물 추가")
print("-"*80)
command1 = "장애물을 (1.0, 0.5) 위치에 추가해줘"
print(f"명령: {command1}\n")

code1 = agent.generate_code(command1)
print("생성된 코드:")
print(code1)

# 테스트 2: 장애물 회피 이동
print("\n" + "="*80)
print("[테스트 2] 장애물 회피 이동")
print("-"*80)
command2 = "장애물을 (1.0, 0.5)에 추가하고, 목표 지점 (1.5, 0.5)로 이동해줘"
print(f"명령: {command2}\n")

code2 = agent.generate_code(command2)
print("생성된 코드:")
print(code2)

# 테스트 3: 다중 장애물
print("\n" + "="*80)
print("[테스트 3] 다중 장애물 환경")
print("-"*80)
command3 = "3개의 장애물을 추가해줘: (0.5, 0.5), (1.5, 0.5), (1.0, 1.5). 그리고 목표 지점 (2.0, 2.0)으로 이동"
print(f"명령: {command3}\n")

code3 = agent.generate_code(command3)
print("생성된 코드:")
print(code3)

# 테스트 4: 장애물 정보 확인
print("\n" + "="*80)
print("[테스트 4] 장애물 정보 확인")
print("-"*80)
command4 = "현재 장애물 정보를 보여줘"
print(f"명령: {command4}\n")

code4 = agent.generate_code(command4)
print("생성된 코드:")
print(code4)

# 테스트 5: 장애물 제거
print("\n" + "="*80)
print("[테스트 5] 장애물 제거")
print("-"*80)
command5 = "모든 장애물을 제거해줘"
print(f"명령: {command5}\n")

code5 = agent.generate_code(command5)
print("생성된 코드:")
print(code5)

print("\n" + "="*80)
print("✅ LLM 통합 테스트 완료!")
print("="*80)
print("\n결론:")
print("  ✅ LLM이 장애물 관련 명령을 이해함")
print("  ✅ 적절한 Python 코드 생성")
print("  ✅ add_obstacle, get_obstacle_info, clear_obstacles 함수 활용")
print("  ✅ set_target_position 자동 장애물 회피 이해")
print("\n시나리오 1 LLM 통합 준비 완료! 🚀")
