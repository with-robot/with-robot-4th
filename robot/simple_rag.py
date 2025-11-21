#!/usr/bin/env python3
"""
Simple RAG (Retrieval-Augmented Generation) for Robot Control
Uses keyword matching instead of vector embeddings for demo purposes
"""

from typing import List, Dict


class MotionPrimitive:
    """모션 프리미티브 데이터 클래스"""

    def __init__(self, name: str, description: str, code: str, keywords: List[str]):
        self.name = name
        self.description = description
        self.code = code
        self.keywords = keywords


class SimpleRAG:
    """간단한 키워드 기반 RAG 시스템"""

    def __init__(self):
        self.primitives = self._build_knowledge_base()

    def _build_knowledge_base(self) -> List[MotionPrimitive]:
        """지식 베이스 구축 (모션 프리미티브 저장소)"""
        return [
            MotionPrimitive(
                name="square_path",
                description="정사각형 경로로 이동",
                code="""print("Starting square path...")
size = 1.0
set_target_position(size, 0, 0, wait=True)
print("1/4 complete")
set_target_position(size, size, PI/2, wait=True)
print("2/4 complete")
set_target_position(0, size, PI, wait=True)
print("3/4 complete")
set_target_position(0, 0, -PI/2, wait=True)
print("Square complete!")""",
                keywords=["정사각형", "사각형", "square", "box", "rectangle"]
            ),

            MotionPrimitive(
                name="circle_path",
                description="원형 경로로 이동",
                code="""import math
print("Starting circular motion...")
radius = 0.5
steps = 16
for i in range(steps + 1):
    angle = 2 * PI * i / steps
    x = radius * (1 - math.cos(angle))
    y = radius * math.sin(angle)
    theta = angle
    set_target_position(x, y, theta, wait=True)
    if i % 4 == 0:
        print(f"Progress: {i*100//steps}%")
print("Circle complete!")""",
                keywords=["원", "원형", "circle", "circular", "round", "돌"]
            ),

            MotionPrimitive(
                name="triangle_path",
                description="삼각형 경로로 이동",
                code="""import math
print("Starting triangle path...")
size = 1.0
set_target_position(size, 0, 0, wait=True)
set_target_position(size/2, size*math.sqrt(3)/2, 2*PI/3, wait=True)
set_target_position(0, 0, -2*PI/3, wait=True)
print("Triangle complete!")""",
                keywords=["삼각형", "triangle", "세모"]
            ),

            MotionPrimitive(
                name="rotate_in_place",
                description="제자리에서 회전",
                code="""print("Rotating in place...")
rotations = 1
for i in range(rotations * 8):
    angle = 2 * PI * i / 8
    set_target_position(0, 0, angle, wait=True)
print("Rotation complete!")""",
                keywords=["회전", "돌", "rotate", "spin", "turn", "제자리"]
            ),

            MotionPrimitive(
                name="return_home",
                description="원점으로 복귀",
                code="""print("Returning to origin...")
set_target_position(0, 0, 0, wait=True)
print("Arrived at home position!")""",
                keywords=["원점", "복귀", "home", "return", "돌아"]
            ),
        ]

    def retrieve(self, query: str, top_k: int = 3) -> List[MotionPrimitive]:
        """
        쿼리와 관련된 모션 프리미티브 검색

        Args:
            query: 사용자 쿼리 (예: "정사각형으로 움직여줘")
            top_k: 반환할 최대 개수

        Returns:
            관련성 높은 모션 프리미티브 리스트
        """
        query_lower = query.lower()
        scores = []

        for primitive in self.primitives:
            score = 0
            # 키워드 매칭 점수 계산
            for keyword in primitive.keywords:
                if keyword.lower() in query_lower:
                    score += 10  # 키워드 일치

            # 설명 매칭
            if any(word in primitive.description for word in query_lower.split()):
                score += 5

            scores.append((score, primitive))

        # 점수 순으로 정렬
        scores.sort(reverse=True, key=lambda x: x[0])

        # top_k 반환 (점수가 0보다 큰 것만)
        return [prim for score, prim in scores[:top_k] if score > 0]

    def format_examples_for_prompt(self, query: str, top_k: int = 2) -> str:
        """
        프롬프트에 포함할 예제 문자열 생성

        Args:
            query: 사용자 쿼리
            top_k: 포함할 예제 개수

        Returns:
            포맷팅된 예제 문자열
        """
        relevant_primitives = self.retrieve(query, top_k)

        if not relevant_primitives:
            return "No specific examples found. Use your knowledge to generate code."

        examples = []
        for i, prim in enumerate(relevant_primitives, 1):
            examples.append(f"Example {i}: {prim.description}\n```python\n{prim.code}\n```")

        return "\n\n".join(examples)


# 테스트
if __name__ == "__main__":
    rag = SimpleRAG()

    test_queries = [
        "정사각형으로 움직여줘",
        "원형으로 돌아줘",
        "삼각형 그려줘",
        "제자리에서 회전",
        "집으로 돌아가"
    ]

    print("="*60)
    print("🔍 Simple RAG Test")
    print("="*60)

    for query in test_queries:
        print(f"\nQuery: {query}")
        print("-"*60)

        results = rag.retrieve(query, top_k=2)

        if results:
            print(f"Found {len(results)} relevant examples:")
            for i, prim in enumerate(results, 1):
                print(f"  {i}. {prim.name} - {prim.description}")
        else:
            print("  No matches found")

        print()

    # 프롬프트 포맷 테스트
    print("="*60)
    print("📝 Formatted Examples for Prompt")
    print("="*60)
    query = "정사각형으로 움직여줘"
    formatted = rag.format_examples_for_prompt(query)
    print(formatted)
