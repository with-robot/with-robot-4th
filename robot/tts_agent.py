#!/usr/bin/env python3
"""
Text-to-Speech Agent using ElevenLabs
Generates voice responses for the robot
"""

import os
from elevenlabs import ElevenLabs, VoiceSettings
from typing import Optional


class TTSAgent:
    """ElevenLabs TTS Agent for robot voice responses"""

    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize TTS agent

        Args:
            api_key: ElevenLabs API key (optional, reads from env if not provided)
        """
        self.api_key = api_key or os.getenv("ELEVENLABS_API_KEY", "")

        if not self.api_key:
            print("⚠️  WARNING: ELEVENLABS_API_KEY not set. TTS disabled.")
            print("   Set with: export ELEVENLABS_API_KEY='your-key'")
            self.enabled = False
            return

        try:
            self.client = ElevenLabs(api_key=self.api_key)
            self.enabled = True
            print("✅ ElevenLabs TTS initialized")
        except Exception as e:
            print(f"❌ ElevenLabs initialization error: {e}")
            self.enabled = False

        # Voice settings for robot-like voice
        self.voice_settings = VoiceSettings(
            stability=0.5,  # Medium stability for natural speech
            similarity_boost=0.75,
            style=0.5,
            use_speaker_boost=True
        )

    def generate_speech(
        self,
        text: str,
        voice_id: str = "21m00Tcm4TlvDq8ikWAM",  # Rachel (female, clear)
        output_path: str = "robot_speech.mp3"
    ) -> Optional[str]:
        """
        Generate speech from text

        Args:
            text: Text to convert to speech
            voice_id: ElevenLabs voice ID
                - "21m00Tcm4TlvDq8ikWAM" - Rachel (clear, professional)
                - "EXAVITQu4vr4xnSDxMaL" - Sarah (warm, friendly)
                - "ErXwobaYiN019PkySvjV" - Antoni (deep, authoritative)
            output_path: Where to save the audio file

        Returns:
            Path to generated audio file, or None if error
        """
        if not self.enabled:
            return None

        try:
            # Generate speech
            audio = self.client.generate(
                text=text,
                voice=voice_id,
                model="eleven_multilingual_v2",  # Supports Korean
                voice_settings=self.voice_settings
            )

            # Save to file
            with open(output_path, 'wb') as f:
                for chunk in audio:
                    f.write(chunk)

            print(f"✅ TTS generated: {output_path}")
            return output_path

        except Exception as e:
            print(f"❌ TTS generation error: {e}")
            return None

    def generate_response(self, user_command: str, action: str) -> str:
        """
        Generate natural language response for user command

        Args:
            user_command: Original user command
            action: Action being performed

        Returns:
            Response text
        """
        responses = {
            "정사각형": "정사각형 경로를 시작합니다.",
            "square": "Starting square pattern.",
            "원": "원형 경로로 이동합니다.",
            "circle": "Moving in circular pattern.",
            "삼각형": "삼각형을 그리겠습니다.",
            "triangle": "Drawing triangle shape.",
            "회전": "제자리에서 회전합니다.",
            "rotate": "Rotating in place.",
            "이동": "명령을 수행합니다.",
            "move": "Executing movement command.",
        }

        # Find matching response
        for keyword, response in responses.items():
            if keyword in user_command.lower():
                return response

        # Default response
        return "명령을 실행합니다."


# Test TTS agent
if __name__ == "__main__":
    import sys

    agent = TTSAgent()

    if not agent.enabled:
        print("\n⚠️  TTS disabled. Set ELEVENLABS_API_KEY to test.")
        sys.exit(1)

    # Test with sample text
    test_text = "안녕하세요, 저는 LLM 기반 로봇입니다. 정사각형 경로를 시작합니다."

    print(f"\nGenerating speech: {test_text}")
    audio_path = agent.generate_speech(test_text)

    if audio_path:
        print(f"\n✅ Success! Audio saved to: {audio_path}")
        print(f"   Play with: afplay {audio_path}")
    else:
        print("\n❌ Failed to generate speech")
