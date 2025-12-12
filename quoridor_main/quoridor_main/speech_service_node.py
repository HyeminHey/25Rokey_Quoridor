#!/usr/bin/env python3
# coding: utf-8

"""
쿼리도 음성 인식 서비스 노드
음성 인식 전용 (개선된 프롬프트)
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate

# 기존 voice_processing 모듈 임포트
from quoridor_main.voice_processing.stt import STT

# 환경 변수 로드
load_dotenv()
openai_api_key = os.getenv("OPENAI_API_KEY")


class SpeechServiceNode(Node):
    """음성 인식 + 명령 파싱을 제공하는 ROS2 서비스 노드"""
    
    def __init__(self):
        super().__init__('speech_service_node')
        
        # API 키 확인
        if not openai_api_key:
            self.get_logger().error("❌ OPENAI_API_KEY가 설정되지 않았습니다.")
            raise ValueError("OPENAI_API_KEY required in .env file")
        
        # 파라미터 선언
        self.declare_parameter('service_name', '/speech_to_text')
        self.declare_parameter('record_duration', 5.0)  # 녹음 시간
        
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.record_duration = self.get_parameter('record_duration').get_parameter_value().double_value
        
        # STT 초기화 (음성 인식 필수)
        try:
            self.stt = STT(openai_api_key=openai_api_key)
            self.get_logger().info("✅ OpenAI Whisper STT 초기화 완료")
            self.get_logger().info(f"   녹음 시간: {self.record_duration}초")
        except Exception as e:
            self.get_logger().error(f"❌ STT 초기화 실패: {e}")
            raise
        
        # LLM 초기화
        self.llm = ChatOpenAI(
            model="gpt-4o", 
            temperature=0.3,
            api_key=openai_api_key
        )
        
        # 프롬프트 템플릿 (더 강력한 버전)
        prompt_content = """
            당신은 쿼리도 보드게임 명령어 분류기입니다.

            사용자가 말한 내용을 보고, 다음 중 정확히 하나를 출력하세요:
            - start
            - end  
            - none

            분류 기준:
            - "start": 게임 시작 의도 (시작, 고, go, start, 레츠고, 가자, 게임 등)
            - "end": 턴 종료 의도 (끝, 다음, 넘겨, pass, done, end, 완료 등)
            - "none": 위 두 가지가 아닌 모든 경우

            예시:
            "시작" → start
            "게임 가자" → start
            "고고" → start
            "끝" → end
            "pass" → end
            "턴 넘겨" → end
            "날씨 좋네" → none

            입력: "{user_input}"
            답변 (start/end/none만):"""

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], 
            template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        
        # ROS2 서비스 생성
        self.srv = self.create_service(
            Trigger, 
            service_name, 
            self.speech_callback
        )
        
        self.get_logger().info(f"✅ 음성 인식 서비스 시작: {service_name}")
        self.get_logger().info(f"   - STT 모드: 🎤 Real Voice Only")
    
    def get_speech_text(self):
        """음성 입력 받기"""
        self.get_logger().info(f"🎤 {self.record_duration}초 동안 음성 입력 대기 중...")
        try:
            # 기존 STT 모듈의 speech2text() 사용
            text = self.stt.speech2text()
            return text
        except Exception as e:
            self.get_logger().error(f"STT 오류: {e}")
            return None
    
    def parse_command(self, text):
        """키워드 필터링 + LLM 파싱 (하이브리드 방식)"""
        if not text or text.strip() == "":
            return "none"
        
        text_lower = text.lower().strip()
        
        # 1단계: 명확한 키워드 매칭 (빠르고 확실)
        start_keywords = [
            # 한국어
            "시작", "스타트", "고", "가자", "가즈아",
            "고고", "레츠고", "렛츠고", "게임", "한판", 
            "켜", "준비", "하자", "한다",
            # 영어
            "start", "go", "let", "lets", "begin", 
            "ready", "play", "game"
        ]
        
        end_keywords = [
            # 한국어
            "끝", "엔드", "다음", "넥스트", "패스",
            "완료", "턴", "차례", "넘", "됐", "오케이",
            # 영어
            "end", "next", "pass", "done", "finish",
            "turn", "ok", "okay", "switch"
        ]
        
        # 시작 키워드 체크
        for keyword in start_keywords:
            if keyword in text_lower:
                self.get_logger().info(f"🎯 키워드 매칭: '{keyword}' → start")
                return "start"
        
        # 종료 키워드 체크
        for keyword in end_keywords:
            if keyword in text_lower:
                self.get_logger().info(f"🎯 키워드 매칭: '{keyword}' → end")
                return "end"
        
        # 2단계: 키워드 매칭 실패 시 LLM 사용
        self.get_logger().info(f"🤖 LLM 파싱 시도: '{text}'")
        try:
            llm_response = self.lang_chain.invoke({"user_input": text})
            parsed = llm_response.content.strip().lower()
            
            # 검증
            if parsed not in ["start", "end", "none"]:
                self.get_logger().warn(f"⚠️  예상치 못한 응답: '{parsed}' → 'none'으로 처리")
                return "none"
            
            self.get_logger().info(f"🤖 LLM 결과: '{parsed}'")
            return parsed
        except Exception as e:
            self.get_logger().error(f"LLM 파싱 오류: {e}")
            return "none"
    
    def speech_callback(self, request, response):
        """
        ROS2 서비스 콜백
        Trigger 서비스:
        - Request: 비어있음
        - Response: success (bool), message (string)
        
        message 형식:
        - "start game" : 게임 시작 명령
        - "end turn"   : 턴 종료 명령
        - ""           : 명령 아님
        """
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info("📞 음성 인식 서비스 호출됨")
        
        # 1. 음성 입력
        text = self.get_speech_text()
        
        if text is None or text.strip() == "":
            response.success = False
            response.message = ""
            self.get_logger().warn("❌ 음성 인식 실패 또는 비어있음")
            return response
        
        self.get_logger().info(f"📝 입력 텍스트: '{text}'")
        
        # 2. LLM 파싱
        command = self.parse_command(text)
        
        # 3. 결과 반환
        if command == "start":
            response.success = True
            response.message = "start game"
            self.get_logger().info("✅ 인식: 게임 시작 명령")
        elif command == "end":
            response.success = True
            response.message = "end turn"
            self.get_logger().info("✅ 인식: 턴 종료 명령")
        else:
            response.success = True
            response.message = ""  # 빈 문자열 = 명령 아님
            self.get_logger().info("ℹ️  인식: 게임 명령 아님")
        
        self.get_logger().info(f"🔄 응답: success={response.success}, message='{response.message}'")
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SpeechServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 종료합니다")
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()