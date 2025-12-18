#!/usr/bin/env python3
# coding: utf-8

"""
Wakeup Word Node for Quoridor Game (Korean Support)
한국어 wakeup word 감지 → ROS2 토픽으로 신호 발행
Porcupine v4.0.0 대응 버전
"""

import os
import struct
import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import pyaudio
import pvporcupine
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import Trigger


# 환경 변수 로드
load_dotenv()
picovoice_api_key = os.getenv("PICOVOICE_ACCESS_KEY")


class WakeupWordNode(Node):
    """Wakeup Word 감지 노드 (한국어 지원, Porcupine v4.0.0)"""

    def __init__(self):
        super().__init__("wakeup_word_node")

        # Parameters
        self.declare_parameter("keyword_filename", "hello-query_ko_linux_v4_0_0.ppn")  # v4.0.0 파일명
        self.declare_parameter("model_filename", "porcupine_params_ko.pv")
        self.declare_parameter("sensitivity", 0.7)
        self.declare_parameter("audio_device_index", -1)
        self.declare_parameter("language", "ko")
        self.declare_parameter("wake_service", "/wakeup_robot")

        keyword_filename = self.get_parameter("keyword_filename").value
        model_filename = self.get_parameter("model_filename").value
        sensitivity = self.get_parameter("sensitivity").value
        audio_device_index = self.get_parameter("audio_device_index").value
        language = self.get_parameter("language").value
        self.wake_srv = self.get_parameter("wake_service").value

        # API Key 확인
        if not picovoice_api_key:
            self.get_logger().error("❌ PICOVOICE_ACCESS_KEY 환경변수 필요")
            self.get_logger().error("   ~/.bashrc에 추가:")
            self.get_logger().error("   export PICOVOICE_ACCESS_KEY='your-key'")
            raise ValueError("PICOVOICE_ACCESS_KEY not found")

        # 패키지 경로
        try:
            package_share = get_package_share_directory('quoridor_main')
            
            # 키워드 파일 경로
            keyword_path = os.path.join(package_share, 'keywords', keyword_filename)
            if not os.path.exists(keyword_path):
                self.get_logger().error(f"❌ 키워드 파일 없음: {keyword_path}")
                self.get_logger().error(f"   Porcupine Console에서 v4.0.0 키워드 파일 다운로드:")
                self.get_logger().error(f"   https://console.picovoice.ai/")
                raise FileNotFoundError(keyword_path)
            
            # 한국어 모델 파일 경로
            model_path = None
            if model_filename:
                model_path = os.path.join(package_share, 'models', model_filename)
                if not os.path.exists(model_path):
                    self.get_logger().error(f"❌ 모델 파일 없음: {model_path}")
                    self.get_logger().error(f"   한국어 모델 파일(.pv)을 다음 위치에 배치:")
                    self.get_logger().error(f"   {package_share}/models/")
                    raise FileNotFoundError(model_path)
                self.get_logger().info(f"✅ 모델 파일: {model_filename}")
            
            self.get_logger().info(f"✅ 키워드 파일: {keyword_filename}")
            
        except Exception as e:
            self.get_logger().error(f"❌ 패키지 경로 오류: {e}")
            raise

        self.group = ReentrantCallbackGroup()

        self.wake_service = self.create_service(
            Trigger,
            self.wake_srv,
            self.handle_wake_request,
            callback_group=self.group
        )

        self.get_logger().info(f"✅ Wakeup Service 서버 생성: {self.wake_srv}")

        # Porcupine v4.0.0 초기화
        try:
            porcupine_params = {
                'access_key': picovoice_api_key,
                'keyword_paths': [keyword_path],
                'sensitivities': [sensitivity]
            }
            
            # 한국어 모델 파일 추가 (중요!)
            if model_path:
                porcupine_params['model_path'] = model_path
            
            self.porcupine = pvporcupine.create(**porcupine_params)
            
            # Porcupine 버전 확인
            porcupine_version = pvporcupine.__version__ if hasattr(pvporcupine, '__version__') else "Unknown"
            
            self.get_logger().info(f"✅ Porcupine 초기화 완료 (v{porcupine_version})")
            self.get_logger().info(f"   언어: {language.upper()}")
            self.get_logger().info(f"   민감도: {sensitivity}")
            self.get_logger().info(f"   샘플레이트: {self.porcupine.sample_rate}Hz")
            self.get_logger().info(f"   프레임길이: {self.porcupine.frame_length}")
            if model_path:
                self.get_logger().info(f"   모델: {os.path.basename(model_path)}")
                
        except pvporcupine.PorcupineInvalidArgumentError as e:
            self.get_logger().error(f"❌ Porcupine 초기화 실패 (버전 불일치): {e}")
            self.get_logger().error("   해결 방법:")
            self.get_logger().error("   1. pip install --upgrade pvporcupine")
            self.get_logger().error("   2. Porcupine Console에서 v4.0.0 키워드 재다운로드")
            self.get_logger().error("   3. 키워드 파일명을 *_v4_0_0.ppn으로 변경")
            raise
        except Exception as e:
            self.get_logger().error(f"❌ Porcupine 초기화 실패: {e}")
            self.get_logger().error("   API 키와 .ppn/.pv 파일을 확인하세요")
            raise

        # 오디오 스트림 초기화
        self.audio = pyaudio.PyAudio()
        try:
            stream_params = {
                'rate': self.porcupine.sample_rate,
                'channels': 1,
                'format': pyaudio.paInt16,
                'input': True,
                'frames_per_buffer': self.porcupine.frame_length,
            }
            
            # 특정 장치 지정 (파라미터로 받은 경우)
            if audio_device_index >= 0:
                stream_params['input_device_index'] = audio_device_index
                self.get_logger().info(f"   지정된 오디오 장치: #{audio_device_index}")
            
            self.stream = self.audio.open(**stream_params)
            self.get_logger().info(f"✅ 오디오 스트림 시작")
            
        except Exception as e:
            self.get_logger().error(f"❌ 오디오 스트림 실패: {e}")
            self.get_logger().error("   다른 장치로 시도:")
            self.get_logger().error("   ros2 run quoridor_game wakeup_word_node_korean --ros-args -p audio_device_index:=0")
            raise

        # 상태 변수
        self.detection_count = 0
        self.frame_count = 0
        self.last_log_time = time.time()
        self.language = language
        self.listening = False

        self.timer = self.create_timer(0.01, self.audio_process_timer, callback_group=self.group)
        self.lock = threading.Lock()

        self.is_listening = False
        self.detected = False

        # 언어별 안내 메시지
        wakeup_phrase = "헤이 쿼리" if language == "ko" else "Hello Query"
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"🎤 Wakeup Word 노드 준비 완료 ({language.upper()})")
        self.get_logger().info(f"   '{wakeup_phrase}'를 명확하게 말하세요!")
        self.get_logger().info(f"   Test Orchestrator 연동 준비 완료")
        self.get_logger().info("="*60)

    def flush_audio(self):
        """Lock을 사용하여 안전하게 버퍼를 비움"""
        with self.lock:  # 락 획득
            try:
                num_frames = self.stream.get_read_available()
                if num_frames > 0:
                    self.stream.read(num_frames, exception_on_overflow=False)
            except Exception as e:
                self.get_logger().warn(f"Buffer flush error: {e}")
    def audio_process_timer(self):
    # 락을 한 번만 걸어서 전체 프로세스를 보호합니다.
        with self.lock:
            try:
                if not self.is_listening:
                    # 청취 중이 아닐 때는 버퍼를 비우기만 함
                    num_frames = self.stream.get_read_available()
                    if num_frames > 0:
                        self.stream.read(num_frames, exception_on_overflow=False)
                    return

                # 청취 중일 때: 프레임이 충분한지 확인
                if self.stream.get_read_available() < self.porcupine.frame_length:
                    return

                # 실제 데이터 읽기 및 처리
                pcm = self.stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)
                
                result = self.porcupine.process(pcm)
                if result >= 0:
                    self.get_logger().info("🔔 WAKEUP WORD DETECTED!")
                    self.detected = True
                    self.is_listening = False
                    
            except Exception as e:
                # 오디오 장치 일시적 오류는 로그만 남기고 무시
                # self.get_logger().debug(f"Audio processing skipped: {e}")
                pass

    def handle_wake_request(self, request, response):
        """서비스 콜백: 요청이 오면 감지될 때까지 대기"""
        self.get_logger().info("🔔 Wakeup 요청 수신 -> 감지 시작")
        
        # 이전 감지 기록 초기화 및 버퍼 비우기
        self.detected = False
        self.flush_audio() 
        self.is_listening = True

        # 감지될 때까지 루프 (MultiThreadedExecutor 덕분에 타이머 콜백은 계속 동작함)
        start_time = time.time()
        timeout = 30.0  # 필요시 타임아웃 설정
        
        while rclpy.ok() and self.is_listening:
            if self.detected:
                break
            if (time.time() - start_time) > timeout:
                self.is_listening = False
                response.success = False
                response.message = "timeout"
                return response
            time.sleep(0.01)

        response.success = True
        response.message = "awake"
        return response


    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        self.get_logger().info("🛑 Wakeup Word 노드 종료 중...")
        
        try:
            if hasattr(self, 'stream') and self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if hasattr(self, 'audio') and self.audio:
                self.audio.terminate()
            if hasattr(self, 'porcupine') and self.porcupine:
                self.porcupine.delete()
        except Exception as e:
            self.get_logger().warn(f"정리 중 오류: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = WakeupWordNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            node.destroy_node()
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()