#!/usr/bin/env python3
"""
Voice Control Pipeline for Humanoid Robots
This module implements a complete voice control pipeline for humanoid robots,
including speech recognition, natural language understanding, and command execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import AudioData
from builtin_interfaces.msg import Duration
import speech_recognition as sr
import threading
import queue
import numpy as np
import openai
import json
import re
from typing import Dict, Any, Optional
import time


class VoiceControlPipelineNode(Node):
    """
    Main voice control pipeline node that integrates speech recognition,
    natural language understanding, and command execution
    """
    def __init__(self):
        super().__init__('voice_control_pipeline_node')

        # Publishers
        self.speech_status_pub = self.create_publisher(String, 'speech_status', 10)
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.action_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.listening_status_pub = self.create_publisher(Bool, 'listening_status', 10)

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10
        )
        self.audio_sub = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10
        )

        # Initialize speech recognition components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        # Configuration parameters
        self.energy_threshold = 3000
        self.dynamic_energy_threshold = True
        self.pause_threshold = 0.8

        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold
        self.recognizer.pause_threshold = self.pause_threshold

        # Command mapping
        self.command_map = {
            'move forward': self.move_forward,
            'move backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'dance': self.perform_dance,
            'wave': self.perform_wave,
            'hello': self.greet_user,
            'hi': self.greet_user,
        }

        # Processing queues
        self.audio_queue = queue.Queue(maxsize=10)
        self.text_queue = queue.Queue(maxsize=5)

        # Threading for continuous listening
        self.listening_thread = None
        self.processing_thread = None
        self.stop_listening = threading.Event()

        # Voice activity detection
        self.vad_threshold = 0.01
        self.is_listening = False

        # Initialize processing threads
        self.start_listening()

        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.update_status)

        self.get_logger().info('Voice Control Pipeline Node initialized')

    def start_listening(self):
        """Start the continuous listening process"""
        if self.listening_thread is None or not self.listening_thread.is_alive():
            self.stop_listening.clear()
            self.listening_thread = threading.Thread(
                target=self.continuous_listening,
                daemon=True
            )
            self.listening_thread.start()

        # Update listening status
        status_msg = Bool()
        status_msg.data = True
        self.listening_status_pub.publish(status_msg)
        self.is_listening = True

    def continuous_listening(self):
        """Continuously listen for speech and process it"""
        self.get_logger().info('Starting continuous listening...')

        while not self.stop_listening.is_set():
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(
                        source,
                        timeout=1.0,
                        phrase_time_limit=10.0
                    )

                # Process the audio
                self.process_audio(audio)

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except sr.UnknownValueError:
                self.get_logger().info('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error in listening: {e}')

        self.get_logger().info('Stopped continuous listening')

    def process_audio(self, audio):
        """Process captured audio and recognize speech"""
        try:
            # Try Google Web Speech API (requires internet)
            try:
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Google STT: {text}')

                # Add to processing queue
                try:
                    self.text_queue.put(text, block=False)
                except queue.Full:
                    self.get_logger().warn('Text queue is full, dropping recognition result')

            except sr.RequestError:
                self.get_logger().warn('Google Web Speech API unavailable')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def audio_callback(self, msg):
        """Handle audio data from ROS audio topic (if available)"""
        # This would be used if audio is coming from a ROS audio topic
        # For now, we're using the system microphone directly
        pass

    def voice_command_callback(self, msg):
        """Process voice command using NLU and execute"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Processing voice command: {command}')

        # Try to execute command directly from command map
        executed = False
        for key, func in self.command_map.items():
            if key in command:
                func()
                executed = True
                break

        if not executed:
            # Use LLM for more complex command understanding
            self.process_with_llm(command)

        # Publish command status
        status_msg = String()
        status_msg.data = f'command_processed:{command}'
        self.speech_status_pub.publish(status_msg)

    def process_with_llm(self, command):
        """Process command using LLM for complex understanding"""
        # In a real implementation, you would call an LLM API
        # to interpret complex commands that aren't in the simple command map
        self.get_logger().info(f'Processing complex command with LLM: {command}')

        # For this example, we'll just log the command
        # In practice, you'd use OpenAI or other LLM APIs
        pass

    def update_status(self):
        """Update status periodically"""
        status_msg = String()
        status_msg.data = f"listening:{self.is_listening},queue_size:{self.text_queue.qsize()}"
        self.speech_status_pub.publish(status_msg)

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.3  # m/s
        cmd.angular.z = 0.0
        self.action_cmd_pub.publish(cmd)
        self.speak_response('Moving forward')

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.3  # m/s
        cmd.angular.z = 0.0
        self.action_cmd_pub.publish(cmd)
        self.speak_response('Moving backward')

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # rad/s
        self.action_cmd_pub.publish(cmd)
        self.speak_response('Turning left')

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.5  # rad/s
        self.action_cmd_pub.publish(cmd)
        self.speak_response('Turning right')

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.action_cmd_pub.publish(cmd)
        self.speak_response('Stopping')

    def perform_dance(self):
        """Perform a simple dance"""
        self.speak_response('Dancing for you!')
        # In a real implementation, this would control the robot's joints
        # to perform dance movements

    def perform_wave(self):
        """Perform waving gesture"""
        self.speak_response('Waving hello!')
        # In a real implementation, this would control the robot's arm

    def greet_user(self):
        """Greet the user"""
        self.speak_response('Hello! How can I help you today?')

    def speak_response(self, text):
        """Publish speech response (in practice, connect to TTS system)"""
        self.get_logger().info(f'Speech response: {text}')
        # In practice, this would go to a text-to-speech system


class NaturalLanguageUnderstanding:
    """
    Natural Language Understanding module for interpreting voice commands
    """
    def __init__(self):
        self.intent_patterns = {
            'move': [
                r'move\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d*\.?\d+)?\s*(?P<unit>meters?|cm|steps?)?',
                r'go\s+(?P<direction>forward|backward|left|right|up|down)',
                r'walk\s+(?P<direction>forward|backward|left|right|up|down)',
                r'come\s+here',
            ],
            'grasp': [
                r'pick\s+up\s+(?P<object>.+)',
                r'grab\s+(?P<object>.+)',
                r'take\s+(?P<object>.+)',
                r'hold\s+(?P<object>.+)',
            ],
            'follow': [
                r'follow\s+(?P<target>.+)',
                r'go\s+with\s+(?P<target>.+)',
                r'come\s+with\s+(?P<target>.+)',
            ],
            'stop': [
                r'stop',
                r'pause',
                r'wait',
                r'hold\s+on',
            ],
            'speak': [
                r'say\s+(?P<text>.+)',
                r'tell\s+(?P<text>.+)\s+to\s+(?P<recipient>.+)?',
                r'speak\s+(?P<text>.+)',
            ],
            'navigate': [
                r'go\s+to\s+(?P<location>.+)',
                r'navigate\s+to\s+(?P<location>.+)',
                r'bring\s+me\s+to\s+(?P<location>.+)',
                r'take\s+me\s+to\s+(?P<location>.+)',
            ]
        }

    def parse_command(self, text):
        """Parse natural language command into structured intent"""
        text_lower = text.lower().strip()

        for intent_type, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower, re.IGNORECASE)
                if match:
                    return {
                        'intent': intent_type,
                        'parameters': match.groupdict(),
                        'original_text': text
                    }

        # If no pattern matches, return unknown intent
        return {
            'intent': 'unknown',
            'parameters': {'text': text},
            'original_text': text
        }


class VoiceActivityDetector:
    """
    Voice Activity Detection module for identifying speech segments
    """
    def __init__(self, frame_duration=0.03, energy_threshold=0.01):
        self.frame_duration = frame_duration  # seconds
        self.energy_threshold = energy_threshold
        self.sample_rate = 16000  # Hz
        self.frame_size = int(self.frame_duration * self.sample_rate)

    def detect_voice_activity(self, audio_data):
        """Detect voice activity in audio data"""
        # Convert to numpy array if needed
        if not isinstance(audio_data, np.ndarray):
            audio_data = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # Process in frames
        speech_frames = []
        for i in range(0, len(audio_data), self.frame_size):
            frame = audio_data[i:i + self.frame_size]
            if len(frame) == self.frame_size:
                energy = np.mean(frame ** 2)
                is_speech = energy > self.energy_threshold
                speech_frames.append(is_speech)

        return any(speech_frames)  # True if any frame contains speech


class CommandExecutor:
    """
    Command execution module for executing parsed commands
    """
    def __init__(self, node):
        self.node = node

    def execute_command(self, parsed_command):
        """Execute a parsed command"""
        intent = parsed_command['intent']
        parameters = parsed_command['parameters']

        if intent == 'move':
            return self.execute_move_command(parameters)
        elif intent == 'grasp':
            return self.execute_grasp_command(parameters)
        elif intent == 'follow':
            return self.execute_follow_command(parameters)
        elif intent == 'stop':
            return self.execute_stop_command(parameters)
        elif intent == 'speak':
            return self.execute_speak_command(parameters)
        elif intent == 'navigate':
            return self.execute_navigate_command(parameters)
        else:
            return self.execute_unknown_command(parsed_command)

    def execute_move_command(self, params):
        """Execute move command"""
        direction = params.get('direction', 'forward')
        distance = params.get('distance')

        cmd = Twist()
        if direction == 'forward':
            cmd.linear.x = 0.3
        elif direction == 'backward':
            cmd.linear.x = -0.3
        elif direction == 'left':
            cmd.angular.z = 0.5
        elif direction == 'right':
            cmd.angular.z = -0.5

        self.node.action_cmd_pub.publish(cmd)
        return True

    def execute_stop_command(self, params):
        """Execute stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.node.action_cmd_pub.publish(cmd)
        return True

    def execute_unknown_command(self, parsed_command):
        """Handle unknown commands"""
        self.node.speak_response(f"I don't understand the command: {parsed_command['original_text']}")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlPipelineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice control pipeline node...')
        node.stop_listening.set()

        # Stop robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.action_cmd_pub.publish(cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()