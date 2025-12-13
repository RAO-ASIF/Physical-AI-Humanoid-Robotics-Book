#!/usr/bin/env python3

"""
Voice Command Processor for Humanoid Robots
This module processes voice commands and converts them to robot actions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import pyttsx3
import threading
import queue
import time
from typing import Dict, List


class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, 'speech_output', 10)

        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Initialize text-to-speech
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Speed of speech
        self.tts_engine.setProperty('volume', 0.9)  # Volume level

        # Command mappings
        self.command_map = {
            'forward': self.move_forward,
            'backward': self.move_backward,
            'go forward': self.move_forward,
            'go backward': self.move_backward,
            'move forward': self.move_forward,
            'move backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'spin left': self.spin_left,
            'spin right': self.spin_right,
            'hello': self.greet,
            'hi': self.greet,
            'how are you': self.how_are_you,
            'what is your name': self.tell_name,
        }

        # Voice processing variables
        self.voice_queue = queue.Queue()
        self.listening = True

        # Start voice processing thread
        self.voice_thread = threading.Thread(target=self.voice_processing_loop, daemon=True)
        self.voice_thread.start()

        self.get_logger().info('Voice Command Processor Node Initialized')

    def voice_command_callback(self, msg):
        """Handle voice command from audio processing"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received voice command: {command}')

        # Try to match command
        matched = False
        for key in self.command_map:
            if key in command:
                self.command_map[key]()
                self.acknowledge_command(key)
                matched = True
                break

        if not matched:
            self.acknowledge_command("unknown")

    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.3  # m/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -0.3  # m/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Moving backward')

    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.angular.z = 0.5  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Turning left')

    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.angular.z = -0.5  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Turning right')

    def spin_left(self):
        """Spin robot left"""
        twist = Twist()
        twist.angular.z = 1.0  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Spinning left')

    def spin_right(self):
        """Spin robot right"""
        twist = Twist()
        twist.angular.z = -1.0  # rad/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Spinning right')

    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Stopping robot')

    def greet(self):
        """Greet the user"""
        self.speak_text("Hello! I am your humanoid robot assistant.")

    def how_are_you(self):
        """Respond to how are you"""
        self.speak_text("I am doing well, thank you for asking!")

    def tell_name(self):
        """Tell the robot's name"""
        self.speak_text("I am a humanoid robot designed to assist you.")

    def acknowledge_command(self, command):
        """Acknowledge received command"""
        if command == "unknown":
            response = "Sorry, I didn't understand that command."
        else:
            response = f"Okay, executing {command} command."

        self.speak_text(response)

    def speak_text(self, text):
        """Speak text using text-to-speech"""
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()

            # Also publish to topic for other nodes
            msg = String()
            msg.data = text
            self.speech_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Text-to-speech error: {e}')

    def voice_processing_loop(self):
        """Main loop for voice processing"""
        with self.microphone as source:
            # Adjust for ambient noise
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info('Voice recognition system calibrated')

        while self.listening:
            try:
                with self.microphone as source:
                    self.get_logger().info('Listening for voice commands...')
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                # Process the audio in a separate thread to avoid blocking
                processing_thread = threading.Thread(
                    target=self.process_audio,
                    args=(audio,),
                    daemon=True
                )
                processing_thread.start()

            except sr.WaitTimeoutError:
                # This is expected when no speech is detected
                continue
            except Exception as e:
                self.get_logger().error(f'Voice recognition error: {e}')
                time.sleep(0.1)  # Brief pause before retrying

    def process_audio(self, audio):
        """Process audio data and convert to text"""
        try:
            # Recognize speech using Google's speech recognition
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Recognized: {text}')

            # Publish the recognized text
            msg = String()
            msg.data = text
            self.voice_command_sub.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().info('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition request error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.listening = False
        if hasattr(self, 'voice_thread') and self.voice_thread.is_alive():
            self.voice_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    """Main function to run the voice command processor"""
    rclpy.init(args=args)

    # Create the voice command processor node
    voice_node = VoiceCommandProcessor()

    try:
        # Keep the node running
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        voice_node.get_logger().info('Shutting down voice command processor...')
    finally:
        # Clean up
        voice_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()