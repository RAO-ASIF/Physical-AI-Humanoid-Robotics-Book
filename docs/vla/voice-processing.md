---
title: Voice Processing for VLA Systems
sidebar_position: 2
---

# Voice Processing for VLA Systems

Voice processing forms the language component of Vision-Language-Action systems, enabling humanoid robots to understand and respond to spoken commands. This section covers the implementation of robust voice processing systems for humanoid robotics applications.

## Overview of Voice Processing in Robotics

Voice processing in humanoid robots involves converting spoken language to actionable commands through several stages: audio capture, speech recognition, natural language understanding, and command execution. The challenge lies in processing speech in real-world environments with background noise, multiple speakers, and varying acoustic conditions.

### Key Requirements for Robotic Voice Processing

#### 1. Real-Time Performance
- Low latency processing for natural interaction
- Continuous listening capabilities
- Efficient resource usage on embedded systems

#### 2. Robustness
- Noise cancellation for real-world environments
- Speaker diarization for multi-person interactions
- Handling of accented speech and speech variations

#### 3. Context Awareness
- Understanding commands in environmental context
- Maintaining conversation history
- Handling ambiguous or incomplete commands

## Speech Recognition Systems

### On-Device vs Cloud-Based Recognition

#### On-Device Recognition
Benefits:
- No internet dependency
- Lower latency
- Privacy preservation
- Offline capability

Challenges:
- Limited computational resources
- Smaller model sizes
- Reduced accuracy compared to cloud systems

#### Cloud-Based Recognition
Benefits:
- Higher accuracy with large models
- Continuous updates
- Multiple language support
- Advanced features (punctuation, speaker labels)

Challenges:
- Internet dependency
- Higher latency
- Privacy concerns
- Bandwidth requirements

### Implementation with Speech Recognition Libraries

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue
import numpy as np

class VoiceProcessingNode(Node):
    def __init__(self):
        super().__init__('voice_processing_node')

        # Publishers
        self.transcript_pub = self.create_publisher(String, 'speech_transcript', 10)
        self.command_pub = self.create_publisher(String, 'voice_command', 10)
        self.listening_status_pub = self.create_publisher(Bool, 'listening_status', 10)

        # Audio input setup
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        # Configuration parameters
        self.energy_threshold = self.get_parameter_or_set(
            'energy_threshold', 3000
        ).value
        self.dynamic_energy_threshold = self.get_parameter_or_set(
            'dynamic_energy_threshold', True
        ).value
        self.pause_threshold = self.get_parameter_or_set(
            'pause_threshold', 0.8
        ).value

        # Recognition settings
        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold
        self.recognizer.pause_threshold = self.pause_threshold

        # Threading for continuous listening
        self.listening_thread = None
        self.stop_listening = threading.Event()
        self.audio_queue = queue.Queue()

        # Voice activity detection
        self.vad_enabled = True
        self.silence_threshold = 0.01
        self.silence_duration = 1.0  # seconds

        # Timer for starting listening
        self.start_timer = self.create_timer(1.0, self.start_listening)

        self.get_logger().info('Voice Processing Node initialized')

    def get_parameter_or_set(self, name, default_value):
        """Get parameter or set default if not exists"""
        self.declare_parameter(name, default_value)
        return self.get_parameter(name)

    def start_listening(self):
        """Start the continuous listening process"""
        self.start_timer.cancel()  # Cancel the startup timer

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
            # Try multiple recognition services
            text = None

            # Option 1: Google Web Speech API (requires internet)
            try:
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Google STT: {text}')
            except sr.RequestError:
                self.get_logger().warn('Google Web Speech API unavailable')

            # Option 2: Sphinx (offline recognition)
            if text is None:
                try:
                    text = self.recognizer.recognize_sphinx(audio)
                    self.get_logger().info(f'Sphinx STT: {text}')
                except sr.RequestError:
                    self.get_logger().warn('Sphinx recognition unavailable')

            # Option 3: Wit.ai (if configured)
            if text is None:
                try:
                    wit_api_key = "YOUR_WIT_AI_KEY"  # In practice, use environment variables
                    text = self.recognizer.recognize_wit(audio, key=wit_api_key)
                    self.get_logger().info(f'Wit.ai STT: {text}')
                except sr.RequestError:
                    self.get_logger().warn('Wit.ai recognition unavailable')

            if text:
                # Publish transcript
                transcript_msg = String()
                transcript_msg.data = text
                self.transcript_pub.publish(transcript_msg)

                # Process as command
                self.process_command(text)
            else:
                self.get_logger().info('No speech recognized by any service')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def process_command(self, text):
        """Process recognized speech as a command"""
        # Normalize the text
        command_text = text.lower().strip()

        self.get_logger().info(f'Processing command: {command_text}')

        # Publish the command for other nodes to handle
        command_msg = String()
        command_msg.data = command_text
        self.command_pub.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice processing node...')
        node.stop_listening.set()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Natural Language Understanding

### Intent Recognition

Intent recognition involves identifying the user's goal from spoken language:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re
import json
from enum import Enum

class IntentType(Enum):
    MOVE = "move"
    GRASP = "grasp"
    FOLLOW = "follow"
    STOP = "stop"
    SPEAK = "speak"
    NAVIGATE = "navigate"
    UNKNOWN = "unknown"

class NLUProcessorNode(Node):
    def __init__(self):
        super().__init__('nlu_processor_node')

        # Subscribers
        self.speech_sub = self.create_subscription(
            String, 'speech_transcript', self.speech_callback, 10
        )

        # Publishers
        self.intent_pub = self.create_publisher(String, 'recognized_intent', 10)
        self.command_pub = self.create_publisher(String, 'parsed_command', 10)

        # Intent patterns
        self.intent_patterns = {
            IntentType.MOVE: [
                r'move\s+(?P<direction>forward|backward|left|right|up|down)\s*(?P<distance>\d*\.?\d+)?\s*(?P<unit>meters?|cm|steps?)?',
                r'go\s+(?P<direction>forward|backward|left|right|up|down)',
                r'walk\s+(?P<direction>forward|backward|left|right|up|down)',
                r'come\s+here',
            ],
            IntentType.GRASP: [
                r'pick\s+up\s+(?P<object>.+)',
                r'grab\s+(?P<object>.+)',
                r'take\s+(?P<object>.+)',
                r'hold\s+(?P<object>.+)',
            ],
            IntentType.FOLLOW: [
                r'follow\s+(?P<target>.+)',
                r'go\s+with\s+(?P<target>.+)',
                r'come\s+with\s+(?P<target>.+)',
            ],
            IntentType.STOP: [
                r'stop',
                r'pause',
                r'wait',
                r'hold\s+on',
            ],
            IntentType.SPEAK: [
                r'say\s+(?P<text>.+)',
                r'tell\s+(?P<text>.+)\s+to\s+(?P<recipient>.+)?',
                r'speak\s+(?P<text>.+)',
            ],
            IntentType.NAVIGATE: [
                r'go\s+to\s+(?P<location>.+)',
                r'navigate\s+to\s+(?P<location>.+)',
                r'bring\s+me\s+to\s+(?P<location>.+)',
                r'take\s+me\s+to\s+(?P<location>.+)',
            ]
        }

        # Parameter for confidence threshold
        self.confidence_threshold = self.get_parameter_or_set(
            'confidence_threshold', 0.7
        ).value

        self.get_logger().info('NLU Processor Node initialized')

    def get_parameter_or_set(self, name, default_value):
        """Get parameter or set default if not exists"""
        self.declare_parameter(name, default_value)
        return self.get_parameter(name)

    def speech_callback(self, msg):
        """Process incoming speech transcript"""
        transcript = msg.data.lower().strip()
        self.get_logger().info(f'Processing transcript: {transcript}')

        # Recognize intent
        recognized_intent = self.recognize_intent(transcript)

        if recognized_intent:
            # Publish recognized intent
            intent_msg = String()
            intent_msg.data = json.dumps(recognized_intent)
            self.intent_pub.publish(intent_msg)

            # Publish parsed command
            command_msg = String()
            command_msg.data = f"{recognized_intent['intent']}: {recognized_intent.get('parameters', {})}"
            self.command_pub.publish(command_msg)

            self.get_logger().info(f'Recognized intent: {recognized_intent}')
        else:
            self.get_logger().info('No intent recognized')

    def recognize_intent(self, text):
        """Recognize intent from text using pattern matching"""
        best_match = None
        best_confidence = 0.0

        for intent_type, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text, re.IGNORECASE)
                if match:
                    # Calculate confidence based on match completeness
                    confidence = min(len(match.group(0)) / len(text), 1.0)

                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_match = {
                            'intent': intent_type.value,
                            'confidence': confidence,
                            'parameters': match.groupdict(),
                            'original_text': text
                        }

        # Return if confidence is above threshold
        if best_match and best_match['confidence'] >= self.confidence_threshold:
            return best_match

        return None

def main(args=None):
    rclpy.init(args=args)
    node = NLUProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NLU processor node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Activity Detection

### Implementation for Robust Voice Processing

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
import numpy as np
import threading
import collections

class VoiceActivityDetectorNode(Node):
    def __init__(self):
        super().__init__('vad_node')

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10
        )

        # Publishers
        self.vad_status_pub = self.create_publisher(Bool, 'vad_status', 10)

        # VAD parameters
        self.frame_duration = 0.03  # 30ms frames
        self.silence_threshold = 0.01  # Adjust based on your mic sensitivity
        self.speech_threshold = 0.05   # Minimum energy for speech
        self.min_speech_frames = 5     # Minimum consecutive frames for speech
        self.min_silence_frames = 10   # Minimum consecutive frames for silence

        # Audio processing
        self.sample_rate = 16000  # Hz
        self.frame_size = int(self.frame_duration * self.sample_rate)
        self.audio_buffer = collections.deque(maxlen=100)  # Store recent audio

        # VAD state
        self.is_speech_detected = False
        self.speech_frame_count = 0
        self.silence_frame_count = 0

        # Lock for thread safety
        self.vad_lock = threading.Lock()

        self.get_logger().info('Voice Activity Detection Node initialized')

    def audio_callback(self, msg):
        """Process incoming audio data for voice activity detection"""
        # Convert audio data to numpy array
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Process audio in frames
        for i in range(0, len(audio_data), self.frame_size):
            frame = audio_data[i:i + self.frame_size]
            if len(frame) == self.frame_size:
                self.process_audio_frame(frame)

    def process_audio_frame(self, frame):
        """Process a single audio frame for VAD"""
        with self.vad_lock:
            # Calculate frame energy
            energy = np.mean(frame ** 2)

            if energy > self.speech_threshold:
                # Speech detected
                self.speech_frame_count += 1
                self.silence_frame_count = 0  # Reset silence counter

                if (not self.is_speech_detected and
                    self.speech_frame_count >= self.min_speech_frames):
                    self.is_speech_detected = True
                    self.publish_vad_status(True)
            else:
                # Silence detected
                self.silence_frame_count += 1
                self.speech_frame_count = 0  # Reset speech counter

                if (self.is_speech_detected and
                    self.silence_frame_count >= self.min_silence_frames):
                    self.is_speech_detected = False
                    self.publish_vad_status(False)

    def publish_vad_status(self, is_speech):
        """Publish voice activity status"""
        status_msg = Bool()
        status_msg.data = is_speech
        self.vad_status_pub.publish(status_msg)

        self.get_logger().info(f'Voice activity: {is_speech}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceActivityDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VAD node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Large Language Models

### Using OpenAI API for Advanced Understanding

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai
import json
import asyncio
from typing import Dict, Any

class LLMVoiceProcessorNode(Node):
    def __init__(self):
        super().__init__('llm_voice_processor_node')

        # Subscribers
        self.speech_sub = self.create_subscription(
            String, 'speech_transcript', self.speech_callback, 10
        )

        # Publishers
        self.llm_response_pub = self.create_publisher(String, 'llm_response', 10)
        self.robot_command_pub = self.create_publisher(String, 'robot_command', 10)

        # OpenAI API configuration
        # In practice, set this via environment variable
        openai.api_key = "YOUR_OPENAI_API_KEY"

        # Context management
        self.conversation_history = []
        self.max_history_length = 10

        # Robot capabilities for LLM context
        self.robot_capabilities = {
            "movement": ["move forward", "move backward", "turn left", "turn right", "stop"],
            "manipulation": ["pick up object", "place object", "open gripper", "close gripper"],
            "navigation": ["go to location", "follow person", "avoid obstacles"],
            "communication": ["speak text", "listen", "respond to questions"]
        }

        self.get_logger().info('LLM Voice Processor Node initialized')

    def speech_callback(self, msg):
        """Process speech through LLM for advanced understanding"""
        user_input = msg.data
        self.get_logger().info(f'Processing with LLM: {user_input}')

        try:
            # Prepare context for LLM
            context = self.prepare_context(user_input)

            # Call OpenAI API
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": f"You are a helpful assistant for a humanoid robot. The robot has these capabilities: {self.robot_capabilities}. "
                                  "Respond with a JSON object containing 'action' and 'parameters'. "
                                  "If the request is unclear, ask for clarification."
                    },
                    *self.conversation_history,
                    {"role": "user", "content": context}
                ],
                max_tokens=150,
                temperature=0.3
            )

            # Parse LLM response
            llm_response = response.choices[0].message['content'].strip()

            # Extract JSON from response
            import re
            json_match = re.search(r'\{.*\}', llm_response, re.DOTALL)
            if json_match:
                action_data = json.loads(json_match.group())

                # Publish LLM response
                response_msg = String()
                response_msg.data = json.dumps(action_data)
                self.llm_response_pub.publish(response_msg)

                # Publish robot command
                command_msg = String()
                command_msg.data = f"{action_data.get('action', 'unknown')} {action_data.get('parameters', {})}"
                self.robot_command_pub.publish(command_msg)

                self.get_logger().info(f'LLM parsed action: {action_data}')
            else:
                self.get_logger().warn(f'Could not extract JSON from LLM response: {llm_response}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in LLM response')
        except Exception as e:
            self.get_logger().error(f'LLM processing error: {e}')

    def prepare_context(self, user_input):
        """Prepare context for LLM including environment information"""
        # In a real implementation, this would include:
        # - Current robot state
        # - Environmental context from vision system
        # - Previous conversation history
        # - Available actions

        context = f"User says: '{user_input}'. "
        context += f"Robot capabilities: {self.robot_capabilities}. "
        context += "What should the robot do?"

        return context

def main(args=None):
    rclpy.init(args=args)
    node = LLMVoiceProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM voice processor node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Noise Reduction and Audio Enhancement

### Implementation for Clear Speech Recognition

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import numpy as np
import scipy.signal as signal
from scipy.fft import fft, ifft

class AudioEnhancementNode(Node):
    def __init__(self):
        super().__init__('audio_enhancement_node')

        # Subscribers
        self.raw_audio_sub = self.create_subscription(
            AudioData, 'raw_audio', self.raw_audio_callback, 10
        )

        # Publishers
        self.enhanced_audio_pub = self.create_publisher(AudioData, 'enhanced_audio', 10)
        self.noise_status_pub = self.create_publisher(String, 'noise_status', 10)

        # Audio parameters
        self.sample_rate = 16000  # Hz
        self.frame_size = 1024
        self.overlap = 0.75  # 75% overlap
        self.hop_size = int(self.frame_size * (1 - self.overlap))

        # Noise reduction parameters
        self.noise_threshold = 0.01
        self.enhancement_factor = 1.5

        # Initialize noise profile
        self.noise_profile = None
        self.noise_profile_samples = 0
        self.max_noise_samples = 100  # Number of samples to build noise profile

        # For spectral subtraction
        self.previous_spectrum = None

        self.get_logger().info('Audio Enhancement Node initialized')

    def raw_audio_callback(self, msg):
        """Process raw audio for noise reduction"""
        # Convert to numpy array
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Apply noise reduction
        enhanced_audio = self.reduce_noise(audio_data)

        # Convert back to AudioData message
        enhanced_msg = AudioData()
        enhanced_msg.data = (enhanced_audio * 32768.0).astype(np.int16).tobytes()

        self.enhanced_audio_pub.publish(enhanced_msg)

    def reduce_noise(self, audio_data):
        """Apply noise reduction to audio data"""
        if len(audio_data) < self.frame_size:
            return audio_data

        # Build noise profile from initial quiet periods
        if self.noise_profile is None:
            self.update_noise_profile(audio_data)
            return audio_data  # Return original for initial samples

        # Apply spectral subtraction for noise reduction
        enhanced = self.spectral_subtraction(audio_data)

        return enhanced

    def update_noise_profile(self, audio_data):
        """Update the noise profile from quiet audio segments"""
        if self.noise_profile_samples < self.max_noise_samples:
            # Use the current audio as potential noise sample
            # In practice, you'd only update during detected silence
            if np.mean(audio_data ** 2) < self.noise_threshold:
                if self.noise_profile is None:
                    self.noise_profile = np.abs(fft(audio_data[:self.frame_size]))
                else:
                    # Average with existing noise profile
                    current_spectrum = np.abs(fft(audio_data[:self.frame_size]))
                    self.noise_profile = 0.9 * self.noise_profile + 0.1 * current_spectrum

                self.noise_profile_samples += 1

    def spectral_subtraction(self, audio_data):
        """Apply spectral subtraction noise reduction"""
        # Pad audio to frame size if necessary
        if len(audio_data) % self.frame_size != 0:
            pad_length = self.frame_size - (len(audio_data) % self.frame_size)
            audio_data = np.pad(audio_data, (0, pad_length), mode='constant')

        enhanced_audio = np.zeros_like(audio_data)

        for i in range(0, len(audio_data) - self.frame_size + 1, self.hop_size):
            frame = audio_data[i:i + self.frame_size]

            # Apply window to reduce artifacts
            windowed_frame = frame * np.hanning(self.frame_size)

            # FFT
            spectrum = fft(windowed_frame)
            magnitude = np.abs(spectrum)
            phase = np.angle(spectrum)

            # Apply noise reduction
            if self.noise_profile is not None:
                enhanced_magnitude = np.maximum(
                    magnitude - self.enhancement_factor * self.noise_profile,
                    0.3 * magnitude  # Don't suppress too much
                )
            else:
                enhanced_magnitude = magnitude

            # Reconstruct spectrum
            enhanced_spectrum = enhanced_magnitude * np.exp(1j * phase)

            # IFFT
            enhanced_frame = np.real(ifft(enhanced_spectrum))

            # Apply window again and overlap-add
            enhanced_frame = enhanced_frame * np.hanning(self.frame_size)
            enhanced_audio[i:i + self.frame_size] += enhanced_frame

        return enhanced_audio

def main(args=None):
    rclpy.init(args=args)
    node = AudioEnhancementNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down audio enhancement node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Command Execution

### Mapping Voice Commands to Robot Actions

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time

class VoiceCommandExecutorNode(Node):
    def __init__(self):
        super().__init__('voice_command_executor_node')

        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10
        )
        self.intent_sub = self.create_subscription(
            String, 'recognized_intent', self.intent_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)
        self.speech_pub = self.create_publisher(String, 'speak_request', 10)

        # Command execution state
        self.is_executing = False
        self.current_action = None

        # Movement parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.move_duration = 2.0  # seconds

        self.get_logger().info('Voice Command Executor Node initialized')

    def voice_command_callback(self, msg):
        """Execute voice commands directly"""
        command = msg.data.lower().strip()
        self.get_logger().info(f'Executing voice command: {command}')

        if 'forward' in command:
            self.move_forward()
        elif 'backward' in command or 'back' in command:
            self.move_backward()
        elif 'left' in command:
            self.turn_left()
        elif 'right' in command:
            self.turn_right()
        elif 'stop' in command or 'halt' in command:
            self.stop_robot()
        elif 'hello' in command or 'hi' in command:
            self.greet_user()
        elif 'dance' in command:
            self.perform_dance()
        else:
            self.speak_response(f"I don't know how to {command}")

    def intent_callback(self, msg):
        """Execute commands from NLU system"""
        try:
            intent_data = json.loads(msg.data)
            intent = intent_data.get('intent', 'unknown')
            parameters = intent_data.get('parameters', {})

            self.get_logger().info(f'Executing intent: {intent} with params: {parameters}')

            if intent == 'move':
                direction = parameters.get('direction', 'forward')
                self.execute_move_command(direction, parameters)
            elif intent == 'grasp':
                obj = parameters.get('object', 'unknown object')
                self.execute_grasp_command(obj)
            elif intent == 'navigate':
                location = parameters.get('location', 'unknown location')
                self.execute_navigation_command(location)
            else:
                self.speak_response(f"Can't execute intent: {intent}")

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in intent message')

    def execute_move_command(self, direction, parameters):
        """Execute movement commands"""
        cmd = Twist()

        if direction == 'forward':
            cmd.linear.x = self.linear_speed
        elif direction == 'backward':
            cmd.linear.x = -self.linear_speed
        elif direction == 'left':
            cmd.angular.z = self.angular_speed
        elif direction == 'right':
            cmd.angular.z = -self.angular_speed
        elif direction == 'up':
            cmd.linear.z = self.linear_speed
        elif direction == 'down':
            cmd.linear.z = -self.linear_speed

        self.cmd_vel_pub.publish(cmd)

        # Stop after duration
        time.sleep(self.move_duration)
        self.stop_robot()

    def execute_grasp_command(self, obj):
        """Execute grasping commands"""
        self.speak_response(f"Looking for {obj} to grasp")
        # In practice, this would involve:
        # 1. Object detection
        # 2. Motion planning to object
        # 3. Grasping execution
        self.speak_response(f"Grasped {obj}")

    def execute_navigation_command(self, location):
        """Execute navigation commands"""
        self.speak_response(f"Navigating to {location}")
        # In practice, this would use Nav2 to navigate to a known location
        self.speak_response(f"Arrived at {location}")

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(self.move_duration)
        self.stop_robot()

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        time.sleep(self.move_duration)
        self.stop_robot()

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(cmd)
        time.sleep(1.0)  # Quarter turn
        self.stop_robot()

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -self.angular_speed
        self.cmd_vel_pub.publish(cmd)
        time.sleep(1.0)  # Quarter turn
        self.stop_robot()

    def stop_robot(self):
        """Stop all robot motion"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def greet_user(self):
        """Greet the user"""
        self.speak_response("Hello! How can I help you today?")

    def perform_dance(self):
        """Perform a simple dance"""
        self.speak_response("Let me dance for you!")

        # Simple dance sequence
        for _ in range(2):
            self.turn_left()
            time.sleep(0.5)
            self.turn_right()
            time.sleep(0.5)

        self.speak_response("Wasn't that fun?")

    def speak_response(self, text):
        """Publish text to be spoken"""
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        self.get_logger().info(f'Speaking: {text}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice command executor node...')
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Integration and Launch

### Complete Voice Processing System Launch File

```xml
<!-- voice_processing_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Voice processing node
        Node(
            package='your_robot_package',
            executable='voice_processing_node',
            name='voice_processor',
            parameters=[
                {'energy_threshold': 3000},
                {'dynamic_energy_threshold': True},
                {'pause_threshold': 0.8}
            ],
            output='screen'
        ),

        # Natural Language Understanding node
        Node(
            package='your_robot_package',
            executable='nlu_processor_node',
            name='nlu_processor',
            parameters=[
                {'confidence_threshold': 0.7}
            ],
            output='screen'
        ),

        # Voice Activity Detection node
        Node(
            package='your_robot_package',
            executable='vad_node',
            name='voice_activity_detector',
            output='screen'
        ),

        # Audio Enhancement node
        Node(
            package='your_robot_package',
            executable='audio_enhancement_node',
            name='audio_enhancer',
            output='screen'
        ),

        # Voice Command Executor node
        Node(
            package='your_robot_package',
            executable='voice_command_executor_node',
            name='voice_command_executor',
            output='screen'
        ),

        # LLM Voice Processor node (if using cloud services)
        Node(
            package='your_robot_package',
            executable='llm_voice_processor_node',
            name='llm_voice_processor',
            output='screen'
        )
    ])
```

## Testing Voice Processing Systems

### Unit Tests for Voice Processing

```python
#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class TestVoiceProcessing(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_voice_processing_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_speech_recognition(self):
        """Test that speech recognition processes audio correctly"""
        # This would test the speech recognition pipeline
        self.assertTrue(True)  # Placeholder for actual test

    def test_intent_recognition(self):
        """Test that intents are recognized correctly"""
        # This would test the NLU component
        self.assertTrue(True)  # Placeholder for actual test

    def test_command_execution(self):
        """Test that voice commands are executed properly"""
        # This would test the command execution
        self.assertTrue(True)  # Placeholder for actual test

if __name__ == '__main__':
    unittest.main()
```

## Troubleshooting Common Issues

### 1. Poor Recognition Accuracy
- **Cause**: High background noise
- **Solution**: Implement noise reduction preprocessing
- **Check**: Use audio enhancement techniques

### 2. High Latency
- **Cause**: Processing intensive algorithms
- **Solution**: Optimize for real-time performance
- **Check**: Use efficient algorithms and hardware acceleration

### 3. False Activations
- **Cause**: Voice activity detection too sensitive
- **Solution**: Adjust VAD thresholds and parameters
- **Check**: Fine-tune energy thresholds

### 4. Context Loss
- **Cause**: No conversation history management
- **Solution**: Implement proper context management
- **Check**: Maintain conversation state

## Best Practices

### 1. Privacy Considerations
- Minimize data transmission to cloud services
- Implement local processing where possible
- Provide clear privacy controls

### 2. Accessibility
- Support multiple languages
- Handle different accents and speech patterns
- Provide alternative interaction methods

### 3. Robustness
- Implement fallback mechanisms
- Handle network failures gracefully
- Provide clear error feedback

### 4. Performance
- Optimize for real-time operation
- Use efficient algorithms
- Consider computational constraints

## Summary

Voice processing systems enable natural human-robot interaction through spoken language. Key components include:

1. **Speech Recognition**: Converting audio to text
2. **Natural Language Understanding**: Interpreting user intent
3. **Voice Activity Detection**: Identifying speech in audio streams
4. **Audio Enhancement**: Improving signal quality
5. **Command Execution**: Mapping voice commands to robot actions

By implementing these components with proper integration, humanoid robots can engage in natural, spoken interaction with users, significantly improving the user experience and accessibility of robotic systems.