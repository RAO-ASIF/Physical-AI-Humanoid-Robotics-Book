---
title: LLM Integration for VLA Systems
sidebar_position: 4
---

# LLM Integration for VLA Systems

Large Language Model (LLM) integration enables humanoid robots to understand complex natural language commands and engage in sophisticated dialogue. This section covers the implementation of LLM-based language understanding and reasoning for Vision-Language-Action systems.

## Overview of LLM Integration

LLM integration in humanoid robotics transforms how robots understand and respond to human commands. By leveraging the reasoning capabilities of large language models, robots can interpret ambiguous requests, maintain conversation context, and plan complex multi-step tasks based on natural language input.

### Key Benefits of LLM Integration

#### 1. Natural Language Understanding
- **Contextual Comprehension**: Understanding commands in environmental and situational context
- **Ambiguity Resolution**: Clarifying unclear or incomplete requests
- **Dialogue Management**: Maintaining coherent conversations

#### 2. Task Planning and Reasoning
- **Goal Decomposition**: Breaking complex requests into executable steps
- **Common Sense Reasoning**: Applying world knowledge to task execution
- **Adaptive Planning**: Adjusting plans based on new information

#### 3. Social Interaction
- **Personality and Engagement**: Creating more natural human-robot interactions
- **Emotion Recognition**: Responding appropriately to human emotions
- **Cultural Sensitivity**: Adapting to different cultural contexts

## LLM Architecture for Robotics

### Integration Approaches

#### 1. Cloud-Based Integration
Benefits:
- Access to most capable models
- Continuous updates and improvements
- Minimal computational requirements on robot

Challenges:
- Internet dependency
- Latency concerns
- Privacy considerations
- Bandwidth limitations

#### 2. Edge-Based Integration
Benefits:
- Low latency responses
- Privacy preservation
- Offline capability
- Reduced bandwidth usage

Challenges:
- Limited computational resources
- Smaller model capabilities
- Storage constraints
- Maintenance complexity

#### 3. Hybrid Approach
Benefits:
- Balance between capability and responsiveness
- Fallback capabilities
- Selective data processing
- Adaptive model selection

Challenges:
- Complexity of orchestration
- Consistency across models
- Resource management

## OpenAI Integration

### Basic OpenAI API Integration

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
import threading
import queue
from typing import Dict, Any, Optional
import time

class OpenAILLMNode(Node):
    def __init__(self):
        super().__init__('openai_llm_node')

        # Publishers
        self.response_pub = self.create_publisher(String, 'llm_response', 10)
        self.thought_pub = self.create_publisher(String, 'llm_thought', 10)
        self.command_pub = self.create_publisher(String, 'llm_command', 10)

        # Subscribers
        self.user_input_sub = self.create_subscription(
            String, 'user_input', self.user_input_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, 'perception_update', self.perception_callback, 10
        )

        # OpenAI API configuration
        # In practice, set this via environment variable
        openai.api_key = "YOUR_OPENAI_API_KEY"

        # LLM configuration
        self.model = "gpt-3.5-turbo"
        self.temperature = 0.3
        self.max_tokens = 200

        # Context management
        self.conversation_history = []
        self.max_history_length = 10
        self.system_context = self.initialize_system_context()

        # Robot state and capabilities
        self.robot_capabilities = {
            "locomotion": ["move forward", "move backward", "turn left", "turn right", "stop"],
            "manipulation": ["pick up", "place", "grasp", "release", "carry"],
            "navigation": ["go to location", "follow person", "avoid obstacles"],
            "communication": ["speak", "listen", "express emotion"],
            "perception": ["detect objects", "recognize faces", "understand scenes"]
        }

        # Threading for async API calls
        self.api_queue = queue.Queue()
        self.api_thread = threading.Thread(target=self.api_worker, daemon=True)
        self.api_thread.start()

        # Status tracking
        self.is_processing = False
        self.last_response_time = 0

        self.get_logger().info('OpenAI LLM Node initialized')

    def initialize_system_context(self) -> str:
        """Initialize the system context for the LLM"""
        context = (
            f"You are an intelligent assistant for a humanoid robot. "
            f"The robot has these capabilities: {self.robot_capabilities}. "
            f"Available locations: kitchen, living room, bedroom, office, hallway. "
            f"Common objects: cup, book, bottle, chair, table, phone, keys. "
            f"Your role is to interpret user requests and provide JSON responses "
            f"with action plans. Respond with a JSON object containing "
            f"'action', 'parameters', and 'explanation'. If you need clarification, "
            f"ask for it in the 'action' field as 'request_clarification'."
        )
        return context

    def user_input_callback(self, msg):
        """Handle user input and process with LLM"""
        user_text = msg.data
        self.get_logger().info(f'Received user input: {user_text}')

        if self.is_processing:
            self.get_logger().warn('LLM is still processing previous request')
            return

        # Add to conversation history
        self.conversation_history.append({"role": "user", "content": user_text})

        # Keep history to manageable size
        if len(self.conversation_history) > self.max_history_length:
            self.conversation_history = self.conversation_history[-self.max_history_length:]

        # Queue the request for processing
        self.api_queue.put({
            'user_input': user_text,
            'timestamp': time.time()
        })

        self.is_processing = True

    def perception_callback(self, msg):
        """Handle perception updates that affect context"""
        perception_data = msg.data
        # Update internal state based on perception
        # This would typically update object locations, etc.
        pass

    def api_worker(self):
        """Worker thread for handling API calls"""
        while True:
            try:
                # Get request from queue (with timeout)
                request = self.api_queue.get(timeout=1.0)

                # Process the request
                response = self.call_openai_api(request['user_input'])
                self.handle_llm_response(response, request['user_input'])

                # Mark request as processed
                self.api_queue.task_done()
                self.is_processing = False

            except queue.Empty:
                # Timeout occurred, continue loop
                continue
            except Exception as e:
                self.get_logger().error(f'API worker error: {e}')
                self.is_processing = False

    def call_openai_api(self, user_input: str) -> Optional[str]:
        """Call OpenAI API and return response"""
        try:
            # Prepare messages for the API
            messages = [
                {"role": "system", "content": self.system_context},
                *self.conversation_history,
                {"role": "user", "content": user_input}
            ]

            # Call the API
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )

            # Extract the response
            llm_response = response.choices[0].message['content'].strip()
            return llm_response

        except Exception as e:
            self.get_logger().error(f'OpenAI API error: {e}')
            return None

    def handle_llm_response(self, response: Optional[str], original_input: str):
        """Handle the LLM response and publish appropriate messages"""
        if response is None:
            self.get_logger().error('LLM returned no response')
            return

        # Publish thought process
        thought_msg = String()
        thought_msg.data = f"Input: {original_input} | Response: {response}"
        self.thought_pub.publish(thought_msg)

        # Try to extract JSON from response
        action_data = self.extract_json_from_response(response)

        if action_data:
            # Publish structured command
            command_msg = String()
            command_msg.data = json.dumps(action_data)
            self.command_pub.publish(command_msg)

            # Add to conversation history
            self.conversation_history.append({"role": "assistant", "content": json.dumps(action_data)})

            self.get_logger().info(f'LLM processed action: {action_data}')
        else:
            # If no JSON, treat as text response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)

            # Add to conversation history
            self.conversation_history.append({"role": "assistant", "content": response})

            self.get_logger().info(f'LLM text response: {response}')

    def extract_json_from_response(self, response: str) -> Optional[Dict[str, Any]]:
        """Extract JSON from LLM response"""
        try:
            # Try to find JSON in response
            import re
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_match:
                json_str = json_match.group()
                return json.loads(json_str)
        except json.JSONDecodeError:
            # If direct JSON fails, try to wrap and parse
            try:
                # Wrap in quotes if it looks like a simple response
                if not response.strip().startswith('{'):
                    wrapped = f'{{"action": "speak", "parameters": {{"text": "{response}"}}}}'
                    return json.loads(wrapped)
            except json.JSONDecodeError:
                pass

        return None

def main(args=None):
    rclpy.init(args=args)
    node = OpenAILLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down OpenAI LLM node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Local LLM Integration

### Using Hugging Face Models

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
import json
import threading
import queue
from typing import Dict, Any, Optional
import time

class LocalLLMNode(Node):
    def __init__(self):
        super().__init__('local_llm_node')

        # Publishers
        self.response_pub = self.create_publisher(String, 'local_llm_response', 10)
        self.command_pub = self.create_publisher(String, 'local_llm_command', 10)

        # Subscribers
        self.user_input_sub = self.create_subscription(
            String, 'user_input', self.user_input_callback, 10
        )

        # Model configuration
        self.model_name = "microsoft/DialoGPT-medium"  # Example model
        self.max_length = 1000
        self.temperature = 0.7
        self.top_p = 0.9

        # Initialize tokenizer and model
        self.tokenizer = None
        self.model = None
        self.conversation_history = ""
        self.max_history_length = 1000

        # Threading for model processing
        self.process_queue = queue.Queue()
        self.process_thread = threading.Thread(target=self.process_worker, daemon=True)
        self.process_thread.start()

        # Status tracking
        self.is_processing = False

        # Load model in background
        self.load_model()

        self.get_logger().info('Local LLM Node initialized')

    def load_model(self):
        """Load the local LLM model"""
        try:
            self.get_logger().info(f'Loading model: {self.model_name}')
            self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)

            # Add padding token if not present
            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token

            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                device_map="auto" if torch.cuda.is_available() else None
            )

            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')

    def user_input_callback(self, msg):
        """Handle user input and process with local LLM"""
        user_text = msg.data
        self.get_logger().info(f'Received user input: {user_text}')

        if self.is_processing:
            self.get_logger().warn('Local LLM is still processing previous request')
            return

        if self.model is None or self.tokenizer is None:
            self.get_logger().error('Model not loaded')
            return

        # Add to processing queue
        self.process_queue.put({
            'user_input': user_text,
            'timestamp': time.time()
        })

        self.is_processing = True

    def process_worker(self):
        """Worker thread for processing with local LLM"""
        while True:
            try:
                # Get request from queue (with timeout)
                request = self.process_queue.get(timeout=1.0)

                # Process the request
                response = self.generate_response(request['user_input'])
                self.handle_response(response, request['user_input'])

                # Mark request as processed
                self.process_queue.task_done()
                self.is_processing = False

            except queue.Empty:
                # Timeout occurred, continue loop
                continue
            except Exception as e:
                self.get_logger().error(f'Processing worker error: {e}')
                self.is_processing = False

    def generate_response(self, user_input: str) -> Optional[str]:
        """Generate response using local LLM"""
        if self.model is None or self.tokenizer is None:
            return None

        try:
            # Prepare input with conversation history
            input_text = f"{self.conversation_history}User: {user_input}\nRobot:"

            # Tokenize input
            inputs = self.tokenizer.encode(
                input_text,
                return_tensors='pt',
                truncation=True,
                max_length=512
            )

            if torch.cuda.is_available():
                inputs = inputs.cuda()

            # Generate response
            with torch.no_grad():
                outputs = self.model.generate(
                    inputs,
                    max_length=len(inputs[0]) + 100,
                    temperature=self.temperature,
                    top_p=self.top_p,
                    do_sample=True,
                    pad_token_id=self.tokenizer.eos_token_id
                )

            # Decode response
            response_text = self.tokenizer.decode(
                outputs[0][len(inputs[0]):],
                skip_special_tokens=True
            ).strip()

            # Update conversation history
            self.conversation_history = f"{input_text}{response_text}\n"

            # Keep history to manageable size
            if len(self.conversation_history) > self.max_history_length:
                self.conversation_history = self.conversation_history[-self.max_history_length:]

            return response_text

        except Exception as e:
            self.get_logger().error(f'Error generating response: {e}')
            return None

    def handle_response(self, response: Optional[str], original_input: str):
        """Handle the local LLM response"""
        if response is None:
            self.get_logger().error('Local LLM returned no response')
            return

        # Try to extract structured command from response
        command_data = self.extract_command_from_response(response)

        if command_data:
            # Publish structured command
            command_msg = String()
            command_msg.data = json.dumps(command_data)
            self.command_pub.publish(command_msg)

            self.get_logger().info(f'Local LLM processed command: {command_data}')
        else:
            # Publish as text response
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'Local LLM response: {response}')

    def extract_command_from_response(self, response: str) -> Optional[Dict[str, Any]]:
        """Extract command from response (simplified)"""
        # In a real implementation, you'd have a more sophisticated parser
        # This is a basic example

        response_lower = response.lower()

        if 'move' in response_lower or 'go' in response_lower:
            return {
                'action': 'move',
                'parameters': {'direction': 'forward', 'distance': 1.0}
            }
        elif 'speak' in response_lower or 'say' in response_lower:
            return {
                'action': 'speak',
                'parameters': {'text': response}
            }
        elif 'grasp' in response_lower or 'pick' in response_lower:
            return {
                'action': 'grasp',
                'parameters': {'object': 'unknown'}
            }

        return None

def main(args=None):
    rclpy.init(args=args)
    node = LocalLLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down local LLM node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## LLM-Based Task Planning

### Hierarchical Task Planning with LLMs

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
from typing import List, Dict, Any
import time

class LLMTaskPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_task_planner_node')

        # Publishers
        self.task_plan_pub = self.create_publisher(String, 'llm_task_plan', 10)
        self.subtask_pub = self.create_publisher(String, 'llm_subtask', 10)
        self.planning_status_pub = self.create_publisher(String, 'llm_planning_status', 10)

        # Subscribers
        self.high_level_goal_sub = self.create_subscription(
            String, 'high_level_goal', self.high_level_goal_callback, 10
        )
        self.task_completion_sub = self.create_subscription(
            String, 'task_completion', self.task_completion_callback, 10
        )

        # OpenAI API configuration
        openai.api_key = "YOUR_OPENAI_API_KEY"

        # Planning state
        self.current_plan = []
        self.current_subtask_index = 0
        self.planning_context = self.initialize_planning_context()

        self.get_logger().info('LLM Task Planner Node initialized')

    def initialize_planning_context(self) -> str:
        """Initialize context for task planning"""
        context = (
            "You are a task planner for a humanoid robot. Given a high-level goal, "
            "decompose it into specific, executable subtasks. Each subtask should be "
            "atomic and achievable. Consider the robot's capabilities and the environment. "
            "Return a JSON array of subtasks, each with 'action', 'target', 'location', "
            "and 'description'. Available actions: navigate, detect_object, grasp_object, "
            "place_object, speak, follow_person, wait. Available locations: kitchen, "
            "living_room, bedroom, office, hallway. Return only valid JSON."
        )
        return context

    def high_level_goal_callback(self, msg):
        """Handle high-level goals and generate task plans"""
        goal = msg.data
        self.get_logger().info(f'Planning for goal: {goal}')

        # Update planning status
        status_msg = String()
        status_msg.data = f"planning_started:{goal}"
        self.planning_status_pub.publish(status_msg)

        try:
            # Generate task plan using LLM
            plan = self.generate_task_plan(goal)

            if plan:
                self.current_plan = plan
                self.current_subtask_index = 0

                # Publish full plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.task_plan_pub.publish(plan_msg)

                # Publish first subtask
                if plan:
                    self.publish_next_subtask()

                self.get_logger().info(f'Generated plan with {len(plan)} subtasks')
            else:
                self.get_logger().error('Failed to generate task plan')

        except Exception as e:
            self.get_logger().error(f'Task planning error: {e}')

    def generate_task_plan(self, goal: str) -> List[Dict[str, Any]]:
        """Generate task plan using LLM"""
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": self.planning_context
                    },
                    {
                        "role": "user",
                        "content": f"Goal: {goal}. Generate a task plan."
                    }
                ],
                max_tokens=500,
                temperature=0.3
            )

            llm_response = response.choices[0].message['content'].strip()

            # Extract JSON from response
            import re
            json_match = re.search(r'\[.*\]', llm_response, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group())
                return plan
            else:
                self.get_logger().warn('Could not extract plan from LLM response')
                return []

        except Exception as e:
            self.get_logger().error(f'LLM task planning error: {e}')
            return []

    def publish_next_subtask(self):
        """Publish the next subtask in the plan"""
        if (self.current_subtask_index < len(self.current_plan) and
            self.current_plan):

            subtask = self.current_plan[self.current_subtask_index]

            subtask_msg = String()
            subtask_msg.data = json.dumps(subtask)
            self.subtask_pub.publish(subtask_msg)

            self.get_logger().info(f'Published subtask {self.current_subtask_index + 1}: {subtask["action"]}')

            # Update planning status
            status_msg = String()
            status_msg.data = f"subtask_started:{subtask['action']}"
            self.planning_status_pub.publish(status_msg)

    def task_completion_callback(self, msg):
        """Handle task completion and move to next subtask"""
        completion_data = msg.data
        self.get_logger().info(f'Task completion: {completion_data}')

        # Move to next subtask
        self.current_subtask_index += 1

        if self.current_subtask_index < len(self.current_plan):
            # Publish next subtask
            self.publish_next_subtask()
        else:
            # Plan completed
            self.get_logger().info('Task plan completed')
            status_msg = String()
            status_msg.data = "plan_completed"
            self.planning_status_pub.publish(status_msg)

            # Reset for next plan
            self.current_plan = []
            self.current_subtask_index = 0

def main(args=None):
    rclpy.init(args=args)
    node = LLMTaskPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM task planner node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Context and Memory Management

### Long-term Memory with LLMs

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openai
import json
import sqlite3
import time
from typing import Dict, Any, List
import os

class LLMContextManagerNode(Node):
    def __init__(self):
        super().__init__('llm_context_manager_node')

        # Publishers
        self.context_update_pub = self.create_publisher(String, 'context_update', 10)
        self.knowledge_query_pub = self.create_publisher(String, 'knowledge_response', 10)

        # Subscribers
        self.interaction_sub = self.create_subscription(
            String, 'interaction_record', self.interaction_callback, 10
        )
        self.knowledge_query_sub = self.create_subscription(
            String, 'knowledge_query', self.knowledge_query_callback, 10
        )
        self.memory_request_sub = self.create_subscription(
            String, 'memory_request', self.memory_request_callback, 10
        )

        # OpenAI API configuration
        openai.api_key = "YOUR_OPENAI_API_KEY"

        # Database setup
        self.db_path = "/tmp/robot_memory.db"
        self.init_database()

        # Context management
        self.context_window = []  # Recent interactions
        self.max_context_length = 50  # Number of interactions to keep

        # Timer for periodic memory consolidation
        self.consolidation_timer = self.create_timer(300.0, self.consolidate_memory)  # Every 5 minutes

        self.get_logger().info('LLM Context Manager Node initialized')

    def init_database(self):
        """Initialize the memory database"""
        self.conn = sqlite3.connect(self.db_path)
        cursor = self.conn.cursor()

        # Create tables for different types of memories
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS interactions (
                id INTEGER PRIMARY KEY,
                timestamp REAL,
                user_input TEXT,
                robot_response TEXT,
                context TEXT,
                importance REAL DEFAULT 0.5
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS facts (
                id INTEGER PRIMARY KEY,
                timestamp REAL,
                subject TEXT,
                predicate TEXT,
                object TEXT,
                source TEXT,
                confidence REAL
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS episodic_memory (
                id INTEGER PRIMARY KEY,
                timestamp REAL,
                event_description TEXT,
                location TEXT,
                participants TEXT,
                summary TEXT
            )
        ''')

        self.conn.commit()

    def interaction_callback(self, msg):
        """Handle interaction records for memory storage"""
        try:
            interaction_data = json.loads(msg.data)
            user_input = interaction_data.get('user_input', '')
            robot_response = interaction_data.get('robot_response', '')
            context = interaction_data.get('context', '')

            # Calculate importance using LLM
            importance = self.calculate_interaction_importance(
                user_input, robot_response, context
            )

            # Store in database
            cursor = self.conn.cursor()
            cursor.execute('''
                INSERT INTO interactions (timestamp, user_input, robot_response, context, importance)
                VALUES (?, ?, ?, ?, ?)
            ''', (time.time(), user_input, robot_response, context, importance))

            self.conn.commit()

            # Add to context window
            self.context_window.append({
                'timestamp': time.time(),
                'user_input': user_input,
                'robot_response': robot_response,
                'importance': importance
            })

            # Keep context window to manageable size
            if len(self.context_window) > self.max_context_length:
                self.context_window = self.context_window[-self.max_context_length:]

            self.get_logger().info(f'Stored interaction with importance: {importance:.2f}')

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in interaction record')
        except Exception as e:
            self.get_logger().error(f'Error handling interaction: {e}')

    def calculate_interaction_importance(self, user_input: str, robot_response: str, context: str) -> float:
        """Calculate importance of interaction using LLM"""
        try:
            prompt = (
                f"Rate the importance of this interaction on a scale of 0.0 to 1.0. "
                f"Consider if it contains important information, decisions, or personal details. "
                f"User: {user_input} "
                f"Robot: {robot_response} "
                f"Context: {context} "
                f"Rate only with a number between 0.0 and 1.0:"
            )

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=10,
                temperature=0.1
            )

            importance_str = response.choices[0].message['content'].strip()
            importance = float(importance_str)
            return max(0.0, min(1.0, importance))  # Clamp between 0 and 1

        except:
            return 0.5  # Default importance

    def knowledge_query_callback(self, msg):
        """Handle knowledge queries from LLM system"""
        query = msg.data
        self.get_logger().info(f'Knowledge query: {query}')

        # Search relevant information from memory
        relevant_info = self.search_memory(query)

        # Generate response using LLM
        response = self.generate_knowledge_response(query, relevant_info)

        # Publish response
        response_msg = String()
        response_msg.data = json.dumps({
            'query': query,
            'response': response,
            'sources': [info[0] for info in relevant_info[:3]]  # Top 3 sources
        })
        self.knowledge_query_pub.publish(response_msg)

    def search_memory(self, query: str) -> List[tuple]:
        """Search relevant information from memory"""
        cursor = self.conn.cursor()

        # Search interactions
        cursor.execute('''
            SELECT user_input, robot_response, timestamp, importance
            FROM interactions
            WHERE user_input LIKE ? OR robot_response LIKE ?
            ORDER BY importance DESC, timestamp DESC
            LIMIT 10
        ''', (f'%{query}%', f'%{query}%'))

        interactions = cursor.fetchall()

        # Search facts
        cursor.execute('''
            SELECT subject, predicate, object, confidence
            FROM facts
            WHERE subject LIKE ? OR predicate LIKE ? OR object LIKE ?
            ORDER BY confidence DESC
            LIMIT 10
        ''', (f'%{query}%', f'%{query}%', f'%{query}%'))

        facts = cursor.fetchall()

        return interactions + facts

    def generate_knowledge_response(self, query: str, relevant_info: List[tuple]) -> str:
        """Generate knowledge response using LLM"""
        try:
            context_str = "\n".join([str(info) for info in relevant_info[:5]])  # Use top 5

            prompt = (
                f"Based on the following information, answer the question: {query}\n\n"
                f"Relevant information:\n{context_str}\n\n"
                f"Answer:"
            )

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=150,
                temperature=0.3
            )

            return response.choices[0].message['content'].strip()

        except Exception as e:
            self.get_logger().error(f'Error generating knowledge response: {e}')
            return "I don't have specific information about that."

    def memory_request_callback(self, msg):
        """Handle specific memory requests"""
        request = json.loads(msg.data)
        request_type = request.get('type', '')
        query = request.get('query', '')

        if request_type == 'recall_recent':
            # Return recent interactions
            recent = self.context_window[-5:]  # Last 5 interactions
            response = json.dumps(recent)
        elif request_type == 'recall_facts':
            # Return relevant facts
            facts = self.search_facts(query)
            response = json.dumps(facts)
        else:
            response = json.dumps([])

        response_msg = String()
        response_msg.data = response
        self.context_update_pub.publish(response_msg)

    def search_facts(self, query: str) -> List[Dict[str, Any]]:
        """Search for specific facts"""
        cursor = self.conn.cursor()
        cursor.execute('''
            SELECT subject, predicate, object, confidence
            FROM facts
            WHERE subject LIKE ? OR object LIKE ?
            ORDER BY confidence DESC
            LIMIT 10
        ''', (f'%{query}%', f'%{query}%'))

        results = []
        for row in cursor.fetchall():
            results.append({
                'subject': row[0],
                'predicate': row[1],
                'object': row[2],
                'confidence': row[3]
            })

        return results

    def consolidate_memory(self):
        """Periodically consolidate and summarize memories"""
        self.get_logger().info('Consolidating memory...')

        try:
            # Find old, low-importance interactions to summarize
            cursor = self.conn.cursor()
            cursor.execute('''
                SELECT id, user_input, robot_response, timestamp
                FROM interactions
                WHERE importance < 0.3 AND timestamp < ?
                ORDER BY timestamp ASC
                LIMIT 20
            ''', (time.time() - 86400,))  # Interactions older than 1 day

            old_interactions = cursor.fetchall()

            if len(old_interactions) > 5:
                # Summarize old interactions using LLM
                interactions_text = "\n".join([
                    f"User: {item[1]}\nRobot: {item[2]}"
                    for item in old_interactions
                ])

                prompt = (
                    f"Summarize the following conversation snippets into key points "
                    f"and important facts. Focus on information that should be remembered:\n\n"
                    f"{interactions_text}"
                )

                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[{"role": "user", "content": prompt}],
                    max_tokens=200,
                    temperature=0.3
                )

                summary = response.choices[0].message['content'].strip()

                # Store summary as episodic memory
                cursor.execute('''
                    INSERT INTO episodic_memory (timestamp, event_description, summary)
                    VALUES (?, ?, ?)
                ''', (time.time(), f"Summary of {len(old_interactions)} old interactions", summary))

                # Delete old interactions
                old_ids = [str(item[0]) for item in old_interactions]
                cursor.execute(
                    f"DELETE FROM interactions WHERE id IN ({','.join(old_ids)})"
                )

                self.conn.commit()

                self.get_logger().info(f'Consolidated {len(old_interactions)} old interactions into summary')

        except Exception as e:
            self.get_logger().error(f'Error during memory consolidation: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LLMContextManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM context manager node...')
        node.conn.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety and Ethics in LLM Integration

### Safe LLM Interaction Framework

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai
import json
import re
from typing import Dict, Any, List
import time

class SafeLLMFrameworkNode(Node):
    def __init__(self):
        super().__init__('safe_llm_framework_node')

        # Publishers
        self.safe_response_pub = self.create_publisher(String, 'safe_llm_response', 10)
        self.safety_alert_pub = self.create_publisher(String, 'safety_alert', 10)
        self.filtered_command_pub = self.create_publisher(String, 'filtered_command', 10)

        # Subscribers
        self.raw_input_sub = self.create_subscription(
            String, 'raw_user_input', self.raw_input_callback, 10
        )
        self.llm_output_sub = self.create_subscription(
            String, 'llm_output', self.llm_output_callback, 10
        )

        # OpenAI API configuration
        openai.api_key = "YOUR_OPENAI_API_KEY"

        # Safety configuration
        self.prohibited_categories = [
            'violence', 'harm', 'discrimination', 'inappropriate_content',
            'dangerous_activities', 'privacy_violations'
        ]
        self.safety_threshold = 0.8  # Confidence threshold for safety checks

        # Content filtering patterns
        self.dangerous_patterns = [
            r'destroy.*robot',
            r'override.*safety',
            r'harm.*human',
            r'ignore.*protocol',
            r'unsafe.*action',
        ]

        # Timer for safety system monitoring
        self.safety_timer = self.create_timer(1.0, self.safety_monitor)

        self.get_logger().info('Safe LLM Framework Node initialized')

    def raw_input_callback(self, msg):
        """Handle raw user input with safety filtering"""
        user_input = msg.data
        self.get_logger().info(f'Received raw input: {user_input}')

        # Check input safety
        safety_check = self.check_input_safety(user_input)

        if safety_check['is_safe']:
            # Input is safe, allow processing
            self.get_logger().info('Input passed safety check')

            # Publish to LLM system
            # In practice, this would go to the LLM processing node
            pass
        else:
            # Input is unsafe, handle appropriately
            self.get_logger().warn(f'Unsafe input detected: {safety_check["reasons"]}')

            # Publish safety alert
            alert_msg = String()
            alert_msg.data = f"unsafe_input:{json.dumps(safety_check)}"
            self.safety_alert_pub.publish(alert_msg)

            # Generate safe response
            safe_response = self.generate_safe_response(safety_check['reasons'])
            response_msg = String()
            response_msg.data = safe_response
            self.safe_response_pub.publish(response_msg)

    def llm_output_callback(self, msg):
        """Handle LLM output with safety filtering"""
        llm_output = msg.data
        self.get_logger().info(f'Received LLM output for safety check')

        # Check output safety
        safety_check = self.check_output_safety(llm_output)

        if safety_check['is_safe']:
            # Output is safe, allow execution
            self.get_logger().info('LLM output passed safety check')

            # Publish safe output
            response_msg = String()
            response_msg.data = llm_output
            self.safe_response_pub.publish(response_msg)
        else:
            # Output is unsafe, handle appropriately
            self.get_logger().warn(f'Unsafe LLM output detected: {safety_check["reasons"]}')

            # Publish safety alert
            alert_msg = String()
            alert_msg.data = f"unsafe_output:{json.dumps(safety_check)}"
            self.safety_alert_pub.publish(alert_msg)

            # Generate safe alternative
            safe_output = self.generate_safe_response(safety_check['reasons'])
            response_msg = String()
            response_msg.data = safe_output
            self.safe_response_pub.publish(response_msg)

    def check_input_safety(self, input_text: str) -> Dict[str, Any]:
        """Check if user input is safe using multiple methods"""
        results = {
            'is_safe': True,
            'reasons': [],
            'confidence': 1.0
        }

        # 1. Pattern matching
        for pattern in self.dangerous_patterns:
            if re.search(pattern, input_text, re.IGNORECASE):
                results['is_safe'] = False
                results['reasons'].append(f'Dangerous pattern matched: {pattern}')
                break

        # 2. LLM-based safety check
        if results['is_safe']:
            llm_safety = self.llm_safety_check(input_text)
            if not llm_safety['is_safe']:
                results['is_safe'] = False
                results['reasons'].append(f'LLM safety check failed: {llm_safety["reason"]}')
                results['confidence'] = llm_safety['confidence']

        # 3. Content category check
        if results['is_safe']:
            category_safety = self.category_safety_check(input_text)
            if not category_safety['is_safe']:
                results['is_safe'] = False
                results['reasons'].extend(category_safety['reasons'])
                results['confidence'] = min(results['confidence'], category_safety['confidence'])

        return results

    def check_output_safety(self, output_text: str) -> Dict[str, Any]:
        """Check if LLM output is safe"""
        # Similar to input safety but with different focus
        results = {
            'is_safe': True,
            'reasons': [],
            'confidence': 1.0
        }

        # Check for command injection or unsafe instructions
        dangerous_outputs = [
            'sudo', 'rm -rf', 'shutdown', 'reboot', 'chmod',
            'systemctl', 'kill', 'pkill', 'ps aux'
        ]

        output_lower = output_text.lower()
        for dangerous in dangerous_outputs:
            if dangerous in output_lower:
                results['is_safe'] = False
                results['reasons'].append(f'Dangerous command detected: {dangerous}')
                break

        # LLM-based safety check for output
        if results['is_safe']:
            llm_safety = self.llm_output_safety_check(output_text)
            if not llm_safety['is_safe']:
                results['is_safe'] = False
                results['reasons'].append(f'LLM output safety check failed: {llm_safety["reason"]}')
                results['confidence'] = llm_safety['confidence']

        return results

    def llm_safety_check(self, text: str) -> Dict[str, Any]:
        """Use LLM to check if text is safe"""
        try:
            prompt = (
                f"Analyze the following text for safety concerns in a robot interaction context. "
                f"Check for: harmful instructions, dangerous activities, inappropriate content, "
                f"privacy violations, or anything that could cause harm. "
                f"Respond with a JSON object: "
                f"{{'is_safe': boolean, 'reason': string, 'confidence': float between 0 and 1}}. "
                f"Text to analyze: {text}"
            )

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=100,
                temperature=0.1
            )

            result_str = response.choices[0].message['content'].strip()

            # Extract JSON from response
            import re
            json_match = re.search(r'\{.*\}', result_str, re.DOTALL)
            if json_match:
                result = json.loads(json_match.group())
                return result
            else:
                return {'is_safe': True, 'reason': 'Could not parse safety result', 'confidence': 0.5}

        except Exception as e:
            self.get_logger().error(f'LLM safety check error: {e}')
            return {'is_safe': True, 'reason': 'Safety check failed', 'confidence': 0.5}

    def llm_output_safety_check(self, text: str) -> Dict[str, Any]:
        """Use LLM to check if output is safe for robot execution"""
        try:
            prompt = (
                f"Analyze the following robot response for safety before execution. "
                f"Check if it contains instructions that could harm humans, damage property, "
                f"violate privacy, or cause unsafe robot behavior. "
                f"Respond with a JSON object: "
                f"{{'is_safe': boolean, 'reason': string, 'confidence': float between 0 and 1}}. "
                f"Response to analyze: {text}"
            )

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=100,
                temperature=0.1
            )

            result_str = response.choices[0].message['content'].strip()

            # Extract JSON from response
            import re
            json_match = re.search(r'\{.*\}', result_str, re.DOTALL)
            if json_match:
                result = json.loads(json_match.group())
                return result
            else:
                return {'is_safe': True, 'reason': 'Could not parse safety result', 'confidence': 0.5}

        except Exception as e:
            self.get_logger().error(f'LLM output safety check error: {e}')
            return {'is_safe': True, 'reason': 'Safety check failed', 'confidence': 0.5}

    def category_safety_check(self, text: str) -> Dict[str, Any]:
        """Check text against prohibited categories"""
        results = {
            'is_safe': True,
            'reasons': [],
            'confidence': 1.0
        }

        # This is a simplified category check
        # In practice, you'd use a more sophisticated classifier
        text_lower = text.lower()

        if any(cat in text_lower for cat in ['harm', 'violence', 'destroy', 'damage']):
            results['is_safe'] = False
            results['reasons'].append('Contains potentially harmful content')
            results['confidence'] = 0.9

        return results

    def generate_safe_response(self, safety_reasons: List[str]) -> str:
        """Generate a safe response when input/output is flagged as unsafe"""
        reasons_str = ", ".join(safety_reasons)
        return f"I understand you're asking about '{reasons_str}', but I can't assist with that as it may not be safe or appropriate. How else can I help you?"

    def safety_monitor(self):
        """Monitor safety system status"""
        # This would typically check system health, safety metrics, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SafeLLMFrameworkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down safe LLM framework node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## LLM-ROS Integration Patterns

### Complete LLM Integration System

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time
from typing import Dict, Any

class LLMIntegrationSystemNode(Node):
    def __init__(self):
        super().__init__('llm_integration_system_node')

        # Publishers for the entire system
        self.system_status_pub = self.create_publisher(String, 'llm_system_status', 10)

        # Subscribers for system coordination
        self.user_input_sub = self.create_subscription(
            String, 'user_input', self.user_input_callback, 10
        )
        self.robot_status_sub = self.create_subscription(
            String, 'robot_status', self.robot_status_callback, 10
        )
        self.task_status_sub = self.create_subscription(
            String, 'task_status', self.task_status_callback, 10
        )

        # System state
        self.system_active = True
        self.current_context = {}
        self.system_components = {
            'llm_core': False,
            'context_manager': False,
            'task_planner': False,
            'safety_framework': False,
            'response_generator': False
        }

        # Timer for system health monitoring
        self.health_timer = self.create_timer(2.0, self.system_health_check)

        # Initialize system components (in practice, these would be separate nodes)
        self.initialize_system()

        self.get_logger().info('LLM Integration System Node initialized')

    def initialize_system(self):
        """Initialize all LLM system components"""
        self.get_logger().info('Initializing LLM integration system...')

        # In practice, each component would be a separate ROS node
        # This is a simplified representation
        self.system_components['llm_core'] = True
        self.system_components['context_manager'] = True
        self.system_components['task_planner'] = True
        self.system_components['safety_framework'] = True
        self.system_components['response_generator'] = True

    def user_input_callback(self, msg):
        """Handle user input through the integrated system"""
        user_input = msg.data
        self.get_logger().info(f'Received user input: {user_input}')

        # Process through integrated system
        response = self.process_input_integrated(user_input)

        if response:
            # Publish response (in practice, this would go to speech system)
            response_msg = String()
            response_msg.data = response
            # This would typically go to the speech output system

    def robot_status_callback(self, msg):
        """Handle robot status updates"""
        status = msg.data
        # Update internal state based on robot status
        pass

    def task_status_callback(self, msg):
        """Handle task status updates from the planning system"""
        status = msg.data
        # Update internal state based on task status
        pass

    def process_input_integrated(self, user_input: str) -> str:
        """Process input through the entire integrated system"""
        # This represents the flow through all system components
        # 1. Input validation and safety check
        # 2. Context enrichment
        # 3. Task planning
        # 4. Response generation
        # 5. Safety validation
        # 6. Output

        # Simulated processing flow
        self.get_logger().info('Processing input through integrated LLM system')

        # In practice, each step would call its respective component
        processed_input = self.validate_and_enrich_input(user_input)
        task_plan = self.generate_task_plan(processed_input)
        response = self.generate_response(task_plan)

        # Safety check final response
        safe_response = self.ensure_response_safety(response)

        return safe_response

    def validate_and_enrich_input(self, user_input: str) -> Dict[str, Any]:
        """Validate and enrich user input with context"""
        return {
            'original_input': user_input,
            'timestamp': time.time(),
            'context_enriched': True
        }

    def generate_task_plan(self, enriched_input: Dict[str, Any]) -> Dict[str, Any]:
        """Generate task plan from enriched input"""
        # In practice, this would call the task planning component
        return {
            'tasks': ['interpret_request', 'plan_response', 'execute_if_needed'],
            'input': enriched_input
        }

    def generate_response(self, task_plan: Dict[str, Any]) -> str:
        """Generate natural response from task plan"""
        # In practice, this would call the response generation component
        return "I understand your request and am processing it appropriately."

    def ensure_response_safety(self, response: str) -> str:
        """Ensure the final response is safe"""
        # In practice, this would call the safety framework
        return response

    def system_health_check(self):
        """Check health of all system components"""
        all_active = all(self.system_components.values())

        status_msg = String()
        status_msg.data = f"system_healthy:{all_active}:components:{self.system_components}"
        self.system_status_pub.publish(status_msg)

        if not all_active:
            self.get_logger().warn('Some LLM system components are not active')

def main(args=None):
    rclpy.init(args=args)
    node = LLMIntegrationSystemNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM integration system node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Integration and Launch

### Complete LLM Integration System Launch File

```xml
<!-- llm_integration_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # OpenAI LLM node
        Node(
            package='your_robot_package',
            executable='openai_llm_node',
            name='openai_llm',
            parameters=[
                {'model': 'gpt-3.5-turbo'},
                {'temperature': 0.3}
            ],
            output='screen'
        ),

        # Local LLM node (if using local models)
        Node(
            package='your_robot_package',
            executable='local_llm_node',
            name='local_llm',
            output='screen'
        ),

        # LLM task planner node
        Node(
            package='your_robot_package',
            executable='llm_task_planner_node',
            name='llm_task_planner',
            output='screen'
        ),

        # LLM context manager node
        Node(
            package='your_robot_package',
            executable='llm_context_manager_node',
            name='llm_context_manager',
            output='screen'
        ),

        # Safe LLM framework node
        Node(
            package='your_robot_package',
            executable='safe_llm_framework_node',
            name='safe_llm_framework',
            output='screen'
        ),

        # Main LLM integration system node
        Node(
            package='your_robot_package',
            executable='llm_integration_system_node',
            name='llm_integration_system',
            output='screen'
        )
    ])
```

## Testing LLM Integration

### Unit Tests for LLM Components

```python
#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestLLMIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('test_llm_integration_node')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_safe_input_processing(self):
        """Test that safe inputs are processed correctly"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_unsafe_input_filtering(self):
        """Test that unsafe inputs are properly filtered"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_context_management(self):
        """Test that context is properly maintained"""
        self.assertTrue(True)  # Placeholder for actual test

    def test_task_planning(self):
        """Test that tasks are properly planned"""
        self.assertTrue(True)  # Placeholder for actual test

if __name__ == '__main__':
    unittest.main()
```

## Troubleshooting LLM Integration Issues

### 1. API Connection Problems
- **Cause**: Network issues or invalid API keys
- **Solution**: Check network connectivity and API key configuration
- **Check**: Verify environment variables and network settings

### 2. Rate Limiting
- **Cause**: Exceeding API rate limits
- **Solution**: Implement request queuing and rate limiting
- **Check**: Monitor API usage and adjust request frequency

### 3. Context Window Overflow
- **Cause**: Long conversations exceeding model limits
- **Solution**: Implement context summarization and management
- **Check**: Monitor conversation length and history

### 4. Hallucination Issues
- **Cause**: LLM generating incorrect or fabricated information
- **Solution**: Implement fact-checking and verification systems
- **Check**: Validate outputs against known facts

## Best Practices for LLM Integration

### 1. Privacy and Security
- Encrypt sensitive data transmission
- Use secure API key management
- Implement data anonymization
- Follow privacy regulations (GDPR, CCPA, etc.)

### 2. Performance Optimization
- Use appropriate model sizes for hardware capabilities
- Implement caching for common queries
- Optimize prompt engineering
- Monitor and optimize response times

### 3. Reliability
- Implement fallback mechanisms
- Use circuit breakers for API calls
- Monitor system health continuously
- Plan for graceful degradation

### 4. Ethics and Safety
- Implement comprehensive safety filters
- Ensure bias detection and mitigation
- Maintain transparency in AI decisions
- Provide clear human oversight capabilities

## Summary

LLM integration for VLA systems enables sophisticated natural language understanding and reasoning capabilities for humanoid robots. Key components include:

1. **Cloud vs Local Integration**: Balancing capability and responsiveness
2. **Task Planning**: Using LLMs for high-level task decomposition
3. **Context Management**: Maintaining conversation and memory context
4. **Safety Frameworks**: Ensuring safe and appropriate robot behavior
5. **System Integration**: Coordinating all LLM components with robot systems

These components work together to create robots that can understand complex natural language commands, maintain contextual awareness, and execute sophisticated multi-step tasks while ensuring safety and reliability.