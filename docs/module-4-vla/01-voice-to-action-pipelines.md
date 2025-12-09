---
title: "Voice-to-Action Pipelines"
sidebar_label: "Voice-to-Action Pipelines"
description: "Learning to implement voice command processing and converting speech to robot actions using Whisper and LLMs."
---

# Voice-to-Action Pipelines

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement voice command processing using speech recognition
- Convert natural language commands to robot actions
- Integrate Whisper for speech-to-text conversion
- Connect LLMs to ROS 2 action sequences
- Build end-to-end perception-planning-manipulation pipelines
- Create natural language interfaces for humanoid robots

## Introduction

Voice-to-action pipelines represent a critical component of human-robot interaction, enabling natural communication between humans and robots using spoken language. For humanoid robots, this capability is essential for creating intuitive interfaces that allow non-expert users to command robots using everyday language.

The voice-to-action pipeline typically involves several stages: speech recognition, natural language understanding, action planning, and robot execution. Modern approaches leverage large language models (LLMs) and advanced speech recognition systems like OpenAI's Whisper to create sophisticated understanding capabilities.

## Speech Recognition Fundamentals

### The Speech Recognition Pipeline

The voice-to-action process begins with converting spoken language to text:

```
Voice Command → Audio Processing → Feature Extraction → Speech Recognition → Text
```

### Audio Preprocessing

Before speech recognition, audio needs to be properly preprocessed:

```python
import numpy as np
import librosa
import sounddevice as sd

class AudioPreprocessor:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio_buffer = np.array([])

    def preprocess_audio(self, audio_data):
        # Normalize audio
        audio_data = audio_data / np.max(np.abs(audio_data))

        # Apply noise reduction (simplified)
        noise_floor = np.mean(np.abs(audio_data)) * 0.1
        audio_data = np.where(np.abs(audio_data) > noise_floor, audio_data, 0)

        # Apply pre-emphasis filter
        pre_emphasis = 0.97
        audio_data[1:] = audio_data[1:] - pre_emphasis * audio_data[:-1]

        return audio_data

    def detect_speech_activity(self, audio_data, threshold=0.01):
        """Simple voice activity detection based on energy"""
        energy = np.mean(np.abs(audio_data) ** 2)
        return energy > threshold
```

### Real-time Audio Capture

```python
import threading
import queue

class RealTimeAudioCapture:
    def __init__(self, sample_rate=16000, duration=5):
        self.sample_rate = sample_rate
        self.duration = duration
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.preprocessor = AudioPreprocessor(sample_rate)

    def audio_callback(self, indata, frames, time, status):
        """Callback for real-time audio input"""
        if status:
            print(f"Audio status: {status}")

        # Add to queue for processing
        self.audio_queue.put(indata.copy())

    def start_recording(self):
        """Start real-time audio recording"""
        self.is_recording = True
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.audio_callback,
            blocksize=1024
        )
        self.stream.start()

    def stop_recording(self):
        """Stop audio recording"""
        self.stream.stop()
        self.stream.close()
        self.is_recording = False
```

## Whisper Integration for Speech Recognition

### Setting up Whisper

OpenAI's Whisper is a state-of-the-art speech recognition model that can be used for voice-to-text conversion:

```python
import whisper
import torch
import numpy as np

class WhisperSpeechRecognizer:
    def __init__(self, model_size="base"):
        """Initialize Whisper model"""
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size).to(self.device)
        self.sample_rate = 16000

    def transcribe_audio(self, audio_data, language="en"):
        """Transcribe audio to text using Whisper"""
        # Convert to float32 if needed
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)

        # Transcribe
        result = self.model.transcribe(
            audio_data,
            language=language,
            task="transcribe"
        )

        return result["text"].strip()

    def transcribe_from_file(self, audio_file_path):
        """Transcribe from audio file"""
        result = self.model.transcribe(audio_file_path)
        return result["text"].strip()

    def transcribe_with_timestamps(self, audio_data):
        """Transcribe with word-level timestamps"""
        result = self.model.transcribe(
            audio_data,
            word_timestamps=True
        )
        return result
```

### Optimized Real-time Whisper Processing

```python
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor

class RealTimeWhisperProcessor:
    def __init__(self, model_size="base"):
        self.whisper_model = WhisperSpeechRecognizer(model_size)
        self.executor = ThreadPoolExecutor(max_workers=2)
        self.is_processing = False

    async def process_audio_chunk(self, audio_chunk):
        """Process audio chunk asynchronously"""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            self.executor,
            self.whisper_model.transcribe_audio,
            audio_chunk
        )

    def process_continuous_audio(self, audio_capture):
        """Process continuous audio stream"""
        while audio_capture.is_recording:
            try:
                # Get audio chunk from queue
                audio_chunk = audio_capture.audio_queue.get(timeout=0.1)

                # Preprocess audio
                processed_audio = audio_capture.preprocessor.preprocess_audio(audio_chunk)

                # Check for speech activity
                if audio_capture.preprocessor.detect_speech_activity(processed_audio):
                    # Transcribe the audio
                    transcription = self.whisper_model.transcribe_audio(processed_audio)

                    if transcription:  # Only process non-empty transcriptions
                        self.handle_transcription(transcription)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error processing audio: {e}")

    def handle_transcription(self, text):
        """Handle the transcribed text"""
        print(f"Transcribed: {text}")
        # Process the text through NLP pipeline
        self.process_natural_language_command(text)
```

## Natural Language Understanding

### Command Parsing and Intent Recognition

Once we have the transcribed text, we need to understand the user's intent:

```python
import re
from typing import Dict, List, Tuple
import spacy

class NaturalLanguageProcessor:
    def __init__(self):
        # Load spaCy model for NLP
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            print("Please install spaCy English model: python -m spacy download en_core_web_sm")
            self.nlp = None

        # Define command patterns
        self.command_patterns = {
            'move': [
                r'go to (.+)',
                r'move to (.+)',
                r'go (.+)',
                r'walk to (.+)',
                r'head to (.+)'
            ],
            'grasp': [
                r'pick up (.+)',
                r'grab (.+)',
                r'take (.+)',
                r'get (.+)',
                r'lift (.+)'
            ],
            'place': [
                r'put (.+) on (.+)',
                r'place (.+) on (.+)',
                r'drop (.+) on (.+)'
            ],
            'follow': [
                r'follow me',
                r'follow (.+)',
                r'come with me'
            ],
            'stop': [
                r'stop',
                r'freeze',
                r'hold on'
            ],
            'introduce': [
                r'tell me about yourself',
                r'who are you',
                r'introduce yourself'
            ]
        }

    def extract_intent_and_entities(self, text: str) -> Dict:
        """Extract intent and entities from natural language command"""
        text = text.lower().strip()

        # Pattern matching for intents
        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    entities = match.groups()
                    return {
                        'intent': intent,
                        'entities': entities,
                        'original_text': text
                    }

        # If no pattern matches, use NLP for more complex understanding
        return self.nlp_based_extraction(text)

    def nlp_based_extraction(self, text: str) -> Dict:
        """Use NLP to extract intent and entities for complex commands"""
        if not self.nlp:
            return {'intent': 'unknown', 'entities': [], 'original_text': text}

        doc = self.nlp(text)

        # Extract named entities
        entities = [(ent.text, ent.label_) for ent in doc.ents]

        # Extract verbs for actions
        verbs = [token.lemma_ for token in doc if token.pos_ == "VERB"]

        # Determine intent based on verbs and context
        intent = self.determine_intent_from_nlp(doc, verbs)

        return {
            'intent': intent,
            'entities': entities,
            'verbs': verbs,
            'original_text': text
        }

    def determine_intent_from_nlp(self, doc, verbs) -> str:
        """Determine intent based on NLP analysis"""
        # Define verb-to-intent mapping
        verb_to_intent = {
            'go': 'move',
            'move': 'move',
            'walk': 'move',
            'head': 'move',
            'pick': 'grasp',
            'grab': 'grasp',
            'take': 'grasp',
            'get': 'grasp',
            'lift': 'grasp',
            'put': 'place',
            'place': 'place',
            'drop': 'place',
            'follow': 'follow',
            'come': 'follow',
            'stop': 'stop',
            'freeze': 'stop',
            'tell': 'introduce',
            'introduce': 'introduce'
        }

        for verb in verbs:
            if verb in verb_to_intent:
                return verb_to_intent[verb]

        return 'unknown'
```

### Semantic Understanding with LLMs

For more sophisticated understanding, we can use large language models:

```python
import openai
from typing import Optional

class LLMCommandInterpreter:
    def __init__(self, api_key: Optional[str] = None):
        if api_key:
            openai.api_key = api_key
        self.client = openai.OpenAI(api_key=api_key) if api_key else None

    def interpret_command_with_llm(self, command: str, robot_capabilities: List[str] = None) -> Dict:
        """Use LLM to interpret natural language command"""
        if not self.client:
            return {"error": "OpenAI API key not provided"}

        capabilities = robot_capabilities or [
            "move to locations", "grasp objects", "place objects",
            "follow humans", "stop movement", "introduce itself"
        ]

        prompt = f"""
        You are a robot command interpreter. Given a natural language command,
        interpret it and return the structured intent and parameters.

        Robot capabilities: {', '.join(capabilities)}

        Command: "{command}"

        Please return a JSON object with:
        - intent: The main action (move, grasp, place, follow, stop, introduce, etc.)
        - parameters: Relevant parameters for the action
        - confidence: Your confidence level (0-1)

        Example:
        {{
            "intent": "move",
            "parameters": {{"location": "kitchen"}},
            "confidence": 0.9
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            import json
            result = json.loads(response.choices[0].message.content)
            return result

        except Exception as e:
            return {"error": str(e), "intent": "unknown", "parameters": {}}
```

## Action Planning and Execution

### ROS 2 Action Client Implementation

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String

class VoiceCommandExecutor(Node):
    def __init__(self):
        super().__init__('voice_command_executor')

        # Action clients for different robot capabilities
        self.move_action_client = ActionClient(self, MoveGroup, 'move_group')
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')

        # Publishers for other commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speech_publisher = self.create_publisher(String, 'tts_input', 10)

        # Command queue
        self.command_queue = []
        self.command_timer = self.create_timer(0.1, self.process_commands)

    def execute_command(self, intent: str, parameters: Dict):
        """Execute robot command based on intent and parameters"""
        if intent == 'move':
            self.execute_move_command(parameters)
        elif intent == 'grasp':
            self.execute_grasp_command(parameters)
        elif intent == 'place':
            self.execute_place_command(parameters)
        elif intent == 'follow':
            self.execute_follow_command(parameters)
        elif intent == 'stop':
            self.execute_stop_command()
        elif intent == 'introduce':
            self.execute_introduce_command()
        else:
            self.get_logger().info(f'Unknown command intent: {intent}')

    def execute_move_command(self, parameters: Dict):
        """Execute navigation/movement command"""
        target_location = parameters.get('location', 'unknown')

        # Convert location to coordinates (this would use a map/waypoint system)
        target_pose = self.get_pose_for_location(target_location)

        if target_pose:
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = 'arm'  # or 'base' for mobile base
            goal_msg.request.workspace_parameters.header.frame_id = 'map'
            goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()

            # Send action goal
            self.move_action_client.wait_for_server()
            future = self.move_action_client.send_goal_async(goal_msg)
            future.add_done_callback(self.move_goal_response_callback)
        else:
            self.speak_response(f"Sorry, I don't know where {target_location} is.")

    def execute_grasp_command(self, parameters: Dict):
        """Execute object grasping command"""
        target_object = parameters.get('object', 'unknown')

        # First, find the object using perception
        object_pose = self.find_object(target_object)

        if object_pose:
            # Plan grasp trajectory
            grasp_trajectory = self.plan_grasp_trajectory(object_pose)

            if grasp_trajectory:
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory = grasp_trajectory

                self.trajectory_action_client.wait_for_server()
                future = self.trajectory_action_client.send_goal_async(goal_msg)
                future.add_done_callback(self.trajectory_goal_response_callback)
        else:
            self.speak_response(f"Sorry, I couldn't find the {target_object}.")

    def execute_place_command(self, parameters: Dict):
        """Execute object placement command"""
        target_object = parameters.get('object', 'unknown')
        target_location = parameters.get('location', 'table')

        # Get placement location
        placement_pose = self.get_pose_for_location(target_location)

        if placement_pose:
            # Plan placement trajectory
            place_trajectory = self.plan_place_trajectory(placement_pose)

            if place_trajectory:
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory = place_trajectory

                self.trajectory_action_client.wait_for_server()
                future = self.trajectory_action_client.send_goal_async(goal_msg)
                future.add_done_callback(self.trajectory_goal_response_callback)
        else:
            self.speak_response(f"Sorry, I don't know where to place it.")

    def execute_follow_command(self, parameters: Dict):
        """Execute follow command"""
        # This would typically involve person following algorithms
        follow_msg = Twist()
        follow_msg.linear.x = 0.5  # Follow at 0.5 m/s
        follow_msg.angular.z = 0.0  # Will be controlled by person following algorithm

        # Publish follow command
        self.cmd_vel_publisher.publish(follow_msg)
        self.speak_response("I will follow you now.")

    def execute_stop_command(self):
        """Execute stop command"""
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.speak_response("I have stopped.")

    def execute_introduce_command(self):
        """Execute introduction command"""
        introduction = "Hello! I am a humanoid robot designed to assist with various tasks. I can move around, pick up objects, and understand voice commands. How can I help you today?"
        self.speak_response(introduction)

    def speak_response(self, text: str):
        """Publish text-to-speech response"""
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)

    def get_pose_for_location(self, location_name: str) -> Optional[PoseStamped]:
        """Get pose for named location (would use map/waypoint system)"""
        # This would typically look up predefined locations in a map
        location_map = {
            'kitchen': PoseStamped(),
            'living room': PoseStamped(),
            'bedroom': PoseStamped(),
            'table': PoseStamped(),
            # Add more locations as needed
        }

        return location_map.get(location_name.lower())

    def find_object(self, object_name: str) -> Optional[PoseStamped]:
        """Find object using perception system"""
        # This would integrate with object detection system
        # For now, return a placeholder
        return PoseStamped() if object_name else None

    def plan_grasp_trajectory(self, object_pose: PoseStamped) -> Optional[JointTrajectory]:
        """Plan trajectory to grasp object"""
        # This would use motion planning (MoveIt, etc.)
        # For now, return a placeholder
        return JointTrajectory()

    def plan_place_trajectory(self, placement_pose: PoseStamped) -> Optional[JointTrajectory]:
        """Plan trajectory to place object"""
        # This would use motion planning
        # For now, return a placeholder
        return JointTrajectory()

    def move_goal_response_callback(self, future):
        """Handle move action response"""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Move goal accepted')
            # Could add feedback handling here

    def trajectory_goal_response_callback(self, future):
        """Handle trajectory action response"""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Trajectory goal accepted')
            # Could add feedback handling here
```

## Complete Voice-to-Action System

### Main Voice Command System

```python
import asyncio
import threading
from dataclasses import dataclass
from typing import Optional

@dataclass
class VoiceCommand:
    text: str
    intent: str
    parameters: Dict
    confidence: float
    timestamp: float

class VoiceToActionSystem:
    def __init__(self, openai_api_key: Optional[str] = None):
        # Initialize components
        self.audio_capture = RealTimeAudioCapture()
        self.whisper_processor = RealTimeWhisperProcessor()
        self.nlp_processor = NaturalLanguageProcessor()
        self.llm_interpreter = LLMCommandInterpreter(openai_api_key)
        self.command_executor = VoiceCommandExecutor()

        # Command history
        self.command_history = []

        # Threading
        self.processing_thread = None
        self.is_running = False

    def start_system(self):
        """Start the complete voice-to-action system"""
        self.is_running = True

        # Start audio capture
        self.audio_capture.start_recording()

        # Start processing thread
        self.processing_thread = threading.Thread(
            target=self.process_audio_continuously,
            daemon=True
        )
        self.processing_thread.start()

        # Initialize ROS 2
        rclpy.init()
        self.command_executor_thread = threading.Thread(
            target=self.run_ros_node,
            daemon=True
        )
        self.command_executor_thread.start()

        print("Voice-to-Action system started. Listening for commands...")

    def process_audio_continuously(self):
        """Continuously process audio for voice commands"""
        while self.is_running and self.audio_capture.is_recording:
            try:
                # Get audio chunk
                audio_chunk = self.audio_capture.audio_queue.get(timeout=0.1)

                # Preprocess
                processed_audio = self.audio_capture.preprocessor.preprocess_audio(audio_chunk)

                # Check for speech activity
                if self.audio_capture.preprocessor.detect_speech_activity(processed_audio):
                    # Transcribe
                    transcription = self.whisper_processor.whisper_model.transcribe_audio(processed_audio)

                    if transcription.strip():
                        # Process the command
                        self.process_voice_command(transcription)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in audio processing: {e}")

    def process_voice_command(self, text: str):
        """Process a complete voice command through the pipeline"""
        print(f"Heard: {text}")

        # 1. Natural Language Processing
        nlp_result = self.nlp_processor.extract_intent_and_entities(text)

        # 2. LLM Enhancement (if available)
        llm_result = self.llm_interpreter.interpret_command_with_llm(text)

        # 3. Determine final intent and parameters
        if llm_result and 'error' not in llm_result:
            final_intent = llm_result.get('intent', nlp_result['intent'])
            final_parameters = llm_result.get('parameters', nlp_result.get('entities', {}))
            confidence = llm_result.get('confidence', 0.7)
        else:
            final_intent = nlp_result['intent']
            final_parameters = {'entities': nlp_result.get('entities', [])}
            confidence = 0.8  # Default confidence for rule-based parsing

        # 4. Create command object
        command = VoiceCommand(
            text=text,
            intent=final_intent,
            parameters=final_parameters,
            confidence=confidence,
            timestamp=time.time()
        )

        # 5. Add to history
        self.command_history.append(command)

        # 6. Execute command
        self.execute_command(command)

        # 7. Provide feedback
        self.provide_feedback(command)

    def execute_command(self, command: VoiceCommand):
        """Execute the interpreted command"""
        if command.confidence > 0.5:  # Only execute confident commands
            self.command_executor.execute_command(command.intent, command.parameters)
        else:
            self.command_executor.speak_response(
                f"I heard '{command.text}' but I'm not confident about the meaning. Could you repeat that?"
            )

    def provide_feedback(self, command: VoiceCommand):
        """Provide feedback about the command"""
        if command.confidence > 0.8:
            response = f"Okay, I will {command.intent} as you requested."
        elif command.confidence > 0.5:
            response = f"I think you want me to {command.intent}. Executing now."
        else:
            response = f"I heard '{command.text}' but I'm not sure what you mean."

        print(f"Response: {response}")
        # In a real system, this would be spoken back to the user

    def run_ros_node(self):
        """Run the ROS 2 command executor node"""
        rclpy.spin(self.command_executor)

    def stop_system(self):
        """Stop the voice-to-action system"""
        self.is_running = False

        # Stop audio capture
        self.audio_capture.stop_recording()

        # Shutdown ROS
        self.command_executor.destroy_node()
        rclpy.shutdown()

        print("Voice-to-Action system stopped.")

# Example usage
def main():
    # Initialize the system
    system = VoiceToActionSystem(openai_api_key="your-api-key-here")

    try:
        # Start the system
        system.start_system()

        # Keep running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down...")
        system.stop_system()

if __name__ == "__main__":
    main()
```

## Integration with Humanoid Robot Control

### Humanoid-Specific Considerations

For humanoid robots, voice command processing needs to consider the unique aspects of bipedal locomotion and human-like interaction:

```python
class HumanoidVoiceController:
    def __init__(self):
        self.balance_checker = BalanceChecker()
        self.gesture_generator = GestureGenerator()
        self.emotion_controller = EmotionController()

    def execute_humane_command(self, command: VoiceCommand):
        """Execute command with humanoid-specific considerations"""

        # Check if robot can safely execute command while maintaining balance
        if not self.balance_checker.can_execute_command(command):
            return self.handle_unsafe_command(command)

        # Add appropriate gestures for human-like interaction
        self.gesture_generator.perform_relevant_gesture(command.intent)

        # Show appropriate emotional response
        self.emotion_controller.show_response_emotion(command.intent)

        # Execute the command
        result = self.execute_command(command)

        # Return to neutral pose if needed
        self.return_to_neutral_pose()

        return result

    def handle_unsafe_command(self, command: VoiceCommand):
        """Handle commands that might compromise robot safety"""
        if command.intent in ['move', 'grasp'] and balance is compromised:
            self.speak_response("I cannot do that right now as it might cause me to fall. Let me adjust my stance first.")
            self.stabilize_robot()
            # Retry command or suggest alternative
```

## Error Handling and Robustness

### Handling Uncertainty and Errors

```python
class RobustVoiceCommandHandler:
    def __init__(self):
        self.confidence_threshold = 0.6
        self.max_retry_attempts = 3
        self.context_buffer = []  # Store recent context

    def handle_command_with_context(self, command: VoiceCommand):
        """Handle command considering context and uncertainty"""

        # Check confidence level
        if command.confidence < self.confidence_threshold:
            return self.request_clarification(command)

        # Consider context from previous commands
        contextual_command = self.apply_context(command)

        # Execute with error handling
        try:
            result = self.execute_command_with_retry(contextual_command)
            return result
        except CommandExecutionError as e:
            return self.handle_execution_error(command, e)

    def request_clarification(self, command: VoiceCommand):
        """Request clarification when confidence is low"""
        clarification_questions = {
            'move': f"Do you want me to go to the {command.parameters.get('location', 'specified location')}?",
            'grasp': f"Do you want me to pick up the {command.parameters.get('object', 'object')}?",
            'place': f"Do you want me to place {command.parameters.get('object', 'it')} on the {command.parameters.get('location', 'surface')}?"
        }

        intent = command.intent
        if intent in clarification_questions:
            self.speak_response(clarification_questions[intent] + " Please confirm with yes or no.")
        else:
            self.speak_response(f"I heard '{command.text}' but I'm not sure what you mean. Could you rephrase that?")

    def execute_command_with_retry(self, command: VoiceCommand, attempt=1):
        """Execute command with retry logic"""
        try:
            return self.execute_command(command)
        except CommandExecutionError as e:
            if attempt < self.max_retry_attempts:
                # Try alternative approach
                alternative_command = self.generate_alternative_command(command, e)
                return self.execute_command_with_retry(alternative_command, attempt + 1)
            else:
                raise e
```

## Performance Optimization

### Optimizing for Real-time Processing

```python
class OptimizedVoiceSystem:
    def __init__(self):
        self.vad_model = self.load_voice_activity_detection_model()
        self.command_cache = {}  # Cache frequent commands
        self.precomputed_locations = {}  # Precompute location poses

    def optimize_audio_processing(self):
        """Optimize audio processing for real-time performance"""
        # Use faster, less accurate model for initial detection
        self.fast_whisper = whisper.load_model("tiny")

        # Use full model only when needed
        self.full_whisper = None  # Load on demand

    def preprocess_commands(self):
        """Preprocess common commands for faster execution"""
        common_commands = [
            "stop", "go forward", "turn left", "turn right",
            "pick up object", "put down object", "hello"
        ]

        for cmd in common_commands:
            # Pre-process and cache
            processed = self.process_command_offline(cmd)
            self.command_cache[cmd.lower()] = processed
```

## Summary

Voice-to-action pipelines enable natural human-robot interaction through spoken language. In this chapter, we covered:

- Speech recognition fundamentals and Whisper integration
- Natural language understanding using both rule-based and LLM approaches
- Action planning and execution for robot commands
- Complete voice command system architecture
- Humanoid-specific considerations for voice interaction
- Error handling and robustness strategies
- Performance optimization for real-time processing

The integration of speech recognition, natural language processing, and robot control creates powerful interfaces that allow users to command robots using everyday language, making robotics more accessible and intuitive.

## Learning Outcomes

After completing this chapter, you should be able to:
- Implement speech recognition using Whisper for voice commands
- Parse natural language commands and extract intent and entities
- Connect voice commands to ROS 2 action sequences
- Build complete voice-to-action pipelines for humanoid robots
- Handle uncertainty and errors in voice command processing
- Optimize voice command systems for real-time performance

## References

1. Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." *arXiv preprint arXiv:2212.04356*.
2. Brown, T., et al. (2020). "Language Models are Few-Shot Learners." *Advances in Neural Information Processing Systems*, 33, 1877-1901.
3. ROS-Industrial Consortium. (2024). "Voice Command Interfaces for Industrial Robots." *Technical Report Series*.