---
title: Conversational Robotics Laboratory
sidebar_position: 4
description: Advanced laboratory exercises for developing and testing conversational interfaces in humanoid robotics
---

# Conversational Robotics Lab

This lab provides hands-on experience with implementing conversational interfaces for humanoid robots. You'll build systems that can understand natural language, manage dialogue, and generate appropriate responses for human-robot interaction.

## Lab Objectives

After completing this lab, you will be able to:
- Implement natural language understanding for robot commands
- Design dialogue management systems
- Create multimodal conversational interfaces
- Handle conversational errors and recovery
- Evaluate conversational system performance

## Exercise 1: Basic Command Parser

### Goal
Create a system that parses simple robot commands from natural language.

### Implementation
```python
import re
from typing import Dict, List, Optional, Tuple

class CommandParser:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'move': [
                r'go to (.+)',
                r'move to (.+)',
                r'navigate to (.+)',
                r'go (.+)',
                r'walk to (.+)',
                r'move (.+)'
            ],
            'grasp': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'get (.+)',
                r'take (.+)',
                r'grab (.+)'
            ],
            'release': [
                r'release (.+)',
                r'drop (.+)',
                r'let go of (.+)',
                r'put down (.+)'
            ],
            'look_at': [
                r'look at (.+)',
                r'watch (.+)',
                r'focus on (.+)',
                r'pay attention to (.+)'
            ],
            'stop': [
                r'stop',
                r'halt',
                r'pause',
                r'freeze'
            ],
            'help': [
                r'help',
                r'what can you do',
                r'how do i use you',
                r'what commands'
            ],
            'greeting': [
                r'hello',
                r'hi',
                r'hey',
                r'good morning',
                r'good afternoon',
                r'good evening'
            ],
            'farewell': [
                r'goodbye',
                r'bye',
                r'see you',
                r'thanks',
                r'thank you'
            ]
        }

        # Location keywords
        self.locations = {
            'kitchen': ['kitchen', 'cooking', 'stove', 'fridge', 'refrigerator', 'sink'],
            'living_room': ['living room', 'couch', 'sofa', 'tv', 'television', 'coffee table'],
            'bedroom': ['bedroom', 'bed', 'sleep', 'wardrobe', 'dresser'],
            'office': ['office', 'desk', 'computer', 'study', 'work'],
            'bathroom': ['bathroom', 'bath', 'shower', 'toilet', 'sink'],
            'dining_room': ['dining room', 'dinner', 'table', 'eat', 'food']
        }

        # Object categories
        self.objects = {
            'graspable': ['cup', 'bottle', 'book', 'phone', 'keys', 'plate', 'fork', 'spoon', 'glass'],
            'furniture': ['table', 'chair', 'couch', 'sofa', 'desk', 'bed', 'cabinet', 'shelf'],
            'appliances': ['refrigerator', 'microwave', 'oven', 'tv', 'computer', 'fan']
        }

    def preprocess_text(self, text: str) -> str:
        """Clean and normalize input text"""
        # Convert to lowercase
        text = text.lower().strip()

        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)

        # Remove punctuation except for question marks
        text = re.sub(r'[^\w\s?]', ' ', text)

        return text

    def extract_command(self, text: str) -> Tuple[str, Dict]:
        """Extract command and parameters from text"""
        original_text = text
        text = self.preprocess_text(text)

        # Check for each command pattern
        for command_type, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    # Extract parameters based on command type
                    params = {}

                    if command_type in ['move', 'look_at']:
                        # Try to identify location
                        location_text = match.group(1).strip()
                        params['target'] = self.identify_location(location_text)

                    elif command_type in ['grasp', 'release']:
                        # Identify object to manipulate
                        object_text = match.group(1).strip()
                        params['object'] = self.identify_object(object_text)

                    return command_type, params

        # If no pattern matches, return 'unknown'
        return 'unknown', {'original': original_text}

    def identify_location(self, location_text: str) -> str:
        """Identify the intended location from text"""
        location_text = self.preprocess_text(location_text)

        # Check for exact matches first
        for location, keywords in self.locations.items():
            if any(keyword in location_text for keyword in keywords):
                return location

        # If no exact match, return the input text as location
        return location_text.strip()

    def identify_object(self, object_text: str) -> str:
        """Identify the intended object from text"""
        object_text = self.preprocess_text(object_text)

        # Check for exact matches
        for category, objects in self.objects.items():
            if any(obj in object_text for obj in objects):
                # Return the first matched object
                for obj in objects:
                    if obj in object_text:
                        return obj

        # If no exact match, return the input text as object
        return object_text.strip()

    def parse_command_batch(self, commands: List[str]) -> List[Dict]:
        """Parse multiple commands"""
        results = []
        for cmd in commands:
            command_type, params = self.extract_command(cmd)
            results.append({
                'original': cmd,
                'command_type': command_type,
                'parameters': params
            })
        return results

# Test the command parser
parser = CommandParser()

# Test commands
test_commands = [
    "Go to the kitchen",
    "Please pick up the red cup",
    "Look at the television",
    "Stop moving",
    "What can you do?",
    "Hello there!",
    "Navigate to the living room",
    "Grasp the book on the table",
    "Goodbye, thanks for your help"
]

print("Command Parser Test Results:")
print("="*50)
for cmd in test_commands:
    command_type, params = parser.extract_command(cmd)
    print(f"Input: '{cmd}'")
    print(f"  Command: {command_type}")
    if params:
        print(f"  Parameters: {params}")
    print()
```

## Exercise 2: Dialogue State Management

### Goal
Create a dialogue manager that maintains conversation context and handles multi-turn interactions.

### Implementation
```python
from datetime import datetime
from enum import Enum
import json

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    REPLYING = "replying"
    WAITING_CONFIRMATION = "waiting_confirmation"

class DialogueManager:
    def __init__(self):
        self.state = DialogueState.IDLE
        self.context = {}
        self.history = []
        self.waiting_for_confirmation = False
        self.confirmation_request = None

        # Task queue for complex operations
        self.task_queue = []
        self.current_task = None

    def update_context(self, key: str, value: any):
        """Update the dialogue context"""
        self.context[key] = value

    def get_context(self, key: str, default=None):
        """Get value from dialogue context"""
        return self.context.get(key, default)

    def add_to_history(self, speaker: str, text: str, metadata: Dict = None):
        """Add utterance to conversation history"""
        entry = {
            'timestamp': datetime.now().isoformat(),
            'speaker': speaker,
            'text': text,
            'metadata': metadata or {}
        }
        self.history.append(entry)

        # Keep only last 20 exchanges
        if len(self.history) > 20:
            self.history = self.history[-20:]

    def get_recent_context(self, num_exchanges: int = 3) -> List[Dict]:
        """Get recent conversation context"""
        return self.history[-num_exchanges:] if len(self.history) >= num_exchanges else self.history

    def handle_user_input(self, user_input: str) -> Dict:
        """Handle user input and return response"""
        self.add_to_history('user', user_input)

        # Update state
        self.state = DialogueState.PROCESSING

        # Parse the input
        parser = CommandParser()
        command_type, params = parser.extract_command(user_input)

        # Generate response based on command
        response = self.generate_response(command_type, params, user_input)

        # Add response to history
        self.add_to_history('robot', response.get('text', ''), response)

        # Update state
        self.state = DialogueState.REPLYING

        return response

    def generate_response(self, command_type: str, params: Dict, original_input: str) -> Dict:
        """Generate appropriate response based on command"""
        import random

        responses = {
            'greeting': [
                "Hello! How can I assist you today?",
                "Hi there! What can I help you with?",
                "Good day! I'm ready to help you."
            ],
            'move': [
                "I'll head to the {target} now.",
                "On my way to the {target}.",
                "Navigating to {target} for you."
            ],
            'grasp': [
                "I'll pick up the {object} for you.",
                "Retrieving the {object}.",
                "Getting the {object} now."
            ],
            'release': [
                "I've released the {object}.",
                "Putting down the {object}.",
                "Object {object} has been released."
            ],
            'look_at': [
                "I'm looking at {target}.",
                "Focusing on {target}.",
                "I can see {target}."
            ],
            'stop': [
                "I've stopped.",
                "Movement halted.",
                "All actions paused."
            ],
            'help': [
                "I can help you with moving, picking up objects, looking at things, and more. Try saying 'go to kitchen' or 'pick up the cup'.",
                "I can navigate to locations, grasp objects, and respond to commands. Say hello, help, or give me a task.",
                "I'm here to assist with various tasks. You can ask me to move somewhere, pick up items, or look at things."
            ],
            'farewell': [
                "Goodbye! Feel free to call me if you need anything.",
                "See you later!",
                "Have a great day!"
            ],
            'unknown': [
                "I'm not sure I understood that. Could you rephrase?",
                "I didn't catch that. Could you try again?",
                "I'm not certain what you mean. Can you clarify?"
            ]
        }

        # Select appropriate response template
        if command_type in responses:
            if command_type in ['move', 'grasp', 'release', 'look_at'] and params:
                # Use template with parameters
                template = random.choice(responses[command_type])
                if 'target' in params:
                    response_text = template.format(target=params['target'])
                elif 'object' in params:
                    response_text = template.format(object=params['object'])
                else:
                    response_text = template
            else:
                response_text = random.choice(responses[command_type])
        else:
            response_text = random.choice(responses['unknown'])

        return {
            'text': response_text,
            'command_type': command_type,
            'parameters': params,
            'state': self.state.value,
            'timestamp': datetime.now().isoformat()
        }

    def request_confirmation(self, action_description: str) -> Dict:
        """Request confirmation for an action"""
        self.waiting_for_confirmation = True
        self.confirmation_request = action_description

        return {
            'text': f"Do you want me to {action_description}? Please confirm with yes or no.",
            'requires_confirmation': True,
            'action_description': action_description
        }

    def process_confirmation(self, user_response: str) -> Dict:
        """Process user confirmation response"""
        user_response = user_response.lower().strip()

        if any(word in user_response for word in ['yes', 'y', 'sure', 'ok', 'okay', 'affirmative']):
            self.waiting_for_confirmation = False
            self.confirmation_request = None

            return {
                'text': "Action confirmed. Proceeding with the task.",
                'confirmed': True
            }
        elif any(word in user_response for word in ['no', 'n', 'cancel', 'negative', 'stop']):
            self.waiting_for_confirmation = False
            self.confirmation_request = None

            return {
                'text': "Action canceled. What else can I help you with?",
                'confirmed': False
            }
        else:
            # Unclear response, ask again
            return {
                'text': "I'm not sure if that's a yes or no. Please confirm with 'yes' or 'no'.",
                'requires_confirmation': True
            }

    def get_state_summary(self) -> Dict:
        """Get a summary of the current dialogue state"""
        return {
            'current_state': self.state.value,
            'context_keys': list(self.context.keys()),
            'history_length': len(self.history),
            'waiting_confirmation': self.waiting_for_confirmation,
            'current_task': self.current_task
        }

# Test the dialogue manager
dialogue_manager = DialogueManager()

print("Dialogue Manager Test:")
print("="*50)

# Simulate a conversation
conversation_flow = [
    "Hello!",
    "Can you help me?",
    "Please go to the kitchen",
    "Pick up the cup",
    "Look at the TV",
    "What can you do?",
    "Goodbye"
]

for user_input in conversation_flow:
    print(f"User: {user_input}")

    # Check if waiting for confirmation
    if dialogue_manager.waiting_for_confirmation:
        response = dialogue_manager.process_confirmation(user_input)
    else:
        response = dialogue_manager.handle_user_input(user_input)

    print(f"Robot: {response['text']}")
    print(f"  Command type: {response.get('command_type', 'N/A')}")
    print(f"  State: {dialogue_manager.state.value}")
    print()
```

## Exercise 3: Multimodal Interaction System

### Goal
Create a system that combines different communication modalities (speech, gestures, visual feedback) for richer human-robot interaction.

### Implementation
```python
import asyncio
import time
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class SpeechRecognitionResult:
    text: str
    confidence: float
    timestamp: float

@dataclass
class GestureRecognitionResult:
    gesture_type: str
    confidence: float
    timestamp: float

@dataclass
class VisualFeedback:
    type: str  # 'status', 'direction', 'attention', 'emotion'
    color: str
    pattern: str  # 'steady', 'pulsing', 'flashing', 'chasing'
    duration: float
    intensity: float

class MultimodalInteractionSystem:
    def __init__(self):
        self.dialogue_manager = DialogueManager()
        self.last_speech = None
        self.last_gesture = None
        self.active_visual_feedback = None

        # Communication channels
        self.speech_recognition_active = True
        self.gesture_recognition_active = True
        self.visual_feedback_active = True

    def integrate_modalities(self, speech_result: Optional[SpeechRecognitionResult] = None,
                           gesture_result: Optional[GestureRecognitionResult] = None) -> Dict:
        """Integrate information from multiple modalities"""
        fusion_result = {
            'primary_modality': None,
            'interpretation': '',
            'confidence': 0.0,
            'multimodal_response': {}
        }

        # Determine primary modality based on confidence and recency
        speech_confidence = speech_result.confidence if speech_result else 0.0
        gesture_confidence = gesture_result.confidence if gesture_result else 0.0

        if speech_confidence >= gesture_confidence and speech_result:
            # Speech is primary modality
            fusion_result['primary_modality'] = 'speech'

            # Process speech command
            parser = CommandParser()
            command_type, params = parser.extract_command(speech_result.text)

            # Generate response using dialogue manager
            response = self.dialogue_manager.generate_response(command_type, params, speech_result.text)

            # Augment with gesture information if available
            if gesture_result and gesture_confidence > 0.7:
                response['gesture_context'] = gesture_result.gesture_type
                response['gesture_confidence'] = gesture_result.confidence

            fusion_result['interpretation'] = response.get('text', '')
            fusion_result['confidence'] = speech_confidence

        elif gesture_confidence > speech_confidence and gesture_result:
            # Gesture is primary modality
            fusion_result['primary_modality'] = 'gesture'

            # Map gesture to verbal response
            gesture_responses = {
                'wave': "Hello! How can I help you?",
                'point': "I see you're pointing. Is there something specific you'd like me to do?",
                'thumbs_up': "Thank you for the positive feedback!",
                'stop': "I understand. I will stop my current action."
            }

            response_text = gesture_responses.get(gesture_result.gesture_type,
                                               f"I noticed you made a {gesture_result.gesture_type} gesture.")

            fusion_result['interpretation'] = response_text
            fusion_result['confidence'] = gesture_confidence

        else:
            # No clear primary input
            fusion_result['primary_modality'] = 'none'
            fusion_result['interpretation'] = "I'm ready to interact. Please speak or use gestures to communicate."
            fusion_result['confidence'] = 0.0

        # Generate multimodal response
        fusion_result['multimodal_response'] = self.generate_multimodal_response(
            fusion_result['primary_modality'],
            fusion_result['interpretation']
        )

        # Update internal state
        if speech_result:
            self.last_speech = speech_result
        if gesture_result:
            self.last_gesture = gesture_result

        return fusion_result

    def generate_multimodal_response(self, modality: str, text: str) -> Dict:
        """Generate coordinated multimodal response"""
        response = {}

        # Speech response
        response['speech'] = {
            'text': text,
            'should_speak': True,
            'voice_properties': {
                'speed': 1.0,
                'pitch': 1.0,
                'volume': 0.8
            }
        }

        # Visual feedback based on interaction
        visual_feedback = self.generate_visual_feedback(modality)
        response['visual'] = visual_feedback

        # Potential gesture response
        gesture_response = self.map_text_to_gesture(text)
        response['gesture'] = {
            'perform_gesture': gesture_response is not None,
            'gesture_type': gesture_response,
            'intensity': 0.7
        }

        return response

    def generate_visual_feedback(self, modality: str) -> VisualFeedback:
        """Generate appropriate visual feedback"""
        if modality == 'speech':
            feedback_type = 'attention'
            color = 'blue'
            pattern = 'pulsing'
            duration = 2.0
            intensity = 0.8
        elif modality == 'gesture':
            feedback_type = 'recognition'
            color = 'green'
            pattern = 'chasing'
            duration = 1.5
            intensity = 0.9
        else:
            feedback_type = 'idle'
            color = 'white'
            pattern = 'steady'
            duration = 3.0
            intensity = 0.3

        return VisualFeedback(
            type=feedback_type,
            color=color,
            pattern=pattern,
            duration=duration,
            intensity=intensity
        )

    def map_text_to_gesture(self, text: str) -> Optional[str]:
        """Map text content to appropriate gesture"""
        text_lower = text.lower()

        if any(word in text_lower for word in ['hello', 'hi', 'greeting']):
            return 'wave'
        elif any(word in text_lower for word in ['yes', 'agree', 'confirm']):
            return 'thumbs_up'
        elif any(word in text_lower for word in ['no', 'disagree', 'stop']):
            return 'stop'
        elif any(word in text_lower for word in ['please', 'thank']):
            return 'nod'
        elif any(word in text_lower for word in ['look', 'watch', 'see']):
            return 'point'
        else:
            return None

    def simulate_user_interaction(self) -> Dict:
        """Simulate a user interaction with multiple modalities"""
        # Simulate speech input
        speech_text = "Please go to the kitchen and pick up the cup"
        speech_result = SpeechRecognitionResult(
            text=speech_text,
            confidence=0.85,
            timestamp=time.time()
        )

        # Simulate gesture input (optional)
        gesture_result = GestureRecognitionResult(
            gesture_type='point',
            confidence=0.78,
            timestamp=time.time()
        )

        # Integrate modalities
        fusion_result = self.integrate_modalities(speech_result, gesture_result)

        return fusion_result

# Test the multimodal system
multimodal_system = MultimodalInteractionSystem()

print("Multimodal Interaction System Test:")
print("="*50)

# Simulate interaction
result = multimodal_system.simulate_user_interaction()

print(f"Primary modality: {result['primary_modality']}")
print(f"Interpretation: {result['interpretation']}")
print(f"Confidence: {result['confidence']:.2f}")
print("\nMultimodal Response:")
print(f"  Speech: {result['multimodal_response']['speech']['text']}")
print(f"  Visual: {result['multimodal_response']['visual'].type} - {result['multimodal_response']['visual'].color}")
print(f"  Gesture: {result['multimodal_response']['gesture']['gesture_type']}")
print()
```

## Exercise 4: Conversational Error Handling and Recovery

### Goal
Implement error detection, handling, and recovery mechanisms for conversational systems.

### Implementation
```python
from enum import Enum
import random

class ErrorType(Enum):
    UNDERSTANDING_ERROR = "understanding_error"
    AMBIGUITY_ERROR = "ambiguity_error"
    CONTEXT_ERROR = "context_error"
    EXECUTION_ERROR = "execution_error"
    COMMUNICATION_ERROR = "communication_error"

class ErrorRecoverySystem:
    def __init__(self):
        self.error_history = []
        self.recovery_strategies = {
            ErrorType.UNDERSTANDING_ERROR: [
                'ask_for_clarification',
                'rephrase_request',
                'suggest_alternatives'
            ],
            ErrorType.AMBIGUITY_ERROR: [
                'request_specifics',
                'list_options',
                'context_aware_disambiguation'
            ],
            ErrorType.CONTEXT_ERROR: [
                'reset_context',
                'ask_for_continuation',
                'summarize_previous_interaction'
            ],
            ErrorType.EXECUTION_ERROR: [
                'retry_action',
                'use_alternative_method',
                'report_unavailability'
            ],
            ErrorType.COMMUNICATION_ERROR: [
                'switch_modality',
                'request_repeat',
                'use_fallback_response'
            ]
        }

    def detect_error(self, user_input: str, system_response: str, context: Dict) -> Optional[ErrorType]:
        """Detect potential errors in the interaction"""
        user_input_lower = user_input.lower()

        # Check for understanding errors
        if not system_response or 'not sure' in system_response.lower() or 'understand' in system_response.lower():
            return ErrorType.UNDERSTANDING_ERROR

        # Check for ambiguity
        if '?' in user_input and len(user_input.split()) <= 3:
            # Short ambiguous questions like "Go where?" or "Pick what?"
            return ErrorType.AMBIGUITY_ERROR

        # Check for context issues
        if context.get('previous_error', False):
            return ErrorType.CONTEXT_ERROR

        # Check for execution issues based on user follow-up
        if any(word in user_input_lower for word in ['repeat', 'try again', 'redo', 'restart']):
            return ErrorType.EXECUTION_ERROR

        # Check for communication issues
        if any(phrase in user_input_lower for phrase in ['can you repeat', 'i didn\'t hear', 'speak louder', 'again']):
            return ErrorType.COMMUNICATION_ERROR

        return None

    def generate_recovery_response(self, error_type: ErrorType, original_input: str, context: Dict) -> Dict:
        """Generate appropriate recovery response"""
        recovery_responses = {
            ErrorType.UNDERSTANDING_ERROR: [
                f"I didn't quite understand '{original_input}'. Could you rephrase that?",
                f"I'm not sure what you mean by '{original_input}'. Can you say it differently?",
                "I couldn't process that command. What would you like me to do?"
            ],
            ErrorType.AMBIGUITY_ERROR: [
                f"You said '{original_input}', but I need more specifics. Can you be more detailed?",
                "That's a bit vague. Could you specify what you want me to do?",
                "I need more information. What exactly are you asking for?"
            ],
            ErrorType.CONTEXT_ERROR: [
                "It seems we got off track. How can I help you?",
                "Let me help you continue from where we left off.",
                "I may have missed something. What were we talking about?"
            ],
            ErrorType.EXECUTION_ERROR: [
                "I had trouble with the last command. Would you like me to try again?",
                "I couldn't complete that task. What should I do instead?",
                "There was an issue with the previous request. How can I help?"
            ],
            ErrorType.COMMUNICATION_ERROR: [
                "Sorry, I didn't catch that. Could you repeat it?",
                "I missed what you said. Can you say it again?",
                "Could you repeat that? I didn't hear it clearly."
            ]
        }

        suggested_responses = recovery_responses.get(error_type, ["I'm not sure how to help with that."])
        selected_response = random.choice(suggested_responses)

        # Select appropriate recovery strategy
        strategies = self.recovery_strategies.get(error_type, [])
        selected_strategy = random.choice(strategies) if strategies else 'fallback_strategy'

        return {
            'response': selected_response,
            'error_type': error_type.value,
            'recovery_strategy': selected_strategy,
            'needs_clarification': error_type in [ErrorType.AMBIGUITY_ERROR, ErrorType.UNDERSTANDING_ERROR]
        }

    def handle_error_interaction(self, user_input: str, current_context: Dict) -> Dict:
        """Handle an interaction that may contain errors"""
        # Simulate processing that might have issues
        # For this example, we'll artificially introduce some errors

        import random
        should_introduce_error = random.random() < 0.3  # 30% chance of error

        if should_introduce_error:
            # Artificially create an error situation
            error_type = random.choice(list(ErrorType))
            recovery = self.generate_recovery_response(error_type, user_input, current_context)

            self.error_history.append({
                'input': user_input,
                'error_type': error_type.value,
                'recovery_strategy': recovery['recovery_strategy'],
                'timestamp': time.time()
            })

            return recovery

        else:
            # No error, return normal processing
            return {
                'response': f"I understood your request: '{user_input}'",
                'error_type': None,
                'recovery_strategy': 'none',
                'needs_clarification': False
            }

    def simulate_conversation_with_errors(self, dialog_manager: DialogueManager) -> List[Dict]:
        """Simulate a conversation with potential errors and recoveries"""
        conversation_scenarios = [
            "I want you to go somewhere",  # Ambiguous
            "Pick up the thing",          # Ambiguous
            "Move to kitchen",            # Clear
            "Do that thing you do",       # Vague
            "Can you help me",            # Clear
            "Repeat the last action",     # Context-dependent
            "What was that"              # Communication issue
        ]

        conversation_log = []

        for input_text in conversation_scenarios:
            # Process with dialogue manager
            normal_response = dialog_manager.generate_response('unknown', {}, input_text)

            # Check for errors
            error_response = self.handle_error_interaction(input_text, {})

            if error_response['error_type']:
                conversation_log.append({
                    'user_input': input_text,
                    'system_response': error_response['response'],
                    'error_detected': True,
                    'error_type': error_response['error_type'],
                    'recovery_strategy': error_response['recovery_strategy']
                })
            else:
                conversation_log.append({
                    'user_input': input_text,
                    'system_response': normal_response['text'],
                    'error_detected': False,
                    'error_type': None,
                    'recovery_strategy': 'none'
                })

        return conversation_log

# Test the error recovery system
error_system = ErrorRecoverySystem()

print("Error Recovery System Test:")
print("="*50)

# Simulate conversation with potential errors
dialogue_mgr = DialogueManager()  # Reuse from previous exercise
conversation_log = error_system.simulate_conversation_with_errors(dialogue_mgr)

for i, exchange in enumerate(conversation_log):
    print(f"Exchange {i+1}:")
    print(f"  User: {exchange['user_input']}")
    print(f"  System: {exchange['system_response']}")
    print(f"  Error detected: {exchange['error_detected']}")
    if exchange['error_detected']:
        print(f"  Error type: {exchange['error_type']}")
        print(f"  Recovery strategy: {exchange['recovery_strategy']}")
    print()

print(f"Total errors detected in simulation: {sum(1 for ex in conversation_log if ex['error_detected'])}")
```

## Exercise 5: Evaluation and Testing Framework

### Goal
Create an evaluation framework to test and measure conversational system performance.

### Implementation
```python
import time
import statistics
from dataclasses import dataclass
from typing import List, Dict, Callable

@dataclass
class InteractionMetrics:
    response_time: float
    understanding_accuracy: float
    user_satisfaction: float
    task_completion_rate: float
    error_recovery_success: float
    conversation_fluency: float

class ConversationalEvaluator:
    def __init__(self):
        self.metrics_history = []
        self.interaction_count = 0
        self.start_time = time.time()

    def measure_response_time(self, process_func: Callable, *args, **kwargs) -> float:
        """Measure the time taken by a function"""
        start = time.time()
        result = process_func(*args, **kwargs)
        end = time.time()
        return end - start, result

    def evaluate_understanding_accuracy(self, test_pairs: List[tuple]) -> float:
        """Evaluate how accurately the system understands inputs"""
        correct = 0
        total = len(test_pairs)

        for input_text, expected_command in test_pairs:
            parser = CommandParser()
            detected_command, _ = parser.extract_command(input_text)

            if detected_command == expected_command:
                correct += 1

        return correct / total if total > 0 else 0.0

    def evaluate_task_completion(self, task_list: List[Dict]) -> float:
        """Evaluate task completion success rate"""
        completed = 0
        total = len(task_list)

        for task in task_list:
            # Simulate task completion
            success_probability = task.get('success_rate', 0.8)  # Default 80%
            if random.random() < success_probability:
                completed += 1

        return completed / total if total > 0 else 0.0

    def evaluate_conversation_fluency(self, conversation_log: List[Dict]) -> float:
        """Evaluate how fluent the conversation appears to be"""
        if len(conversation_log) < 2:
            return 0.5  # Neutral if not enough data

        # Measure turn-taking smoothness
        smooth_exchanges = 0
        total_exchanges = len(conversation_log) - 1

        for i in range(total_exchanges):
            current = conversation_log[i]
            next_utterance = conversation_log[i + 1]

            # Check if responses are timely and relevant
            if not any(error_word in next_utterance.get('system_response', '').lower()
                      for error_word in ['not sure', 'don\'t understand', 'repeat']):
                smooth_exchanges += 1

        return smooth_exchanges / total_exchanges if total_exchanges > 0 else 0.0

    def run_comprehensive_evaluation(self, system_components: Dict) -> InteractionMetrics:
        """Run comprehensive evaluation of the conversational system"""
        start_eval = time.time()

        # Test understanding accuracy
        test_cases = [
            ("Go to the kitchen", "move"),
            ("Pick up the red cup", "grasp"),
            ("Hello there", "greeting"),
            ("Stop moving", "stop"),
            ("What can you do?", "help")
        ]

        understanding_acc = self.evaluate_understanding_accuracy(test_cases)

        # Test response time
        def dummy_process(text):
            time.sleep(random.uniform(0.1, 0.5))  # Simulate processing
            return "Processed: " + text

        response_times = []
        for test_input, _ in test_cases:
            resp_time, _ = self.measure_response_time(dummy_process, test_input)
            response_times.append(resp_time)

        avg_response_time = statistics.mean(response_times) if response_times else 0.0

        # Simulate user satisfaction (would be from real users in practice)
        user_satisfaction = random.uniform(0.6, 0.9)  # 60-90% satisfaction

        # Test task completion
        tasks = [
            {'name': 'navigation', 'success_rate': 0.85},
            {'name': 'manipulation', 'success_rate': 0.75},
            {'name': 'greeting', 'success_rate': 0.95}
        ]
        task_completion_rate = self.evaluate_task_completion(tasks)

        # Simulate error recovery (would be measured in real system)
        error_recovery_success = random.uniform(0.7, 0.95)

        # Create a sample conversation for fluency evaluation
        sample_conversation = [
            {'user_input': 'Hello', 'system_response': 'Hi there!'},
            {'user_input': 'Go to kitchen', 'system_response': 'Going to kitchen'},
            {'user_input': 'Pick cup', 'system_response': 'Picking up cup'}
        ]
        conversation_fluency = self.evaluate_conversation_fluency(sample_conversation)

        # Create metrics object
        metrics = InteractionMetrics(
            response_time=avg_response_time,
            understanding_accuracy=understanding_acc,
            user_satisfaction=user_satisfaction,
            task_completion_rate=task_completion_rate,
            error_recovery_success=error_recovery_success,
            conversation_fluency=conversation_fluency
        )

        self.metrics_history.append({
            'timestamp': time.time(),
            'metrics': metrics,
            'test_duration': time.time() - start_eval
        })

        return metrics

    def generate_evaluation_report(self) -> Dict:
        """Generate a comprehensive evaluation report"""
        if not self.metrics_history:
            return {'error': 'No evaluation data available'}

        recent_metrics = self.metrics_history[-1]['metrics']

        report = {
            'summary': {
                'total_interactions': len(self.metrics_history),
                'average_response_time': statistics.mean([m['metrics'].response_time for m in self.metrics_history]),
                'average_understanding_accuracy': statistics.mean([m['metrics'].understanding_accuracy for m in self.metrics_history]),
                'average_user_satisfaction': statistics.mean([m['metrics'].user_satisfaction for m in self.metrics_history]),
                'average_task_completion_rate': statistics.mean([m['metrics'].task_completion_rate for m in self.metrics_history])
            },
            'latest_metrics': {
                'response_time': recent_metrics.response_time,
                'understanding_accuracy': recent_metrics.understanding_accuracy,
                'user_satisfaction': recent_metrics.user_satisfaction,
                'task_completion_rate': recent_metrics.task_completion_rate,
                'error_recovery_success': recent_metrics.error_recovery_success,
                'conversation_fluency': recent_metrics.conversation_fluency
            },
            'recommendations': self._generate_recommendations(recent_metrics)
        }

        return report

    def _generate_recommendations(self, metrics: InteractionMetrics) -> List[str]:
        """Generate recommendations based on metrics"""
        recommendations = []

        if metrics.response_time > 1.0:
            recommendations.append("Response time is high (>1s). Consider optimizing processing or using caching.")

        if metrics.understanding_accuracy < 0.8:
            recommendations.append("Understanding accuracy is low (<80%). Improve NLP models or add more training data.")

        if metrics.user_satisfaction < 0.7:
            recommendations.append("User satisfaction is low. Consider improving response quality or adding personality.")

        if metrics.task_completion_rate < 0.8:
            recommendations.append("Task completion rate is low. Investigate action execution issues.")

        if metrics.conversation_fluency < 0.7:
            recommendations.append("Conversation fluency needs improvement. Work on context management and turn-taking.")

        if not recommendations:
            recommendations.append("System is performing well across all metrics!")

        return recommendations

# Test the evaluation framework
evaluator = ConversationalEvaluator()

print("Conversational System Evaluation:")
print("="*50)

# Run evaluation
system_components = {
    'parser': CommandParser(),
    'dialogue_manager': DialogueManager(),
    'multimodal_system': MultimodalInteractionSystem(),
    'error_system': ErrorRecoverySystem()
}

metrics = evaluator.run_comprehensive_evaluation(system_components)

print("Evaluation Metrics:")
print(f"  Response time: {metrics.response_time:.3f}s")
print(f"  Understanding accuracy: {metrics.understanding_accuracy:.2f}")
print(f"  User satisfaction: {metrics.user_satisfaction:.2f}")
print(f"  Task completion rate: {metrics.task_completion_rate:.2f}")
print(f"  Error recovery success: {metrics.error_recovery_success:.2f}")
print(f"  Conversation fluency: {metrics.conversation_fluency:.2f}")

# Generate report
report = evaluator.generate_evaluation_report()
print(f"\nRecommendations: {report['recommendations']}")
```

## Exercise Challenges

### Challenge 1: Context-Aware Dialogue
Enhance the dialogue manager to maintain context across multiple turns, remembering user preferences, task states, and conversation history.

### Challenge 2: Robust Error Handling
Implement more sophisticated error detection and recovery, including graceful degradation and user-friendly error messages.

### Challenge 3: Personality Integration
Add personality traits to the robot's communication style that can be customized for different user preferences or applications.

### Challenge 4: Multilingual Support
Extend the system to handle multiple languages and cultural communication differences.

## Assessment Questions

1. How does your system handle ambiguous user requests?
2. What strategies did you implement for maintaining conversation context?
3. How does your system provide feedback during different interaction phases?
4. What metrics would you use to evaluate conversational quality?
5. How does your system recover from misunderstandings or errors?

## Summary

This lab has provided hands-on experience with implementing conversational interfaces for humanoid robots. You've built systems for natural language understanding, dialogue management, multimodal interaction, error handling, and evaluation. These skills are essential for creating robots that can engage in natural, effective communication with humans.