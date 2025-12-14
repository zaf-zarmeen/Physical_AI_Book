# Vision-Language-Action (VLA) Examples

This directory contains examples related to Vision-Language-Action systems as described in the textbook.

## Prerequisites

Before running these examples, ensure you have:

1. OpenAI API key for Whisper and GPT usage
2. Required Python packages:
   ```bash
   pip install openai torch torchaudio transformers
   pip install speechrecognition pyaudio sounddevice
   pip install opencv-python numpy
   pip install rclpy  # if using with ROS 2
   ```

3. Set up your OpenAI API key in environment:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

## Speech Recognition with Whisper

Here's an example of using Whisper for speech recognition:

```python
# speech_recognition.py

import openai
import os
import pyaudio
import wave
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
openai.api_key = os.getenv('OPENAI_API_KEY')

class WhisperRecognizer:
    def __init__(self):
        # Audio recording parameters
        self.chunk = 1024  # Record in chunks of 1024 samples
        self.format = pyaudio.paInt16  # 16 bits per sample
        self.channels = 1  # Mono
        self.rate = 44100  # 44.1kHz
        self.record_seconds = 5
        self.output_filename = "temp_audio.wav"
        
    def record_audio(self, duration=None):
        """Record audio from microphone"""
        if duration is None:
            duration = self.record_seconds
            
        p = pyaudio.PyAudio()
        
        print("Recording audio...")
        
        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        frames = []
        
        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)
        
        print("Finished recording")
        
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Save the recorded data as a WAV file
        wf = wave.open(self.output_filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        return self.output_filename
    
    def transcribe_audio(self, audio_file):
        """Transcribe audio using OpenAI Whisper API"""
        with open(audio_file, "rb") as audio:
            response = openai.Audio.transcribe(
                model="whisper-1",
                file=audio
            )
        return response.text

def main():
    recognizer = WhisperRecognizer()
    
    try:
        # Record audio
        audio_file = recognizer.record_audio(duration=5)
        
        # Transcribe audio
        transcription = recognizer.transcribe_audio(audio_file)
        
        print(f"Transcription: {transcription}")
        
        # Clean up the temporary file
        os.remove(audio_file)
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
```

## Language Understanding with GPT

Here's an example of using GPT for language understanding and planning:

```python
# language_planning.py

import openai
import os
import json
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
openai.api_key = os.getenv('OPENAI_API_KEY')

class GPTPlanner:
    def __init__(self):
        self.model = "gpt-3.5-turbo"
    
    def create_task_prompt(self, command, robot_capabilities, environment_state):
        """Create a prompt for GPT with context"""
        prompt = f"""
        You are a Physical AI planning assistant. The user has requested: "{command}"
        
        The robot has these capabilities:
        {robot_capabilities}
        
        The current environment state is:
        {environment_state}
        
        Please generate a detailed action plan to fulfill the user's request.
        Respond with a JSON object containing:
        1. "actions": A sequence of actions to accomplish the task
        2. "questions": Any clarifying questions if the command is ambiguous
        3. "estimated_time": Estimated time to complete the task
        
        Example action types: navigate, detect, manipulate, speak, wait
        Example response format:
        {{
          "actions": [
            {{"type": "navigate", "target": "kitchen", "description": "Go to the kitchen"}},
            {{"type": "detect", "object": "water bottle", "description": "Look for a water bottle"}},
            {{"type": "manipulate", "action": "pick_up", "object": "water bottle", "description": "Pick up the water bottle"}}
          ],
          "questions": [],
          "estimated_time": "5 minutes"
        }}
        """
        return prompt
    
    def plan_command(self, command, robot_capabilities, environment_state):
        """Generate an action plan from a command using GPT"""
        try:
            prompt = self.create_task_prompt(command, robot_capabilities, environment_state)
            
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a Physical AI planning assistant. Respond with valid JSON only."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3
            )
            
            # Parse the response
            plan_text = response.choices[0].message['content']
            
            # Extract JSON from the response (in case it's wrapped in code blocks)
            if plan_text.startswith("```json"):
                plan_text = plan_text[7:]  # Remove ```json
            if plan_text.endswith("```"):
                plan_text = plan_text[:-3]  # Remove ```
            
            plan = json.loads(plan_text.strip())
            return plan
        except Exception as e:
            print(f"Error planning command: {e}")
            return None

def main():
    planner = GPTPlanner()
    
    # Example usage
    command = "Please bring me the red cup from the kitchen"
    robot_capabilities = [
        "Navigate to positions",
        "Detect objects using camera",
        "Pick up objects",
        "Manipulate objects",
        "Speak responses"
    ]
    environment_state = {
        "objects": ["red cup", "blue mug", "keyboard", "mouse"],
        "locations": ["kitchen", "living room", "bedroom"],
        "robot_position": "office"
    }
    
    plan = planner.plan_command(command, robot_capabilities, environment_state)
    
    if plan:
        print("Generated Plan:")
        print(json.dumps(plan, indent=2))
    else:
        print("Failed to generate plan")

if __name__ == '__main__':
    main()
```

## Vision Component

Here's an example of a vision component that might work with the VLA system:

```python
# vision_component.py

import cv2
import numpy as np

class VisionComponent:
    def __init__(self):
        # For this example, we'll use basic OpenCV functionality
        # In practice, this could interface with a more sophisticated vision system
        pass
    
    def detect_objects(self, image):
        """Detect objects in an image using basic color detection.
        
        In a real Physical AI system, this would use a trained neural network
        for object detection (e.g., YOLO, SSD, or similar).
        """
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define ranges for common colors
        color_ranges = {
            'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'red2': (np.array([170, 50, 50]), np.array([180, 255, 255])),  # For red wraparound
            'blue': (np.array([100, 50, 50]), np.array([130, 255, 255])),
            'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
            'yellow': (np.array([20, 50, 50]), np.array([40, 255, 255])),
        }
        
        detected_objects = []
        
        for color_name, (lower, upper) in color_ranges.items():
            if color_name == 'red2':  # Special case for red wraparound
                mask1 = cv2.inRange(hsv, lower, upper)
                combined_mask = cv2.bitwise_or(mask1, detected_objects[-1][1]) if detected_objects else mask1
                continue
                
            # Create mask for the color
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # For each contour, if it's large enough, consider it an object
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Filter small contours
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center
                    center_x, center_y = x + w//2, y + h//2
                    
                    # Add to detected objects
                    detected_objects.append({
                        'name': color_name,
                        'bbox': (x, y, x + w, y + h),
                        'center': (center_x, center_y),
                        'area': cv2.contourArea(contour)
                    })
        
        # Handle red wraparound
        mask1 = cv2.inRange(hsv, color_ranges['red'][0], color_ranges['red'][1])
        mask2 = cv2.inRange(hsv, color_ranges['red2'][0], color_ranges['red2'][1])
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in red_contours:
            if cv2.contourArea(contour) > 500:
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = x + w//2, y + h//2
                detected_objects.append({
                    'name': 'red',
                    'bbox': (x, y, x + w, y + h),
                    'center': (center_x, center_y),
                    'area': cv2.contourArea(contour)
                })
        
        return detected_objects
    
    def detect_specific_object(self, image, target_object):
        """Detect a specific object in the image.
        
        This could be extended with more sophisticated object detection
        methods like training a custom detector or using pre-trained models.
        """
        detected_objects = self.detect_objects(image)
        
        # Filter for the target object
        target_objects = []
        for obj in detected_objects:
            if target_object.lower() in obj['name'].lower():
                target_objects.append(obj)
        
        return target_objects
    
    def get_object_position(self, image, target_object):
        """Get the position of a target object in the image."""
        target_objects = self.detect_specific_object(image, target_object)
        
        if not target_objects:
            return None
        
        # For simplicity, return the first detected object's center
        return target_objects[0]['center']

def main():
    # Example usage of the Vision Component
    vision = VisionComponent()
    
    # For this example, we'll create a simple synthetic image
    # In practice, you'd capture this from a camera
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Draw some colored rectangles to simulate objects
    cv2.rectangle(image, (100, 100), (150, 150), (0, 0, 255), -1)  # Red
    cv2.rectangle(image, (300, 200), (350, 250), (255, 0, 0), -1)  # Blue
    cv2.rectangle(image, (500, 300), (550, 350), (0, 255, 0), -1)  # Green
    
    # Detect objects
    objects = vision.detect_objects(image)
    
    print(f"Detected {len(objects)} objects:")
    for obj in objects:
        print(f"  - {obj['name']}: bbox={obj['bbox']}, center={obj['center']}")
    
    # Find a specific object
    red_objects = vision.detect_specific_object(image, 'red')
    print(f"\nFound {len(red_objects)} red objects")
    
    if red_objects:
        pos = vision.get_object_position(image, 'red')
        print(f"Position of red object: {pos}")

if __name__ == '__main__':
    main()
```

## Complete VLA System Example

Here's a complete example that ties together speech recognition, language planning, and vision:

```python
# complete_vla_system.py

import openai
import os
import json
import cv2
import numpy as np
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
openai.api_key = os.getenv('OPENAI_API_KEY')

class CompleteVLASystem:
    def __init__(self):
        self.model = "gpt-3.5-turbo"
        self.vision_component = VisionComponent()
    
    def create_task_prompt(self, command, robot_capabilities, environment_state):
        """Create a prompt for GPT with context"""
        prompt = f"""
        You are a Physical AI planning assistant. The user has requested: "{command}"
        
        The robot has these capabilities:
        {robot_capabilities}
        
        The current environment state is:
        {environment_state}
        
        Please generate a detailed action plan to fulfill the user's request.
        Respond with a JSON object containing:
        1. "actions": A sequence of actions to accomplish the task
        2. "questions": Any clarifying questions if the command is ambiguous
        3. "estimated_time": Estimated time to complete the task
        """
        return prompt
    
    def plan_command(self, command, robot_capabilities, environment_state):
        """Generate an action plan from a command using GPT"""
        try:
            prompt = self.create_task_prompt(command, robot_capabilities, environment_state)
            
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a Physical AI planning assistant. Respond with valid JSON only."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3
            )
            
            # Parse the response
            plan_text = response.choices[0].message['content']
            
            # Extract JSON from the response (in case it's wrapped in code blocks)
            if plan_text.startswith("```json"):
                plan_text = plan_text[7:]  # Remove ```json
            if plan_text.endswith("```"):
                plan_text = plan_text[:-3]  # Remove ```
            
            plan = json.loads(plan_text.strip())
            return plan
        except Exception as e:
            print(f"Error planning command: {e}")
            return None
    
    def execute_action(self, action, camera_image=None):
        """Execute a single action.
        
        In a real system, this would interface with robot control systems.
        """
        action_type = action.get('type', 'unknown')
        
        if action_type == 'navigate':
            target = action.get('target', 'unknown')
            print(f"Navigating to {target}")
            # Actual navigation code would go here
            return True
            
        elif action_type == 'detect':
            obj = action.get('object', 'unknown')
            print(f"Detecting {obj}")
            
            if camera_image is not None:
                positions = self.vision_component.detect_specific_object(camera_image, obj)
                if positions:
                    print(f"Found {len(positions)} {obj}(s) at positions: {[p['center'] for p in positions]}")
                    return True
                else:
                    print(f"No {obj} found")
                    return False
            else:
                print("No camera image available for detection")
                return False
                
        elif action_type == 'manipulate':
            manip_action = action.get('action', 'unknown')
            obj = action.get('object', 'unknown')
            print(f"Attempting to {manip_action} {obj}")
            # Actual manipulation code would go here
            return True
            
        elif action_type == 'speak':
            text = action.get('text', '')
            print(f"Speaking: {text}")
            # Actual text-to-speech code would go here
            return True
            
        else:
            print(f"Unknown action type: {action_type}")
            return False
    
    def execute_plan(self, plan, camera_image=None):
        """Execute a sequence of planned actions."""
        if not plan.get('actions'):
            print("No actions in plan")
            return False
        
        success = True
        for i, action in enumerate(plan['actions']):
            print(f"Executing action {i+1}/{len(plan['actions'])}: {action.get('description', 'No description')}")
            action_success = self.execute_action(action, camera_image)
            
            if not action_success:
                print(f"Action {i+1} failed")
                success = False
                break
        
        return success

def main():
    vla_system = CompleteVLASystem()
    
    # Simulated camera image with objects
    camera_image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(camera_image, (100, 100), (150, 150), (0, 0, 255), -1)  # Red object
    
    command = "Please navigate to the red object and detect it"
    robot_capabilities = [
        "Navigate to positions",
        "Detect objects using camera",
        "Speak responses"
    ]
    environment_state = {
        "objects": ["red object", "blue object"],
        "robot_position": "starting position",
        "camera_available": True
    }
    
    print(f"Processing command: {command}")
    
    # Generate plan
    plan = vla_system.plan_command(command, robot_capabilities, environment_state)
    
    if plan:
        print("Generated Plan:")
        print(json.dumps(plan, indent=2))
        
        # Execute plan
        success = vla_system.execute_plan(plan, camera_image)
        
        if success:
            print("Plan executed successfully")
        else:
            print("Plan execution failed")
    else:
        print("Failed to generate plan")

if __name__ == '__main__':
    main()
```

## Running the Examples

1. Make sure you have set your OpenAI API key:
   ```bash
   export OPENAI_API_KEY='your-api-key-here'
   ```

2. Run individual components:
   ```bash
   # For speech recognition
   python speech_recognition.py
   
   # For language planning
   python language_planning.py
   
   # For vision component
   python vision_component.py
   
   # For complete system
   python complete_vla_system.py
   ```

## Integration with ROS 2

For integration with ROS 2 (as mentioned in the textbook), you would wrap these components as ROS 2 nodes that communicate via topics and services.

The VLA system would typically:
1. Subscribe to audio input from a microphone
2. Use Whisper for speech-to-text
3. Send text to GPT for planning
4. Use vision component to understand the environment
5. Execute the planned actions using robot control interfaces

These examples provide a foundation for building more sophisticated Vision-Language-Action systems for Physical AI applications.