# GPT-4 Vision Integration

Vision-Language-Action (VLA) models combine visual perception with natural language understanding to generate robot actions. This lesson covers integrating GPT-4 Vision (GPT-4V) with robotic systems.

## What is VLA?

**Vision-Language-Action** models:
1. **Vision**: Process camera images
2. **Language**: Understand natural language commands
3. **Action**: Generate executable robot actions

**Example**:
- Input: Image + "Pick up the red cup"
- Output: `grasp(object="red_cup", position=[0.5, 0.2, 0.1])`

## Architecture

```
Camera → Image → GPT-4V → Structured → Motion → Robot
Command → Text →         → Actions  → Planning → Execution
```

## Setting Up OpenAI API

```bash
# Install OpenAI SDK
pip install openai

# Set API key
export OPENAI_API_KEY="your-api-key-here"
```

```python
from openai import OpenAI
import base64
import cv2

class GPT4VisionClient:
    def __init__(self, api_key=None):
        self.client = OpenAI(api_key=api_key)
        self.model = "gpt-4-vision-preview"

    def encode_image(self, image):
        """
        Encode OpenCV image to base64

        Args:
            image: OpenCV image (BGR)

        Returns:
            base64 string
        """
        # Convert to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Encode to JPEG
        success, buffer = cv2.imencode('.jpg', image_rgb)

        if not success:
            raise ValueError("Failed to encode image")

        # Convert to base64
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        return image_base64

    def query(self, image, prompt, max_tokens=500):
        """
        Query GPT-4V with image and text prompt

        Args:
            image: OpenCV image or base64 string
            prompt: Text prompt/question
            max_tokens: Maximum response length

        Returns:
            Response text
        """
        # Encode image if needed
        if isinstance(image, str):
            image_data = image
        else:
            image_data = self.encode_image(image)

        # Create message
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": prompt
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_data}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=max_tokens
        )

        return response.choices[0].message.content
```

## Basic Usage Example

```python
# Initialize client
client = GPT4VisionClient()

# Load image
image = cv2.imread('robot_scene.jpg')

# Query
prompt = "Describe all objects you see in this image."
response = client.query(image, prompt)

print(response)
# Output: "I see a table with a red cup, blue book, and yellow banana..."
```

## Structured Action Generation

Use function calling for structured outputs.

```python
import json

class VLAActionGenerator:
    def __init__(self, api_key=None):
        self.client = OpenAI(api_key=api_key)

        # Define available robot actions
        self.actions_schema = {
            "name": "generate_robot_actions",
            "description": "Generate a sequence of robot actions based on visual scene and command",
            "parameters": {
                "type": "object",
                "properties": {
                    "actions": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "action_type": {
                                    "type": "string",
                                    "enum": ["grasp", "place", "move_to", "rotate", "wait"]
                                },
                                "object_name": {
                                    "type": "string"
                                },
                                "position": {
                                    "type": "array",
                                    "items": {"type": "number"},
                                    "minItems": 3,
                                    "maxItems": 3
                                },
                                "gripper_state": {
                                    "type": "string",
                                    "enum": ["open", "close"]
                                }
                            },
                            "required": ["action_type"]
                        }
                    }
                },
                "required": ["actions"]
            }
        }

    def generate_actions(self, image, command):
        """
        Generate structured action sequence

        Args:
            image: OpenCV image
            command: Natural language command

        Returns:
            List of action dictionaries
        """
        # Encode image
        image_base64 = self.encode_image(image)

        # Create prompt
        prompt = f"""
        You are a robot action planner. Given the scene in the image and the command: "{command}",
        generate a sequence of robot actions.

        Available actions:
        - grasp: Pick up an object
        - place: Put down an object
        - move_to: Move to a position
        - rotate: Rotate gripper
        - wait: Pause execution

        Provide actions as a structured JSON sequence.
        """

        response = self.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}"
                            }
                        }
                    ]
                }
            ],
            functions=[self.actions_schema],
            function_call={"name": "generate_robot_actions"}
        )

        # Extract actions
        function_call = response.choices[0].message.function_call
        actions = json.loads(function_call.arguments)

        return actions['actions']

    def encode_image(self, image):
        """Encode OpenCV image to base64"""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        success, buffer = cv2.imencode('.jpg', image_rgb)
        return base64.b64encode(buffer).decode('utf-8')
```

## Example Usage

```python
# Initialize
vla = VLAActionGenerator()

# Load scene image
scene = cv2.imread('table_scene.jpg')

# Generate actions
command = "Pick up the red cup and place it on the blue plate"
actions = vla.generate_actions(scene, command)

print(json.dumps(actions, indent=2))
```

**Output**:
```json
[
  {
    "action_type": "move_to",
    "position": [0.5, 0.2, 0.3],
    "object_name": "red_cup"
  },
  {
    "action_type": "grasp",
    "object_name": "red_cup",
    "gripper_state": "close"
  },
  {
    "action_type": "move_to",
    "position": [0.3, 0.4, 0.3],
    "object_name": "blue_plate"
  },
  {
    "action_type": "place",
    "object_name": "red_cup",
    "gripper_state": "open"
  }
]
```

## ROS 2 Integration

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class VLANode(Node):
    def __init__(self):
        super().__init__('vla_node')

        # VLA client
        self.vla = VLAActionGenerator()

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        # Publisher
        self.action_pub = self.create_publisher(
            String, '/robot_actions', 10
        )

        # State
        self.latest_image = None
        self.processing = False

    def image_callback(self, msg):
        """Store latest image"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def command_callback(self, msg):
        """Process voice command"""
        if self.processing:
            self.get_logger().warn("Already processing a command")
            return

        if self.latest_image is None:
            self.get_logger().warn("No image available")
            return

        self.processing = True

        # Generate actions
        command = msg.data
        self.get_logger().info(f"Processing command: {command}")

        try:
            actions = self.vla.generate_actions(self.latest_image, command)

            # Publish actions
            action_msg = String()
            action_msg.data = json.dumps(actions)
            self.action_pub.publish(action_msg)

            self.get_logger().info(f"Generated {len(actions)} actions")

        except Exception as e:
            self.get_logger().error(f"Action generation failed: {e}")

        self.processing = False
```

## Optimizations

### 1. Image Compression

Reduce API costs by compressing images.

```python
def compress_image(image, max_size=(640, 480), quality=85):
    """
    Compress image for API

    Args:
        image: OpenCV image
        max_size: Maximum (width, height)
        quality: JPEG quality (0-100)

    Returns:
        Compressed image
    """
    h, w = image.shape[:2]
    max_w, max_h = max_size

    # Resize if needed
    if w > max_w or h > max_h:
        scale = min(max_w / w, max_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        image = cv2.resize(image, (new_w, new_h))

    # Encode with quality
    encode_param = [cv2.IMWRITE_JPEG_QUALITY, quality]
    success, buffer = cv2.imencode('.jpg', image, encode_param)

    return cv2.imdecode(buffer, cv2.IMREAD_COLOR)
```

### 2. Caching

Cache responses for similar queries.

```python
import hashlib

class CachedVLA:
    def __init__(self, vla_generator):
        self.vla = vla_generator
        self.cache = {}

    def generate_actions(self, image, command):
        """Generate actions with caching"""

        # Create cache key
        image_hash = hashlib.md5(image.tobytes()).hexdigest()
        cache_key = f"{image_hash}_{command}"

        if cache_key in self.cache:
            return self.cache[cache_key]

        # Generate
        actions = self.vla.generate_actions(image, command)

        # Cache
        self.cache[cache_key] = actions

        return actions
```

### 3. Batch Processing

Process multiple queries efficiently.

```python
async def batch_generate_actions(vla, image_command_pairs):
    """
    Generate actions for multiple (image, command) pairs

    Args:
        vla: VLAActionGenerator instance
        image_command_pairs: List of (image, command) tuples

    Returns:
        List of action sequences
    """
    import asyncio

    tasks = [
        asyncio.to_thread(vla.generate_actions, img, cmd)
        for img, cmd in image_command_pairs
    ]

    results = await asyncio.gather(*tasks)

    return results
```

## Error Handling

```python
def robust_vla_query(vla, image, command, max_retries=3):
    """VLA query with retries and error handling"""

    for attempt in range(max_retries):
        try:
            actions = vla.generate_actions(image, command)
            return actions

        except Exception as e:
            if attempt < max_retries - 1:
                wait_time = 2 ** attempt  # Exponential backoff
                print(f"Retry {attempt + 1} after {wait_time}s...")
                time.sleep(wait_time)
            else:
                print(f"Failed after {max_retries} attempts: {e}")
                return []

    return []
```

## Next Steps

In the next lesson, you'll learn to process multi-modal inputs (images + text) and generate more sophisticated action sequences.

## Further Reading

- [OpenAI Vision API](https://platform.openai.com/docs/guides/vision)
- [Function Calling](https://platform.openai.com/docs/guides/function-calling)
- [GPT-4V System Card](https://cdn.openai.com/papers/GPTV_System_Card.pdf)
