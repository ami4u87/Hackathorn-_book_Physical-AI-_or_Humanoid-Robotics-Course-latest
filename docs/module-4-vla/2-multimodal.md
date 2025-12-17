# Multi-Modal Processing

Process camera images and text together for enhanced robot understanding.

## Multi-Modal Inputs

Combine:
- **Visual**: Camera images, depth maps
- **Textual**: Commands, context, constraints
- **Spatial**: Robot state, object poses
- **Temporal**: Previous actions, history

## Enhanced VLA with Context

```python
class ContextualVLA:
    def __init__(self, api_key=None):
        self.client = OpenAI(api_key=api_key)
        self.conversation_history = []
        self.scene_memory = []

    def add_context(self, context_type, data):
        """Add contextual information"""
        self.conversation_history.append({
            "type": context_type,
            "data": data,
            "timestamp": time.time()
        })

    def generate_actions_with_context(self, image, command, robot_state=None):
        """
        Generate actions using full context

        Args:
            image: Current scene image
            command: User command
            robot_state: Current robot configuration

        Returns:
            Action sequence with reasoning
        """
        # Build context prompt
        context_parts = []

        # Add robot state
        if robot_state:
            context_parts.append(f"Robot state: {json.dumps(robot_state)}")

        # Add recent history
        recent_history = self.conversation_history[-5:]
        if recent_history:
            history_str = "\n".join([
                f"- {h['type']}: {h['data']}"
                for h in recent_history
            ])
            context_parts.append(f"Recent history:\n{history_str}")

        # Combine with command
        full_prompt = f"""
        Context:
        {chr(10).join(context_parts)}

        Current Command: {command}

        Based on the image and context, generate appropriate robot actions.
        Explain your reasoning.
        """

        # Encode image
        image_base64 = self.encode_image(image)

        response = self.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {"role": "system", "content": "You are a robot action planner with spatial reasoning."},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": full_prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_base64}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=800
        )

        result = response.choices[0].message.content

        # Store in history
        self.add_context("command", command)
        self.add_context("response", result)

        return result

    def encode_image(self, image):
        import base64, cv2
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        success, buffer = cv2.imencode('.jpg', image_rgb)
        return base64.b64encode(buffer).decode('utf-8')
```

## Multiple Image Processing

Process multiple views simultaneously.

```python
def process_multiple_views(client, images, command):
    """
    Process multiple camera views

    Args:
        client: OpenAI client
        images: List of OpenCV images
        command: User command

    Returns:
        Integrated understanding
    """
    # Encode all images
    encoded_images = [encode_image(img) for img in images]

    # Build multi-view prompt
    content = [
        {
            "type": "text",
            "text": f"You have {len(images)} camera views. Command: {command}\n\n"
                    "Analyze all views and provide a comprehensive action plan."
        }
    ]

    # Add all images
    for i, img_data in enumerate(encoded_images):
        content.append({
            "type": "text",
            "text": f"\n## View {i+1}:"
        })
        content.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{img_data}"
            }
        })

    response = client.chat.completions.create(
        model="gpt-4-vision-preview",
        messages=[{"role": "user", "content": content}],
        max_tokens=1000
    )

    return response.choices[0].message.content
```

## Depth Integration

Include depth information in prompts.

```python
def create_depth_visualization(depth_image):
    """Create interpretable depth visualization"""
    import cv2
    import numpy as np

    # Normalize depth
    depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

    # Add distance labels
    h, w = depth_image.shape
    for y in range(0, h, 100):
        for x in range(0, w, 100):
            dist = depth_image[y, x] / 1000.0  # Convert to meters
            cv2.putText(depth_colored, f"{dist:.1f}m",
                       (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                       0.4, (255, 255, 255), 1)

    return depth_colored

def process_with_depth(vla, rgb_image, depth_image, command):
    """Process RGB and depth together"""

    # Create depth visualization
    depth_viz = create_depth_visualization(depth_image)

    # Process both images
    result = process_multiple_views(
        vla.client,
        [rgb_image, depth_viz],
        f"{command} (Second image shows depth/distance information)"
    )

    return result
```

## Object Grounding

Link language to visual objects.

```python
def ground_objects_in_scene(vla, image, objects_of_interest):
    """
    Ground object references in scene

    Args:
        vla: VLA client
        image: Scene image
        objects_of_interest: List of object names

    Returns:
        Dict mapping object names to bounding boxes
    """
    prompt = f"""
    Identify and locate these objects in the image: {', '.join(objects_of_interest)}.

    For each object found, provide:
    1. Object name
    2. Bounding box (x, y, width, height) in pixels
    3. Confidence (low/medium/high)

    Format as JSON.
    """

    response = vla.client.chat.completions.create(
        model="gpt-4-vision-preview",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{vla.encode_image(image)}"
                        }
                    }
                ]
            }
        ]
    )

    # Parse response (implement JSON extraction)
    return parse_object_groundings(response.choices[0].message.content)
```

## Temporal Reasoning

Include action history.

```python
class TemporalVLA:
    def __init__(self):
        self.action_history = []
        self.scene_history = []

    def plan_next_action(self, current_image, goal):
        """Plan considering previous actions"""

        history_summary = self.summarize_history()

        prompt = f"""
        Goal: {goal}

        Previous actions:
        {history_summary}

        Current scene: [see image]

        What should the robot do next? Consider what has already been done.
        """

        # Generate with context
        # ... (similar to previous examples)

    def summarize_history(self):
        """Create concise history summary"""
        if not self.action_history:
            return "No previous actions."

        summary = "\n".join([
            f"{i+1}. {action['type']}: {action.get('object', 'N/A')}"
            for i, action in enumerate(self.action_history[-10:])
        ])

        return summary
```

## Further Reading

- [Multi-Modal Learning](https://arxiv.org/abs/2306.13549)
- [Vision-Language Models](https://arxiv.org/abs/2304.10592)
