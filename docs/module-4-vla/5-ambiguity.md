# Handling Ambiguous Commands

Resolve ambiguities in natural language commands through clarification and context.

## Types of Ambiguity

1. **Object Ambiguity**: "Pick up the cup" (which cup?)
2. **Location Ambiguity**: "Place it there" (where exactly?)
3. **Method Ambiguity**: "Move the box" (push, carry, or drag?)
4. **Temporal Ambiguity**: "Do it later" (when?)
5. **Goal Ambiguity**: "Clean up" (what constitutes clean?)

## Ambiguity Detection

```python
class AmbiguityDetector:
    def detect_ambiguities(self, command, scene_description):
        """
        Detect potential ambiguities in command

        Returns:
            List of detected ambiguities with types
        """
        ambiguities = []

        # Check for underspecified objects
        if self.has_vague_reference(command):
            ambiguities.append({
                'type': 'object_reference',
                'issue': 'Vague object reference detected',
                'examples': ['the cup', 'it', 'that one']
            })

        # Check for missing location
        if 'place' in command.lower() and not self.has_location(command):
            ambiguities.append({
                'type': 'location',
                'issue': 'Target location not specified'
            })

        # Check for multiple viable interpretations
        interpretations = self.count_interpretations(command, scene_description)
        if interpretations > 1:
            ambiguities.append({
                'type': 'multiple_interpretations',
                'issue': f'{interpretations} possible interpretations found'
            })

        return ambiguities

    def has_vague_reference(self, command):
        """Check for vague object references"""
        vague_terms = ['it', 'that', 'this', 'the thing', 'something']
        return any(term in command.lower() for term in vague_terms)

    def has_location(self, command):
        """Check if command includes location"""
        location_terms = ['on', 'in', 'at', 'to', 'near', 'beside']
        return any(term in command.lower() for term in location_terms)
```

## Clarification Generation

```python
class ClarificationGenerator:
    def __init__(self, vla_client):
        self.vla = vla_client

    def generate_clarification_questions(self, command, image, ambiguities):
        """
        Generate natural clarification questions

        Returns:
            List of questions to ask user
        """
        prompt = f"""
        User command: "{command}"

        Detected ambiguities:
        {json.dumps(ambiguities, indent=2)}

        Scene: [see image]

        Generate 1-3 concise clarification questions to resolve these ambiguities.
        Make questions specific to objects visible in the scene.

        Format as JSON:
        {{
            "questions": [
                {{"question": "...", "resolves": "object_reference"}},
                ...
            ]
        }}
        """

        image_base64 = self.vla.encode_image(image)

        response = self.vla.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}
                        }
                    ]
                }
            ],
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)
        return result['questions']
```

## Interactive Disambiguation

```python
class InteractiveDisambiguator:
    def __init__(self, vla_client):
        self.vla = vla_client
        self.detector = AmbiguityDetector()
        self.clarifier = ClarificationGenerator(vla_client)

    def resolve_command(self, command, image, interaction_callback):
        """
        Iteratively resolve ambiguities

        Args:
            command: Original command
            image: Scene image
            interaction_callback: Function to get user responses

        Returns:
            Resolved, unambiguous command
        """
        resolved_command = command
        scene_desc = self.describe_scene(image)

        max_iterations = 3
        for iteration in range(max_iterations):
            # Detect ambiguities
            ambiguities = self.detector.detect_ambiguities(resolved_command, scene_desc)

            if not ambiguities:
                # No more ambiguities
                return resolved_command

            # Generate clarification questions
            questions = self.clarifier.generate_clarification_questions(
                resolved_command, image, ambiguities
            )

            # Get user responses
            responses = interaction_callback(questions)

            # Incorporate responses
            resolved_command = self.incorporate_responses(
                resolved_command, questions, responses
            )

        return resolved_command

    def describe_scene(self, image):
        """Get scene description for ambiguity detection"""
        # Use VLA to describe scene
        prompt = "Describe all objects in this scene briefly."
        description = self.vla.query(image, prompt)
        return description

    def incorporate_responses(self, command, questions, responses):
        """Update command with clarification responses"""
        prompt = f"""
        Original command: "{command}"

        Clarification Q&A:
        {self.format_qa_pairs(questions, responses)}

        Rewrite the command to be completely unambiguous based on the clarifications.
        Provide only the rewritten command.
        """

        response = self.vla.client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[{"role": "user", "content": prompt}]
        )

        return response.choices[0].message.content.strip()

    def format_qa_pairs(self, questions, responses):
        """Format questions and answers"""
        pairs = []
        for q, r in zip(questions, responses):
            pairs.append(f"Q: {q['question']}\nA: {r}")
        return "\n\n".join(pairs)
```

## Context-Based Resolution

```python
class ContextResolver:
    def __init__(self):
        self.scene_graph = {}
        self.recent_actions = []

    def resolve_with_context(self, command):
        """Resolve using scene graph and history"""

        # Resolve pronoun references
        if 'it' in command.lower():
            last_object = self.get_last_mentioned_object()
            command = command.replace('it', last_object)

        # Resolve spatial references
        if 'there' in command.lower():
            last_location = self.get_last_location()
            command = command.replace('there', f'at {last_location}')

        return command

    def get_last_mentioned_object(self):
        """Get most recently mentioned object"""
        if self.recent_actions:
            last_action = self.recent_actions[-1]
            if 'object' in last_action:
                return last_action['object']
        return "the object"

    def get_last_location(self):
        """Get most recently referenced location"""
        if self.recent_actions:
            for action in reversed(self.recent_actions):
                if 'location' in action:
                    return f"({action['location'][0]:.2f}, {action['location'][1]:.2f})"
        return "(0.5, 0.5)"
```

## ROS 2 Integration

```python
from std_msgs.msg import String
from std_srvs.srv import Trigger

class DisambiguationNode(Node):
    def __init__(self):
        super().__init__('disambiguation_node')

        self.disambiguator = InteractiveDisambiguator(vla_client)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )

        # Publishers
        self.clarification_pub = self.create_publisher(
            String, '/clarification_question', 10
        )
        self.resolved_command_pub = self.create_publisher(
            String, '/resolved_command', 10
        )

        # Service for user responses
        self.response_srv = self.create_service(
            Trigger, '/user_response', self.handle_user_response
        )

        self.latest_image = None
        self.pending_questions = []
        self.user_responses = []

    def command_callback(self, msg):
        """Handle incoming command"""
        if self.latest_image is None:
            self.get_logger().warn("No image available")
            return

        # Resolve with interaction
        resolved = self.disambiguator.resolve_command(
            msg.data,
            self.latest_image,
            self.interaction_callback
        )

        # Publish resolved command
        resolved_msg = String()
        resolved_msg.data = resolved
        self.resolved_command_pub.publish(resolved_msg)

    def interaction_callback(self, questions):
        """Ask user for clarifications"""
        self.pending_questions = questions
        self.user_responses = []

        # Publish questions
        for q in questions:
            msg = String()
            msg.data = q['question']
            self.clarification_pub.publish(msg)

        # Wait for responses (in practice, use async/await)
        while len(self.user_responses) < len(questions):
            time.sleep(0.1)

        return self.user_responses

    def handle_user_response(self, request, response):
        """Handle user response to clarification"""
        # Store response
        self.user_responses.append(request.data)

        response.success = True
        return response
```

## Further Reading

- [Natural Language Disambiguation](https://arxiv.org/abs/2010.12083)
- [Interactive Robot Learning](https://arxiv.org/abs/2203.00954)
