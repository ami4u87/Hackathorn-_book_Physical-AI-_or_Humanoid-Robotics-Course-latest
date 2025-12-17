# Structured Action Generation

Generate executable robot action sequences from high-level commands.

## Action Primitives

Define basic robot capabilities:

```python
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class RobotAction:
    action_type: str
    parameters: dict
    preconditions: List[str] = None
    postconditions: List[str] = None

# Define action types
ACTION_TYPES = {
    "grasp": {
        "params": ["object_id", "grasp_pose"],
        "duration": 2.0
    },
    "place": {
        "params": ["object_id", "target_pose"],
        "duration": 2.0
    },
    "move_to": {
        "params": ["target_pose"],
        "duration": 3.0
    },
    "open_gripper": {
        "params": [],
        "duration": 1.0
    },
    "close_gripper": {
        "params": [],
        "duration": 1.0
    },
    "rotate": {
        "params": ["axis", "angle"],
        "duration": 1.5
    }
}
```

## Structured Generation

Use JSON schemas for consistent outputs.

```python
def get_action_schema():
    """Define JSON schema for action sequences"""
    return {
        "type": "object",
        "properties": {
            "task_understanding": {
                "type": "string",
                "description": "Brief description of understood task"
            },
            "action_sequence": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "step": {"type": "integer"},
                        "action_type": {
                            "type": "string",
                            "enum": list(ACTION_TYPES.keys())
                        },
                        "parameters": {
                            "type": "object"
                        },
                        "reasoning": {
                            "type": "string",
                            "description": "Why this action is needed"
                        }
                    },
                    "required": ["step", "action_type", "parameters"]
                }
            },
            "estimated_duration": {
                "type": "number",
                "description": "Total estimated time in seconds"
            }
        },
        "required": ["task_understanding", "action_sequence"]
    }

class ActionSequenceGenerator:
    def __init__(self, vla_client):
        self.vla = vla_client
        self.action_schema = get_action_schema()

    def generate_sequence(self, image, command, constraints=None):
        """
        Generate structured action sequence

        Args:
            image: Scene image
            command: Natural language command
            constraints: Optional constraints (e.g., max_duration, forbidden_objects)

        Returns:
            Structured action sequence
        """
        # Build prompt
        prompt = self.build_generation_prompt(command, constraints)

        # Encode image
        image_base64 = self.vla.encode_image(image)

        # Generate with function calling
        response = self.vla.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "system",
                    "content": "You are a robot task planner. Generate structured, executable action sequences."
                },
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

        # Parse result
        result = json.loads(response.choices[0].message.content)

        # Validate
        self.validate_sequence(result)

        return result

    def build_generation_prompt(self, command, constraints):
        """Build detailed prompt for action generation"""
        prompt = f"""
        Generate a robot action sequence for: "{command}"

        Available actions:
        {json.dumps(ACTION_TYPES, indent=2)}

        Constraints:
        {json.dumps(constraints, indent=2) if constraints else "None"}

        Provide response as JSON following this structure:
        {{
            "task_understanding": "...",
            "action_sequence": [
                {{
                    "step": 1,
                    "action_type": "move_to",
                    "parameters": {{"target_pose": [x, y, z]}},
                    "reasoning": "..."
                }},
                ...
            ],
            "estimated_duration": 10.5
        }}
        """
        return prompt

    def validate_sequence(self, sequence):
        """Validate generated sequence"""
        assert "action_sequence" in sequence, "Missing action_sequence"

        for action in sequence["action_sequence"]:
            action_type = action["action_type"]
            assert action_type in ACTION_TYPES, f"Unknown action: {action_type}"

            # Validate parameters
            required_params = ACTION_TYPES[action_type]["params"]
            for param in required_params:
                assert param in action["parameters"], f"Missing parameter: {param}"
```

## Hierarchical Planning

Break complex tasks into subtasks.

```python
def hierarchical_plan(vla, image, high_level_goal):
    """
    Generate hierarchical plan

    Returns:
        {
            "goal": "...",
            "subtasks": [
                {
                    "subtask": "...",
                    "actions": [...]
                },
                ...
            ]
        }
    """
    prompt = f"""
    Break down this goal into subtasks: "{high_level_goal}"

    For each subtask, provide:
    1. Subtask description
    2. Action sequence
    3. Success criteria

    Format as hierarchical JSON.
    """

    # Generate
    # ... (similar to previous examples)
```

## Action Libraries

Reusable action templates.

```python
ACTION_LIBRARY = {
    "pick_and_place": {
        "template": [
            {"action_type": "move_to", "parameters": {"target": "{{object_location}}"}},
            {"action_type": "open_gripper"},
            {"action_type": "grasp", "parameters": {"object_id": "{{object_id}}"}},
            {"action_type": "close_gripper"},
            {"action_type": "move_to", "parameters": {"target": "{{place_location}}"}},
            {"action_type": "open_gripper"}
        ]
    },
    "inspect_object": {
        "template": [
            {"action_type": "move_to", "parameters": {"target": "{{object_location}}"}},
            {"action_type": "rotate", "parameters": {"axis": "z", "angle": 360}},
        ]
    }
}

def instantiate_template(template_name, **kwargs):
    """Fill template with actual values"""
    template = ACTION_LIBRARY[template_name]["template"]

    # Replace placeholders
    instantiated = []
    for action in template:
        action_copy = json.loads(json.dumps(action))  # Deep copy

        # Replace in parameters
        for key, value in action_copy["parameters"].items():
            if isinstance(value, str) and value.startswith("{{"):
                param_name = value.strip("{}")
                action_copy["parameters"][key] = kwargs[param_name]

        instantiated.append(action_copy)

    return instantiated
```

## Further Reading

- [Task and Motion Planning](https://arxiv.org/abs/2010.01083)
- [LLM Planning](https://arxiv.org/abs/2305.14992)
