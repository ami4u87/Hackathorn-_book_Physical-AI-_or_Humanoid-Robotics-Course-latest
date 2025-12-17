# Action Feasibility Validation

Validate that generated actions are safe and executable before execution.

## Validation Pipeline

```
Generated Actions → Geometric Validation → Physics Validation → Safety Check → Executable Actions
```

## Geometric Feasibility

Check reachability and collision-free paths.

```python
class GeometricValidator:
    def __init__(self, robot_model):
        self.robot = robot_model
        self.workspace_bounds = {
            'x': (-0.5, 0.8),
            'y': (-0.6, 0.6),
            'z': (0.0, 1.2)
        }

    def validate_reachability(self, target_pose):
        """Check if target is within workspace"""
        x, y, z = target_pose

        if not (self.workspace_bounds['x'][0] <= x <= self.workspace_bounds['x'][1]):
            return False, "Target x out of workspace"

        if not (self.workspace_bounds['y'][0] <= y <= self.workspace_bounds['y'][1]):
            return False, "Target y out of workspace"

        if not (self.workspace_bounds['z'][0] <= z <= self.workspace_bounds['z'][1]):
            return False, "Target z out of workspace"

        # Check IK solution exists
        ik_solution = self.robot.inverse_kinematics(target_pose)

        if ik_solution is None:
            return False, "No IK solution found"

        # Check joint limits
        if not self.check_joint_limits(ik_solution):
            return False, "Joint limits exceeded"

        return True, "Reachable"

    def check_joint_limits(self, joint_positions):
        """Verify joint positions are within limits"""
        for i, pos in enumerate(joint_positions):
            lower, upper = self.robot.joint_limits[i]
            if not (lower <= pos <= upper):
                return False
        return True

    def check_collision(self, path):
        """Check if path is collision-free"""
        # Simplified collision check
        for waypoint in path:
            if self.is_in_collision(waypoint):
                return False
        return True

    def is_in_collision(self, joint_state):
        """Check single configuration for collisions"""
        # Implement actual collision checking with environment
        # Use libraries like FCL or built-in simulator collision detection
        pass
```

## Physics Validation

Ensure actions respect physics constraints.

```python
class PhysicsValidator:
    def validate_grasp(self, object_properties, gripper_properties):
        """Validate grasp is physically possible"""

        # Check object weight
        if object_properties['mass'] > gripper_properties['max_payload']:
            return False, f"Object too heavy: {object_properties['mass']}kg > {gripper_properties['max_payload']}kg"

        # Check object size
        obj_width = object_properties['dimensions'][0]
        gripper_max_width = gripper_properties['max_opening']

        if obj_width > gripper_max_width:
            return False, f"Object too wide: {obj_width}m > {gripper_max_width}m"

        # Check friction coefficient
        if object_properties['friction'] < 0.3:
            return False, "Object too slippery to grasp reliably"

        return True, "Grasp feasible"

    def validate_dynamics(self, action_sequence):
        """Check if sequence respects dynamic limits"""

        for i in range(len(action_sequence) - 1):
            current = action_sequence[i]
            next_action = action_sequence[i + 1]

            # Check velocity limits
            if not self.check_velocity_limits(current, next_action):
                return False, f"Velocity limit exceeded between step {i} and {i+1}"

            # Check acceleration limits
            if not self.check_acceleration_limits(current, next_action):
                return False, f"Acceleration limit exceeded between step {i} and {i+1}"

        return True, "Dynamics feasible"
```

## Safety Validation

```python
class SafetyValidator:
    def __init__(self):
        self.forbidden_zones = [
            {"name": "human_workspace", "bounds": [[0.0, 0.5], [-0.3, 0.3], [0.0, 2.0]]},
        ]

        self.max_velocity = 0.5  # m/s
        self.max_force = 50.0    # N

    def validate_safety(self, action):
        """Comprehensive safety check"""

        # Check forbidden zones
        if 'target_pose' in action['parameters']:
            pose = action['parameters']['target_pose']
            if self.is_in_forbidden_zone(pose):
                return False, "Target in forbidden zone"

        # Check velocity
        if 'velocity' in action['parameters']:
            if action['parameters']['velocity'] > self.max_velocity:
                return False, f"Velocity too high: {action['parameters']['velocity']} > {self.max_velocity}"

        # Check force limits
        if 'force' in action['parameters']:
            if action['parameters']['force'] > self.max_force:
                return False, "Force limit exceeded"

        return True, "Safe"

    def is_in_forbidden_zone(self, pose):
        """Check if pose is in any forbidden zone"""
        x, y, z = pose

        for zone in self.forbidden_zones:
            x_bounds, y_bounds, z_bounds = zone['bounds']

            if (x_bounds[0] <= x <= x_bounds[1] and
                y_bounds[0] <= y <= y_bounds[1] and
                z_bounds[0] <= z <= z_bounds[1]):
                return True

        return False
```

## Complete Validation System

```python
class ActionValidator:
    def __init__(self, robot_model):
        self.geometric = GeometricValidator(robot_model)
        self.physics = PhysicsValidator()
        self.safety = SafetyValidator()

    def validate_action_sequence(self, actions, environment):
        """Validate entire action sequence"""

        validation_results = []

        for i, action in enumerate(actions):
            result = self.validate_single_action(action, environment)
            result['step'] = i
            validation_results.append(result)

            if not result['valid']:
                # Stop at first invalid action
                break

        # Overall result
        all_valid = all(r['valid'] for r in validation_results)

        return {
            'valid': all_valid,
            'results': validation_results,
            'executable_count': sum(1 for r in validation_results if r['valid'])
        }

    def validate_single_action(self, action, environment):
        """Validate a single action"""

        checks = []

        # Geometric validation
        if 'target_pose' in action['parameters']:
            valid, msg = self.geometric.validate_reachability(action['parameters']['target_pose'])
            checks.append({'check': 'reachability', 'valid': valid, 'message': msg})

        # Physics validation
        if action['action_type'] == 'grasp':
            obj_id = action['parameters'].get('object_id')
            if obj_id and obj_id in environment['objects']:
                obj_props = environment['objects'][obj_id]
                gripper_props = environment['gripper']
                valid, msg = self.physics.validate_grasp(obj_props, gripper_props)
                checks.append({'check': 'grasp_physics', 'valid': valid, 'message': msg})

        # Safety validation
        valid, msg = self.safety.validate_safety(action)
        checks.append({'check': 'safety', 'valid': valid, 'message': msg})

        # Overall
        all_valid = all(c['valid'] for c in checks)

        return {
            'action': action,
            'valid': all_valid,
            'checks': checks
        }
```

## Integration with VLA

```python
class ValidatedVLA:
    def __init__(self, vla_generator, validator):
        self.vla = vla_generator
        self.validator = validator

    def generate_and_validate(self, image, command, environment, max_attempts=3):
        """
        Generate actions and validate, retry if invalid

        Args:
            image: Scene image
            command: User command
            environment: Environment description
            max_attempts: Maximum generation attempts

        Returns:
            Valid action sequence or None
        """
        for attempt in range(max_attempts):
            # Generate actions
            actions = self.vla.generate_actions(image, command)

            # Validate
            validation = self.validator.validate_action_sequence(actions, environment)

            if validation['valid']:
                return {
                    'actions': actions,
                    'validation': validation,
                    'attempts': attempt + 1
                }

            # If invalid, provide feedback and regenerate
            feedback = self.create_feedback(validation)
            command_with_feedback = f"{command}\n\nPrevious attempt failed: {feedback}"

        return None  # Failed after max attempts

    def create_feedback(self, validation):
        """Create feedback from validation results"""
        failed_checks = [
            r for r in validation['results']
            if not r['valid']
        ]

        if not failed_checks:
            return "No specific issues found"

        feedback_parts = []
        for result in failed_checks:
            for check in result['checks']:
                if not check['valid']:
                    feedback_parts.append(f"Step {result['step']}: {check['message']}")

        return "; ".join(feedback_parts)
```

## Further Reading

- [Motion Planning](https://moveit.ros.org/)
- [Safety-Critical Systems](https://arxiv.org/abs/2108.01661)
