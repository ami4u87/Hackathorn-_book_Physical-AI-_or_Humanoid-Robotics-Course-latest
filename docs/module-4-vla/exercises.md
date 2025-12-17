# Module 4 Exercises

Practice Vision-Language-Action integration with these exercises.

## Exercise 1: Basic VLA Integration

**Objective**: Set up GPT-4 Vision and generate simple actions.

**Tasks**:
1. Set up OpenAI API key
2. Implement basic image + text query
3. Generate action for "Pick up the red object"
4. Print structured action output

**Deliverables**: Working VLA client code, example outputs

---

## Exercise 2: Multi-View Processing

**Objective**: Process multiple camera views simultaneously.

**Tasks**:
1. Set up 2-3 cameras in simulation
2. Capture images from all views
3. Send all views to GPT-4V
4. Generate actions based on complete scene understanding

**Success Criteria**: Actions consider information from all views

---

## Exercise 3: Structured Action Generation

**Objective**: Generate consistent, structured action sequences.

**Tasks**:
1. Define action schema (JSON)
2. Implement schema validation
3. Generate actions using function calling
4. Handle malformed outputs gracefully

**Deliverables**: Schema definition, validated action sequences

---

## Exercise 4: Geometric Validation

**Objective**: Validate action feasibility.

**Tasks**:
1. Implement workspace bounds checking
2. Add IK reachability validation
3. Test with valid and invalid targets
4. Report validation results

**Test Cases**:
- Target within workspace → Valid
- Target out of reach → Invalid
- Target behind robot → Invalid

---

## Exercise 5: Physics Validation

**Objective**: Check physical constraints.

**Tasks**:
1. Define object properties (mass, size, friction)
2. Validate grasp feasibility
3. Check payload limits
4. Test with various objects

**Scenarios**:
- Light object, good friction → Valid grasp
- Heavy object → Invalid (exceeds payload)
- Slippery object → Invalid (low friction)

---

## Exercise 6: Ambiguity Detection

**Objective**: Detect and resolve ambiguous commands.

**Tasks**:
1. Implement ambiguity detector
2. Generate clarification questions
3. Test with ambiguous commands
4. Resolve through user interaction

**Test Commands**:
- "Pick up the cup" (multiple cups) → Ambiguous
- "Place it there" → Ambiguous
- "Pick up the red cup on the left" → Unambiguous

---

## Exercise 7: Interactive Disambiguation

**Objective**: Build interactive clarification system.

**Tasks**:
1. Create ROS 2 node for disambiguation
2. Subscribe to voice commands
3. Publish clarification questions
4. Incorporate user responses
5. Publish resolved commands

**Deliverables**: Disambiguation node, launch file, demo video

---

## Exercise 8: Context-Aware Planning

**Objective**: Use history and context for better planning.

**Tasks**:
1. Maintain action history
2. Track mentioned objects
3. Resolve pronouns ("it", "that")
4. Use previous locations as context

**Example**:
- "Pick up the red cup" → Store "red cup" as last object
- "Place it on the table" → Resolve "it" to "red cup"

---

## Exercise 9: Complete VLA Pipeline

**Objective**: Integrate all components.

**Pipeline**:
1. Camera input
2. VLA action generation
3. Validation (geometric + physics + safety)
4. Clarification (if ambiguous)
5. Action execution (simulation)

**Success Criteria**:
- End-to-end execution
- Handles valid and invalid commands
- Resolves ambiguities
- Safe execution only

---

## Exercise 10: Real-World Task

**Objective**: Complete practical task using VLA.

**Task**: "Clean up the table"

**Requirements**:
1. Identify all objects on table
2. Generate pickup sequence
3. Validate each action
4. Execute in simulation
5. Verify table is clear

**Bonus**: Handle unexpected situations (object falls, grasp fails)

---

## Bonus Challenges

### Challenge A: Multi-Step Reasoning
Command: "Make a sandwich"
Generate multi-step plan with dependencies.

### Challenge B: Safety-Critical
Implement hard safety constraints (never enter human workspace).

### Challenge C: Failure Recovery
Handle action failures and replan.

### Challenge D: Natural Dialogue
Support multi-turn conversations for task refinement.

---

## Submission Guidelines

For each exercise:
1. **Code**: Clean implementation with error handling
2. **Examples**: Multiple test cases with outputs
3. **Analysis**: Performance metrics (latency, accuracy)
4. **Documentation**: Setup instructions and API usage

### Evaluation Criteria
- **Correctness** (30%): Does it generate valid actions?
- **Robustness** (25%): Handles edge cases and errors?
- **Validation** (20%): Proper feasibility checking?
- **Usability** (15%): Clear API and documentation?
- **Performance** (10%): Reasonable latency?
