# Physics Validation

Ensuring that your robot simulation accurately represents real-world physics is critical for successful sim-to-real transfer. This lesson covers techniques for validating and tuning physics parameters in your simulation environments.

## Why Physics Validation Matters

The "reality gap" between simulation and real-world behavior can cause:

- **Controller Failure**: Controllers that work in simulation fail on real hardware
- **Unsafe Behavior**: Robots that are stable in simulation become unstable in reality
- **Training Inefficiency**: RL policies trained in poor simulations don't transfer
- **Wasted Resources**: Time spent debugging hardware when the issue is in simulation

Proper physics validation minimizes these risks.

## Key Physics Parameters

### 1. Mass and Inertia

**Why It Matters**: Incorrect mass distribution affects dynamics, balance, and energy consumption.

**Validation Process**:

```python
# Check mass properties in simulation
def validate_mass_properties(robot_urdf, real_mass_kg):
    """Compare simulated vs real mass"""
    # Parse URDF to get total mass
    sim_total_mass = calculate_urdf_mass(robot_urdf)

    error_percent = abs(sim_total_mass - real_mass_kg) / real_mass_kg * 100

    if error_percent > 5:
        print(f"⚠️ Mass error: {error_percent:.2f}%")
        print(f"Simulated: {sim_total_mass:.2f}kg, Real: {real_mass_kg:.2f}kg")
        return False
    else:
        print(f"✓ Mass validated: {error_percent:.2f}% error")
        return True

# Example usage
validate_mass_properties("humanoid.urdf", real_mass_kg=45.0)
```

**Measuring Real Inertia**:

For complex shapes, use the **bifilar pendulum** method:

```python
import numpy as np

def calculate_inertia_from_pendulum(mass_kg, length_m, period_s):
    """
    Calculate moment of inertia from bifilar pendulum test

    Args:
        mass_kg: Mass of the object
        length_m: Length of pendulum strings
        period_s: Measured oscillation period

    Returns:
        Moment of inertia (kg⋅m²)
    """
    g = 9.81  # m/s²
    I = (mass_kg * g * length_m * period_s**2) / (4 * np.pi**2)
    return I

# Example: Measured period of 2.3 seconds
real_inertia = calculate_inertia_from_pendulum(
    mass_kg=5.0,
    length_m=0.5,
    period_s=2.3
)
print(f"Measured inertia: {real_inertia:.4f} kg⋅m²")
```

### 2. Joint Friction and Damping

**Static vs Dynamic Friction**:

```xml
<!-- In URDF/SDF -->
<joint name="knee_joint" type="revolute">
  <dynamics damping="0.7" friction="0.5"/>
  <!-- damping: velocity-dependent resistance (N⋅m⋅s/rad) -->
  <!-- friction: static friction (N⋅m) -->
</joint>
```

**Validation Experiment**:

1. **Free Swing Test**: Release joint from angle and measure decay
2. **Compare with Simulation**: Tune damping until curves match

```python
import matplotlib.pyplot as plt
import numpy as np

def fit_damping_coefficient(time_data, angle_data, inertia):
    """
    Fit damping coefficient from free swing data

    Damped oscillation: θ(t) = A⋅e^(-bt)⋅cos(ωt + φ)
    """
    from scipy.optimize import curve_fit

    def damped_oscillation(t, A, b, omega, phi):
        return A * np.exp(-b * t) * np.cos(omega * t + phi)

    # Fit parameters
    popt, _ = curve_fit(damped_oscillation, time_data, angle_data)
    A, b, omega, phi = popt

    # Calculate damping coefficient
    damping = 2 * b * inertia

    return damping, popt

# Example data from real robot
time_real = np.linspace(0, 5, 100)
angle_real = np.load('real_joint_swing.npy')  # Your measured data

inertia = 0.05  # kg⋅m²
damping, params = fit_damping_coefficient(time_real, angle_real, inertia)

print(f"Optimal damping coefficient: {damping:.4f} N⋅m⋅s/rad")

# Plot comparison
plt.plot(time_real, angle_real, label='Real Robot')
# ... plot simulation data with tuned damping
plt.legend()
plt.show()
```

### 3. Contact Parameters

**Contact Model Parameters**:
- **Stiffness** (kp): How "hard" surfaces are
- **Damping** (kd): Energy dissipation at contact
- **Friction coefficients**: μ_static, μ_dynamic

**Validation: Drop Test**

```python
def validate_contact_stiffness(drop_height_m, bounce_height_m, mass_kg):
    """
    Estimate contact parameters from drop test

    Coefficient of restitution: e = sqrt(h_bounce / h_drop)
    """
    e = np.sqrt(bounce_height_m / drop_height_m)

    # For contact modeling
    print(f"Coefficient of restitution: {e:.3f}")

    if e < 0.1:
        print("→ Use high damping (kd > 100)")
    elif e > 0.7:
        print("→ Use low damping (kd < 50)")

    return e

# Example: Drop from 0.5m, bounces to 0.05m
validate_contact_stiffness(drop_height_m=0.5, bounce_height_m=0.05, mass_kg=2.0)
```

**Friction Validation: Incline Test**

```python
def measure_friction_coefficient(incline_angle_deg):
    """
    Measure static friction from incline test

    At the angle of slipping: μ_s = tan(θ)
    """
    theta_rad = np.deg2rad(incline_angle_deg)
    mu_static = np.tan(theta_rad)

    print(f"Static friction coefficient: {mu_static:.3f}")
    return mu_static

# Example: Object slips at 25 degrees
mu = measure_friction_coefficient(25)
```

Update simulation:

```xml
<!-- Gazebo/SDF -->
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>    <!-- static friction -->
      <mu2>0.6</mu2>  <!-- dynamic friction -->
    </ode>
  </friction>
  <contact>
    <ode>
      <kp>1000000</kp>  <!-- stiffness -->
      <kd>100</kd>      <!-- damping -->
    </ode>
  </contact>
</surface>
```

### 4. Actuator Models

Real motors have:
- **Torque limits**
- **Velocity limits**
- **Back-EMF** (torque decreases at high speed)
- **Gear ratios** and **transmission losses**

**Realistic Motor Model**:

```python
class DCMotorModel:
    """Simplified DC motor with back-EMF"""

    def __init__(self, stall_torque, no_load_speed, voltage):
        """
        Args:
            stall_torque: Maximum torque at 0 speed (N⋅m)
            no_load_speed: Maximum speed at 0 load (rad/s)
            voltage: Operating voltage (V)
        """
        self.tau_stall = stall_torque
        self.omega_max = no_load_speed
        self.V = voltage

    def get_torque(self, angular_velocity, voltage_cmd):
        """
        Calculate actual output torque

        Torque-speed relationship: τ = τ_stall * (1 - ω/ω_max) * (V_cmd/V_nom)
        """
        voltage_factor = voltage_cmd / self.V
        speed_factor = max(0, 1 - abs(angular_velocity) / self.omega_max)

        torque = self.tau_stall * speed_factor * voltage_factor

        return torque

# Example: Dynamixel MX-64 specs
motor = DCMotorModel(
    stall_torque=6.0,      # N⋅m
    no_load_speed=116.0,   # RPM → rad/s
    voltage=12.0
)

# At half speed, full voltage
torque = motor.get_torque(angular_velocity=58.0, voltage_cmd=12.0)
print(f"Output torque: {torque:.2f} N⋅m")
```

Integrate into simulation:

```python
# In Gazebo plugin or Isaac Sim controller
def apply_motor_command(joint, desired_torque, current_velocity):
    """Apply realistic motor limits"""

    # Get actual torque from motor model
    actual_torque = motor.get_torque(current_velocity, desired_torque)

    # Apply limits
    actual_torque = np.clip(actual_torque, -motor.tau_stall, motor.tau_stall)

    joint.set_torque(actual_torque)
```

## Validation Experiments

### 1. Trajectory Tracking

**Test**: Command a known trajectory, compare sim vs real

```python
def trajectory_tracking_test(sim_data, real_data):
    """
    Compare trajectory tracking performance

    Args:
        sim_data: (time, position) from simulation
        real_data: (time, position) from real robot
    """
    from scipy import interpolate

    # Interpolate real data to match sim timestamps
    interp_func = interpolate.interp1d(real_data['time'], real_data['position'])
    real_interp = interp_func(sim_data['time'])

    # Calculate RMSE
    rmse = np.sqrt(np.mean((sim_data['position'] - real_interp)**2))

    # Calculate correlation
    correlation = np.corrcoef(sim_data['position'], real_interp)[0, 1]

    print(f"RMSE: {rmse:.4f} rad")
    print(f"Correlation: {correlation:.4f}")

    # Plot
    plt.figure(figsize=(10, 5))
    plt.plot(sim_data['time'], sim_data['position'], label='Simulation')
    plt.plot(real_data['time'], real_data['position'], 'o', label='Real Robot')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Angle (rad)')
    plt.legend()
    plt.grid(True)
    plt.show()

    return rmse, correlation

# Example usage
rmse, corr = trajectory_tracking_test(sim_data, real_data)
if rmse < 0.05 and corr > 0.95:
    print("✓ Validation passed!")
else:
    print("⚠️ Significant deviation detected")
```

### 2. Energy Consumption

**Test**: Compare power consumption for same task

```python
def validate_energy_consumption(sim_power, real_power, duration):
    """
    Compare energy usage

    Power = Torque × Angular_Velocity
    """
    sim_energy = np.trapz(sim_power, dx=duration)
    real_energy = np.trapz(real_power, dx=duration)

    error = abs(sim_energy - real_energy) / real_energy * 100

    print(f"Simulated energy: {sim_energy:.2f} J")
    print(f"Real energy: {real_energy:.2f} J")
    print(f"Error: {error:.1f}%")

    return error < 20  # 20% tolerance

# Calculate power in simulation
def calculate_power(torques, velocities):
    """Power = τ · ω"""
    return np.sum(torques * velocities, axis=1)  # Sum over all joints
```

### 3. Impact Testing

**Test**: Drop robot or impact a surface, measure response

```python
def validate_impact_response(sim_impulse, real_impulse):
    """
    Compare impact forces and duration

    Impulse = ∫ F dt
    """
    sim_peak = np.max(sim_impulse)
    real_peak = np.max(real_impulse)

    sim_duration = len(sim_impulse[sim_impulse > 0.1 * sim_peak]) * 0.001  # dt = 1ms
    real_duration = len(real_impulse[real_impulse > 0.1 * real_peak]) * 0.001

    print(f"Peak force - Sim: {sim_peak:.1f}N, Real: {real_peak:.1f}N")
    print(f"Contact duration - Sim: {sim_duration:.3f}s, Real: {real_duration:.3f}s")

    # Both should be similar for good validation
    return abs(sim_peak - real_peak) / real_peak < 0.3
```

## Simulator-Specific Tuning

### Gazebo

```xml
<!-- Increase solver iterations for stability -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <max_contacts>20</max_contacts>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- Increase from default 20 -->
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

### Isaac Sim

```python
from omni.isaac.core.utils.physics import set_physics_scene_settings

# Tune physics parameters
set_physics_scene_settings(
    physics_dt=1.0/240.0,      # Higher frequency for accuracy
    rendering_dt=1.0/60.0,
    solver_type="TGS",          # Temporal Gauss-Seidel (more stable)
    num_position_iterations=8,  # Increase for accuracy
    num_velocity_iterations=1
)
```

### Unity

```csharp
// Set physics timestep
Time.fixedDeltaTime = 0.01f;  // 100Hz physics

// Solver iterations
Physics.defaultSolverIterations = 10;
Physics.defaultSolverVelocityIterations = 5;
```

## Domain Randomization

To bridge the remaining reality gap, use **domain randomization**:

```python
def randomize_physics_parameters():
    """
    Randomize physics to make RL policies more robust
    """
    import random

    # Randomize mass (±10%)
    mass_multiplier = random.uniform(0.9, 1.1)

    # Randomize friction (±30%)
    friction_multiplier = random.uniform(0.7, 1.3)

    # Randomize motor strength (±15%)
    motor_multiplier = random.uniform(0.85, 1.15)

    # Apply to robot
    robot.set_mass_scale(mass_multiplier)
    robot.set_friction_scale(friction_multiplier)
    robot.set_motor_scale(motor_multiplier)

    # Randomize timestep slightly
    dt = random.uniform(0.0008, 0.0012)  # ±20% around 0.001

    return {
        'mass': mass_multiplier,
        'friction': friction_multiplier,
        'motor': motor_multiplier,
        'dt': dt
    }
```

## Validation Checklist

Before deploying to real hardware:

- [ ] **Mass properties**: Total mass within 5% of real robot
- [ ] **Inertia tensors**: Validated with pendulum or CAD
- [ ] **Joint friction**: Free-swing test matches simulation
- [ ] **Contact stiffness**: Drop test restitution coefficient matches
- [ ] **Friction coefficients**: Incline test validates μ values
- [ ] **Motor limits**: Torque-speed curves match datasheets
- [ ] **Trajectory tracking**: RMSE < 5% for simple motions
- [ ] **Energy consumption**: Within 20% of real measurements
- [ ] **Timestep**: Small enough (< 1ms for contacts)
- [ ] **Solver iterations**: Sufficient for stability (> 20 iterations)

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Robot falls through floor | Collision not detected | Reduce timestep, increase iterations |
| Jittery motion | Stiff contacts, low damping | Increase contact damping (kd) |
| Simulation too slow | Timestep too small | Increase timestep (if stable) |
| Robot explodes | Mass/inertia errors | Check URDF, verify positive inertia |
| Drift over time | Numerical integration error | Use higher-order integrators |

## Further Reading

- [Physics-Based Simulation for Robot Learning](https://arxiv.org/abs/2109.05590)
- [Sim-to-Real Transfer Survey](https://arxiv.org/abs/2009.13303)
- [Gazebo Physics Parameters](http://gazebosim.org/tutorials?tut=physics_params)
- [Isaac Sim Physics](https://docs.omniverse.nvidia.com/isaacsim/latest/features/physics_settings.html)
