import math

def get_trajectory_with_air_resistance(
    initial_speed,
    launch_angle_deg,
    mass,
    radius,
    air_density=1.225,  # kg/m^3 (at sea level)
    drag_coefficient=0.47,  # for a sphere
    dt=0.01,
):
    """
    Calculates the trajectory of a projectile with air resistance.

    Args:
        initial_speed (float): The initial speed of the projectile in m/s.
        launch_angle_deg (float): The launch angle in degrees.
        mass (float): The mass of the projectile in kg.
        radius (float): The radius of the projectile in meters.
        air_density (float): The density of the air.
        drag_coefficient (float): The drag coefficient of the projectile.
        dt (float): The time step for the simulation in seconds.

    Returns:
        A list of tuples, where each tuple contains (x, y) coordinates.
    """
    
    # Constants
    g = 9.81  # m/s^2

    # Convert angle from degrees to radians
    angle_rad = math.radians(launch_angle_deg)

    # Initial velocities (m/s)
    vx = initial_speed * math.cos(angle_rad)
    vy = initial_speed * math.sin(angle_rad)

    # Initial position (m)
    x = 0.0
    y = 0.0
    
    # Calculate cross-sectional area
    area = math.pi * radius**2

    # Trajectory data points
    trajectory = [(x, y)]

    # Simulation loop
    while y >= 0:
        # Calculate velocity magnitude
        speed = math.sqrt(vx**2 + vy**2)

        # Calculate drag force
        drag_force = 0.5 * air_density * area * drag_coefficient * speed**2
        
        # Determine acceleration components
        ax = -(drag_force / mass) * (vx / speed)
        ay = -g - (drag_force / mass) * (vy / speed)

        # Update velocities using the Euler method
        vx += ax * dt
        vy += ay * dt

        # Update positions
        x += vx * dt
        y += vy * dt

        # Store the current position
        if y >= 0:  # Only append if the ball is still above the ground
            trajectory.append((x, y))

    return trajectory

# --- Example Usage ---
# Simulate a baseball (approx. mass=0.145 kg, radius=0.037 m)
# thrown at 40 m/s at a 30-degree angle.

initial_speed = 40.0  # m/s
launch_angle = 30.0  # degrees
mass_ball = 0.145  # kg
radius_ball = 0.037  # m

ball_trajectory = get_trajectory_with_air_resistance(
    initial_speed, launch_angle, mass_ball, radius_ball
)

# You can now print the trajectory points or plot them
# For example, print the last point (where the ball lands)
final_position = ball_trajectory[-1]
print(f"The ball traveled a distance of {final_position[0]:.2f} meters.")