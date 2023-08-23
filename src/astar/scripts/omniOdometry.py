import math

class OdometryPublisher:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, delta_x, delta_y, delta_theta):
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def get_odometry(self):
        return self.x, self.y, self.theta

def calculate_odometry_from_velocities(Vx, Vy, Vw, delta_t):
    delta_x = Vx * delta_t
    delta_y = Vy * delta_t
    delta_theta = Vw * delta_t

    return delta_x, delta_y, delta_theta
# Example values
Vx = 0.5  # meters per second along x-axis
Vy = 0.3  # meters per second along y-axis
Vw = math.radians(45)  # radians per second angular velocity
delta_t = 1.0  # seconds

odometry_publisher = OdometryPublisher()

# Simulate updating the odometry over time
for _ in range(10):
    delta_x, delta_y, delta_theta = calculate_odometry_from_velocities(Vx, Vy, Vw, delta_t)
    odometry_publisher.update(delta_x, delta_y, delta_theta)

# Get the accumulated odometry
final_odometry = odometry_publisher.get_odometry()
print("Final Odometry:", final_odometry)