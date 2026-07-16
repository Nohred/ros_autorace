from geometry_msgs.msg import Twist
import math


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TurnLogic:
    def __init__(
        self,
        node,
        direction='right',
        turn_angle_degrees=35,
        turn_speed=0.3,
        straight_speed=0.03,
        straight_seconds=3.5,
        turn_tolerance=0.10,
        approach_distance=0.10,
        approach_speed=0.05,
    ):
        self.node = node
        self.direction = direction
        self.turn_angle = math.radians(turn_angle_degrees)
        self.turn_speed = turn_speed
        self.straight_speed = straight_speed
        self.straight_seconds = straight_seconds
        self.turn_tolerance = turn_tolerance
        self.approach_distance = approach_distance
        self.approach_speed = approach_speed

        self.phase = 'APPROACH'
        self.start_time = None
        self.target_yaw = None
        self.last_error = 0.0

    def on_enter(self):
        self.start_time = self.node.get_clock().now()
        self.phase = 'APPROACH'
        self.last_error = 0.0
        self.target_yaw = None  # Se calcula al terminar el acercamiento, no antes

    def execute(self, sensor_data):
        cmd_vel = Twist()
        current_yaw = sensor_data.get('odom_yaw')

        # --- FASE 0: ACERCARSE HASTA 0.10m DE LA SEÑAL ---
        if self.phase == 'APPROACH':
            cmd_vel.linear.x = self.straight_speed
            cmd_vel.angular.z = 0.0

            elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds
            if elapsed >= self.straight_seconds * 1e9:
                self.phase = 'TURN'
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
            # Calculamos el yaw objetivo justo AHORA, ya frente a la señal
            base_yaw = current_yaw if current_yaw is not None else 0.0
            if self.direction == 'left':
                self.target_yaw = normalize_angle(base_yaw + self.turn_angle + math.radians(15))
            else:
                self.target_yaw = normalize_angle(base_yaw - self.turn_angle)

        

        # --- FASE 1: GIRO ---
        elif self.phase == 'TURN':
            cmd_vel.linear.x = 0.03
            cmd_vel.angular.z = self.turn_speed if self.direction == 'left' else -self.turn_speed

            if current_yaw is not None:
                error = normalize_angle(self.target_yaw - current_yaw)
                self.last_error = error
                if abs(error) <= self.turn_tolerance:
                    self.phase = 'STRAIGHT'
                    self.start_time = self.node.get_clock().now()
                    cmd_vel.linear.x = self.straight_speed
                    cmd_vel.angular.z = 0.0

        # --- FASE 2: AVANZAR RECTO DESPUES DE GIRAR ---
        elif self.phase == 'STRAIGHT':
            cmd_vel.linear.x = self.straight_speed
            cmd_vel.angular.z = 0.0

            elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds
            if elapsed >= self.straight_seconds * 1e9:
                return True, Twist()

        return False, cmd_vel