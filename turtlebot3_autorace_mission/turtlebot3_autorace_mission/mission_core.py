# turtlebot3_autorace_mission/mission_core.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import UInt8, Bool
import math
from .states import FollowLane
from sensor_msgs.msg import LaserScan




class MissionCore(Node):
    def __init__(self):
        super().__init__('mission_core')
        
        self.mission_step = 0  # 0=WaitGreenLight, 1=Interseccion, 2=Construccion, 3=Parking, 4=Giro Izq, 5=Level, 7=Tunnel

        self.detector_clients = {
            'traffic_light': self.create_client(
                SetParameters, '/detect_traffic_light/set_parameters'
            ),
            'sign_intersection': self.create_client(
                SetParameters, '/detect_intersection_sign/set_parameters'
            ),
            'sign_construction': self.create_client(
                SetParameters, '/detect_construction_sign/set_parameters'
            ),
            'sign_parking': self.create_client(
                SetParameters, '/detect_parking_sign/set_parameters'
            ),
            'sign_level': self.create_client(
                SetParameters, '/detect_level_crossing_sign/set_parameters'
            ),
            'sign_tunnel': self.create_client(
                SetParameters, '/detect_tunnel_sign/set_parameters'
            ),
            'avoid_construction': self.create_client(
                SetParameters, '/avoid_construction/set_parameters'
            ),
            'level_crossing': self.create_client(
                SetParameters, '/detect_level_crossing/set_parameters'
            ),

        }
        self.detector_enabled = {key: None for key in self.detector_clients}


        self.sensor_data = {
            'light_color': 0,
            'sign_intersection': 0,
            'sign_construction': 0,
            'sign_parking': 0,
            'sign_level': 0,
            'sign_tunnel': 0,
            'avoid_active': False,
            'odom_x': 0.0,
            'odom_y': 0.0,
            'odom_yaw': None,
            'lane_cmd': Twist(),
            'avoid_cmd': Twist(),
            'front_distance': float('inf'),
            'level_crossing_state': 0,
            'lane_state': 0,
        }

        # Agrega esto en tu __init__ de MissionCore
        self.last_sign_time = {
            'sign_intersection': self.get_clock().now(),
            'sign_construction': self.get_clock().now(),
            'sign_parking': self.get_clock().now(),
            'sign_level': self.get_clock().now(),
            'sign_tunnel': self.get_clock().now(),
        }
        
        self.current_state = FollowLane(self)
        self.current_state.on_enter()

        self.create_subscription(UInt8, '/detect/traffic_light', self.cb_light, 10)
        self.create_subscription(UInt8, '/detect/sign_intersection', self.cb_sign_intersection, 10)
        self.create_subscription(UInt8, '/detect/sign_construction', self.cb_sign_construction, 10)
        self.create_subscription(UInt8, '/detect/sign_parking', self.cb_sign_parking, 10)
        self.create_subscription(UInt8, '/detect/sign_level', self.cb_sign_level, 10)
        self.create_subscription(UInt8, '/detect/sign_tunnel', self.cb_sign_tunnel, 10)
        self.create_subscription(Bool, '/avoid/active', self.cb_avoid_flag, 10)
        self.create_subscription(Twist, '/avoid/control', self.cb_avoid_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(Twist, '/control/cmd_vel_lane', self.cb_lane, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.create_subscription(UInt8, '/detect/level_crossing_state', self.cb_level_state, 10)
        self.lane_state_sub = self.create_subscription(UInt8, '/detect/lane_state', self.lane_state_callback, 10)

        
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def set_detector_enabled(self, detector_name, enabled):
        if self.detector_enabled.get(detector_name) == enabled:
            return

        client = self.detector_clients[detector_name]
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.0):
            return

        request = SetParameters.Request()
        parameter_value = ParameterValue()
        parameter_value.type = ParameterType.PARAMETER_BOOL
        parameter_value.bool_value = enabled
        request.parameters = [Parameter(name='enabled', value=parameter_value)]
        client.call_async(request)
        self.detector_enabled[detector_name] = enabled

    def disable_all_detectors(self):
        for detector_name in self.detector_clients:
            self.set_detector_enabled(detector_name, False)


    def sync_detectors_for_step(self, step):
        profiles = {
            0: {'traffic_light': True},
            1: {'sign_intersection': True},
            2: {'sign_construction': True, 'avoid_construction': True},
            3: {'sign_parking': True, 'avoid_construction': True},  
            4: {'sign_intersection': True},
            5: {'sign_level': True, 'level_crossing': True},
            6: {'sign_level': True, 'level_crossing': True},
            7: {'sign_tunnel': True},
        }

        desired_state = {detector_name: False for detector_name in self.detector_clients}
        desired_state.update(profiles.get(step, {}))

        for detector_name, enabled in desired_state.items():
            self.set_detector_enabled(detector_name, enabled)

    def cb_light(self, msg): self.sensor_data['light_color'] = msg.data
    def cb_sign_intersection(self, msg):
        self.sensor_data['sign_intersection'] = msg.data
        self.last_sign_time['sign_intersection'] = self.get_clock().now()
    def cb_sign_construction(self, msg):
        self.sensor_data['sign_construction'] = msg.data
        self.last_sign_time['sign_construction'] = self.get_clock().now()
    def cb_sign_parking(self, msg):
        self.sensor_data['sign_parking'] = msg.data
        self.last_sign_time['sign_parking'] = self.get_clock().now()
    def cb_sign_level(self, msg):
        self.sensor_data['sign_level'] = msg.data
        self.last_sign_time['sign_level'] = self.get_clock().now()
    def cb_sign_tunnel(self, msg):
        self.sensor_data['sign_tunnel'] = msg.data
        self.last_sign_time['sign_tunnel'] = self.get_clock().now()
    def cb_avoid_flag(self, msg): 
        self.sensor_data['avoid_active'] = msg.data
    def cb_avoid_cmd(self, msg): 
        self.sensor_data['avoid_cmd'] = msg
    def cb_lane(self, msg): 
        self.sensor_data['lane_cmd'] = msg
    def cb_level_state(self, msg):
        self.sensor_data['level_crossing_state'] = msg.data
    def lane_state_callback(self, msg):
        self.sensor_data['lane_state'] = msg.data



    def cb_scan(self, msg):
        # Tomamos un cono estrecho de +/- 10 grados frente al robot (indice 0 = frente)
        cone_deg = 10
        ranges = msg.ranges
        n = len(ranges)
        cone_indices = list(range(0, cone_deg)) + list(range(n - cone_deg, n))
        
        valid = [ranges[i] for i in cone_indices if msg.range_min < ranges[i] < msg.range_max]
        
        if valid:
            self.sensor_data['front_distance'] = min(valid)
        else:
            self.sensor_data['front_distance'] = float('inf')

    

    def cb_odom(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.sensor_data['odom_x'] = msg.pose.pose.position.x
        self.sensor_data['odom_y'] = msg.pose.pose.position.y
        self.sensor_data['odom_yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        next_state, cmd_vel = self.current_state.execute(self.sensor_data)
        if type(next_state) != type(self.current_state):
            self.current_state = next_state
            self.current_state.on_enter()
        self.pub_cmd_vel.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MissionCore())
    rclpy.shutdown()

if __name__ == '__main__':
    main()