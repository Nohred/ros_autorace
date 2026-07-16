# turtlebot3_autorace_mission/states.py
from geometry_msgs.msg import Twist
import math
import subprocess
from .turn_logic import TurnLogic, normalize_angle


class State:
    def __init__(self, node):
        self.node = node

    def on_enter(self):
        pass

    def execute(self, sensor_data):
        return self, Twist()


# --- ESTADO 0: ESPERAR SEMAFORO ---
class WaitGreenLight(State):
    def __init__(self, node):
        super().__init__(node)
        self.green_start_time = None
        self.stop_logged = False
        self.GREEN_DURATION = 0.5  # segundos de verde continuo para arrancar

    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: ESPERANDO LUZ VERDE ---")
        self.green_start_time = None
        self.node.sync_detectors_for_step(0)

    def execute(self, sensor_data):
        cmd_vel = Twist()
        light_color = sensor_data['light_color']

        if light_color == 3:
            self.stop_logged = False
            if self.green_start_time is None:
                self.green_start_time = self.node.get_clock().now()

            elapsed = (self.node.get_clock().now() - self.green_start_time).nanoseconds / 1e9
            if elapsed >= self.GREEN_DURATION:
                return FollowLane(self.node), cmd_vel
        else:
            if light_color in (1, 2) and not self.stop_logged:
                self.node.get_logger().info("--- SEMAFORO EN ROJO/AMARILLO: DETENIDO ---")
                self.stop_logged = True
            self.green_start_time = None  # Se corto la racha de verde, reiniciar

        return self, cmd_vel


# --- ESTADO 1: SEGUIR LINEA (Router Central) ---
class FollowLane(State):
    def __init__(self, node):
        super().__init__(node)
        self.green_start_time = None
        self.stop_logged = False
        self.GREEN_DURATION = 0.5

        # Cronometro de "visto desde" por cada señal, independiente entre ellas
        self.sign_seen_since = {
            'sign_intersection': None,
            'sign_construction': None,
            'sign_parking': None,
            'sign_level': None,
            'sign_tunnel': None,
        }
        self.sign_score = {}  # Score acumulado para cada señal, independiente entre ellas

        # Duracion minima (segundos) que cada señal debe verse SIN interrupcion
        self.REQUIRED_DURATION = {
            'sign_intersection': 2.0,
            'sign_construction': 0.5,
            'sign_parking': 2.5,
            'sign_level': 1.5,
            'sign_tunnel': 2.0,
        }

        # Tiempo maximo sin recibir un mensaje NUEVO antes de considerar "ya no la veo"
        self.FRESHNESS_TIMEOUT = 0.5

    def on_enter(self):
        self.node.get_logger().info(
            f"--- ESTADO: SIGUIENDO LINEA (Esperando mision #{self.node.mission_step}) ---"
        )
        self.node.sync_detectors_for_step(self.node.mission_step)

    def _check_sign(self, sensor_data, topic_key, expected_value):
        now = self.node.get_clock().now()
        last_seen = self.node.last_sign_time[topic_key]
        elapsed_since_last_msg = (now - last_seen).nanoseconds / 1e9

        is_currently_visible = elapsed_since_last_msg < self.FRESHNESS_TIMEOUT
        topic_value = sensor_data[topic_key]
        matches = is_currently_visible and topic_value == expected_value

        # Inicializar el "score" acumulado si no existe
        if topic_key not in self.sign_score:
            self.sign_score[topic_key] = 0.0

        if matches:
            # Sube rapido: 1 "punto" de tiempo real transcurrido
            self.sign_score[topic_key] += 0.05  # aprox tu periodo de timer
        else:
            # Baja lento: castiga menos que lo que premia
            self.sign_score[topic_key] = max(0.0, self.sign_score[topic_key] - 0.02)

        required = self.REQUIRED_DURATION[topic_key]
        if self.sign_score[topic_key] >= required:
            self.sign_score[topic_key] = 0.0
            return True
        return False

    def execute(self, sensor_data):
        step = self.node.mission_step
        self.node.sync_detectors_for_step(step)

        # PASO 0: Esperar semaforo en verde continuo antes de iniciar ruta
        if step == 0:
            cmd_vel = Twist()
            light_color = sensor_data['light_color']

            if light_color == 3:
                self.stop_logged = False
                if self.green_start_time is None:
                    self.green_start_time = self.node.get_clock().now()

                elapsed = (self.node.get_clock().now() - self.green_start_time).nanoseconds / 1e9
                if elapsed >= self.GREEN_DURATION:
                    self.node.get_logger().info("Luz verde confirmada. Iniciando misiones de FOLLOW_LANE.")
                    self.node.mission_step = 1
                    self.node.sync_detectors_for_step(self.node.mission_step)
                    self.green_start_time = None
            else:
                if light_color in (1, 2) and not self.stop_logged:
                    self.node.get_logger().info("--- SEMAFORO EN ROJO/AMARILLO: DETENIDO ---")
                    self.stop_logged = True
                self.green_start_time = None

            return self, cmd_vel

        # PRIORIDAD MAXIMA: Construccion siempre interrumpe (seguridad fisica)
        if sensor_data['avoid_active'] and self.node.mission_step < 3:
            return AvoidConstruction(self.node), Twist()

        # PASO 1: Interseccion -> Giro a la derecha (valor 3)
        if step == 1 and self._check_sign(sensor_data, 'sign_intersection', 3):
            self.node.mission_step = 2
            return IntersectionRight(self.node), Twist()

        # PASO 2: Construccion 
        elif step == 2 and self._check_sign(sensor_data, 'sign_construction', 1):
            self.node.get_logger().info("Senal de CONSTRUCCION detectada.")
            self.node.mission_step = 3
  

        # PASO 3: Parking
        elif step == 3 and self._check_sign(sensor_data, 'sign_parking', 1):
            self.node.get_logger().info("Senal de PARKING detectada. Iniciando rutina de estacionamiento...")
            self.node.set_detector_enabled('avoid_construction', False)  # apagar aqui, no antes
            self.node.construction_cleared = True
            return ParkingMission(self.node), Twist()

        # PASO 4: Giro a la izquierda
        elif step == 4 and self._check_sign(sensor_data, 'sign_intersection', 2):
            self.node.mission_step = 5
            return IntersectionLeft(self.node), Twist()

        # PASO 5: Approach Level Crossing -> Esperar barrera abierta
        elif step == 5 and self._check_sign(sensor_data, 'sign_level', 1):
            self.node.get_logger().info("Senal de CRUCE FERROVIARIO detectada.")
            self.node.mission_step = 6
            return ApproachingLevelCrossing(self.node), sensor_data['lane_cmd']

        # PASO 6: Level Crossing -> Esperar barrera abierta
        elif step == 6:
            self.node.get_logger().info("Nodo de CRUCE FERROVIARIO iniciado.")
            return LevelCrossingMission(self.node), Twist()

        # PASO 7: Tunnel
        elif step == 7 and self._check_sign(sensor_data, 'sign_tunnel', 1):
            self.node.mission_step = 8
            self.node.get_logger().info("Senal de TUNEL detectada. Iniciando rutina de tunel...")
            return TunnelMission(self.node), Twist()

        return self, sensor_data['lane_cmd']


# --- ESTADO: ESQUIVAR CONSTRUCCION ---
class AvoidConstruction(State):
    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: ESQUIVANDO CONSTRUCCION ---")
        # if self.node.mission_step < 3:
        #     self.node.mission_step = 3
        if sensor_data['avoid_active'] and not self.node.construction_cleared:
            return AvoidConstruction(self.node), Twist()

    def execute(self, sensor_data):
        if not sensor_data['avoid_active']:
            self.node.construction_cleared = True
            return FollowLane(self.node), Twist()
        return self, sensor_data['avoid_cmd']

# --- ESTADO: PARKING ---
class ParkingMission(State):
    def __init__(self, node):
        super().__init__(node)
        self.phase = 'ALIGN'
        self.start_time = None
        self.target_yaw = None
        self.original_yaw = None
        self.last_error = 0.0

        self.ALIGN_SECONDS = 17
        self.TURN_ANGLE_DEG = 93.0
        self.PARK_SECONDS = 28.0
        self.WAIT_SECONDS = 5.0
        self.RETURN_SECONDS = 10.0
        self.RETURN_TURN_ANGLE_DEG = 186.0
        self.TURN_TOLERANCE = 0.08

        self.forward_speed = 0.04
        self.turn_speed = 0.3

    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: MISION PARKING INICIADA ---")
        self.node.disable_all_detectors()
        self.phase = 'ALIGN'
        self.start_time = self.node.get_clock().now()
        self.original_yaw = self.node.sensor_data.get('odom_yaw') or 0.0

    def _elapsed(self):
        return (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9

    def execute(self, sensor_data):
        cmd_vel = Twist()
        current_yaw = sensor_data.get('odom_yaw')
        if current_yaw is None:
            current_yaw = self.original_yaw

        if self.phase == 'ALIGN':
            cmd_vel.linear.x = self.forward_speed
            cmd_vel.angular.z = 0.0
            if self._elapsed() >= self.ALIGN_SECONDS:
                self.target_yaw = normalize_angle(current_yaw + math.radians(self.TURN_ANGLE_DEG))
                self.phase = 'TURN_IN'
                self.last_error = 0.0

        elif self.phase == 'TURN_IN':
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.turn_speed
            error = normalize_angle(self.target_yaw - current_yaw)
            if abs(error) <= self.TURN_TOLERANCE:
                self.phase = 'PARK'
                self.start_time = self.node.get_clock().now()

        elif self.phase == 'PARK':
            cmd_vel.linear.x = self.forward_speed
            cmd_vel.angular.z = 0.0
            if self._elapsed() >= self.PARK_SECONDS:
                self.phase = 'WAIT'
                self.start_time = self.node.get_clock().now()

        elif self.phase == 'WAIT':
            cmd_vel = Twist()
            if self._elapsed() >= self.WAIT_SECONDS:
                self.target_yaw = normalize_angle(current_yaw + math.radians(self.RETURN_TURN_ANGLE_DEG))
                self.phase = 'TURN_OUT'
                self.start_time = self.node.get_clock().now()
                self.last_error = 0.0
                self.node.get_logger().info("Saliendo del estacionamiento con giro de 180 grados...")

        elif self.phase == 'TURN_OUT':
            cmd_vel.linear.x = 0.0
            error = normalize_angle(self.target_yaw - current_yaw)
            cmd_vel.angular.z = self.turn_speed if error > 0.0 else -self.turn_speed
            self.last_error = error
            if abs(error) <= self.TURN_TOLERANCE:
                self.phase = 'RETURN'
                self.start_time = self.node.get_clock().now()
                cmd_vel.linear.x = self.forward_speed
                cmd_vel.angular.z = 0.0

        elif self.phase == 'RETURN':
            cmd_vel.linear.x = self.forward_speed
            cmd_vel.angular.z = 0.0
            if self._elapsed() >= self.RETURN_SECONDS:
                self.node.get_logger().info("Parking completado. Regresando a FOLLOW_LANE.")
                self.node.mission_step = 4
                self.node.get_logger().info("Mision completada. Regresando a FOLLOW_LANE.")
                return FollowLane(self.node), Twist()

        return self, cmd_vel

# --- ESTADO: INTERSECCION (Giro a la derecha) ---
class IntersectionRight(State):
    def __init__(self, node):
        super().__init__(node)
        self.turn_logic = TurnLogic(node, direction='right')

    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: GIRANDO EN INTERSECCION (DERECHA) ---")
        self.node.disable_all_detectors()
        self.turn_logic.on_enter()

    def execute(self, sensor_data):
        done, cmd_vel = self.turn_logic.execute(sensor_data)
        if done:
            return FollowLane(self.node), cmd_vel
        return self, cmd_vel

# --- ESTADO: GIRO IZQUIERDA + STOP ---
class IntersectionLeft(State):
    def __init__(self, node):
        super().__init__(node)
        self.turn_logic = TurnLogic(node, direction='left')

    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: GIRO IZQUIERDA + STOP ---")
        self.node.disable_all_detectors()
        self.turn_logic.on_enter()

    def execute(self, sensor_data):
        done, cmd_vel = self.turn_logic.execute(sensor_data)
        if done:
            return FollowLaneTimed(self.node, duration_seconds=15.0, next_step=5), Twist()
        return self, cmd_vel

# --- ESTADO: ACERCANDOSE AL CRUCE DE FERROCARRIL ---
class ApproachingLevelCrossing(State):
    def __init__(self, node):
        super().__init__(node)
        self.start_time = None
        self.APPROACH_SECONDS = 14.25  # tiempo para llegar frente al cruce

    def on_enter(self):
        self.node.get_logger().info(
            "--- ESTADO: ACERCANDOSE AL CRUCE (avanzando %.1fs) ---" % self.APPROACH_SECONDS
        )
        self.start_time = self.node.get_clock().now()

    def execute(self, sensor_data):
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        cmd_vel = sensor_data['lane_cmd']  # sigue la linea con el PID normal

        if elapsed >= self.APPROACH_SECONDS:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            return LevelCrossingMission(self.node), cmd_vel

        return self, cmd_vel

# --- ESTADO: SEGUIR LINEA POR TIEMPO ---
class FollowLaneTimed(State):
    def __init__(self, node, duration_seconds, next_step=None):
        super().__init__(node)
        self.duration_seconds = duration_seconds
        self.next_step = next_step
        self.start_time = None

    def on_enter(self):
        self.node.get_logger().info(
            f"--- ESTADO: SIGUIENDO LINEA POR {self.duration_seconds}s ---"
        )
        self.start_time = self.node.get_clock().now()

    def execute(self, sensor_data):
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9
        cmd_vel = sensor_data['lane_cmd']

        if elapsed >= self.duration_seconds:
            if self.next_step is not None:
                self.node.mission_step = self.next_step
            self.node.get_logger().info("Tiempo de seguimiento completado. Regresando a FollowLane.")
            return FollowLane(self.node), cmd_vel

        return self, cmd_vel

# --- ESTADO: CRUCE DE FERROCARRIL ---
class LevelCrossingMission(State):
    def __init__(self, node):
        super().__init__(node)
        self.open_score = 0.0
        self.CONFIRM_OPEN_SECONDS = 3.0
        self.DECAY_FACTOR = 0.5  # penaliza flicker sin resetear todo
        self._last_check_time = None

    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: CRUCE DE FERROCARRIL (esperando barrera) ---")
        # self.node.disable_all_detectors()
        self.node.sync_detectors_for_step(self.node.mission_step)
        self.open_score = 0.0
        self._last_check_time = None

    def execute(self, sensor_data):
        cmd_vel = Twist()
        level_state = sensor_data.get('level_crossing_state', 0)
        # Print log
        self.node.get_logger().info(f"Estado del cruce: {level_state}")

        now = self.node.get_clock().now()
        if self._last_check_time is None:
            dt = 0.05  # primer ciclo, asume periodo nominal
        else:
            dt = (now - self._last_check_time).nanoseconds / 1e9
        self._last_check_time = now

        if level_state == 3:  # OPEN - via libre
            self.open_score += dt
            cmd_vel = sensor_data['lane_cmd']
        else:
            # 0, 1 o 2: penaliza pero no resetea de golpe (tolera flicker)
            self.open_score = max(0.0, self.open_score - dt * self.DECAY_FACTOR)
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        if self.open_score >= self.CONFIRM_OPEN_SECONDS:
            self.node.get_logger().info("Cruce despejado. Regresando a FOLLOW_LANE.")
            self.node.mission_step += 1
            return FollowLane(self.node), cmd_vel

        return self, cmd_vel

# --- ESTADO: TUNEL (Placeholder, aqui iria Nav2/SLAM) ---
class TunnelMission(State):
    def __init__(self, node):
        super().__init__(node)
        self.phase = 'FOLLOW_LANE_BRIEF'
        self.start_time = None
        self.FOLLOW_SECONDS = 10.0
        self.FORWARD_SECONDS = 9.0
        self.forward_speed = 0.05
        self.tunnel_process = None

    def on_enter(self):
        self.node.get_logger().info("--- ESTADO: MISION TUNEL INICIADA ---")
        self.phase = 'FOLLOW_LANE_BRIEF'
        self.start_time = self.node.get_clock().now()
        self.tunnel_process = None
        # Aun no desactivamos detectores: seguimos usando lane_cmd un momento mas

    def execute(self, sensor_data):
        cmd_vel = Twist()
        elapsed = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.phase == 'FOLLOW_LANE_BRIEF':
            cmd_vel = sensor_data['lane_cmd']  # sigue la linea normalmente

            if elapsed >= self.FOLLOW_SECONDS:
                self.node.get_logger().info("Fin de seguimiento breve. Avanzando recto hacia el tunel...")
                self.node.disable_all_detectors()
                self.phase = 'FORWARD'
                self.start_time = self.node.get_clock().now()

        elif self.phase == 'FORWARD':
            cmd_vel.linear.x = self.forward_speed
            cmd_vel.angular.z = 0.0

            if elapsed >= self.FORWARD_SECONDS:
                self.node.get_logger().info("Pose ideal alcanzada. Lanzando mission_tunnel...")
                self.phase = 'LAUNCHING'
                # self.phase = 'RUNNING'

        elif self.phase == 'LAUNCHING':
            cmd_vel = Twist()
            if self.tunnel_process is None:
                self.tunnel_process = subprocess.Popen([
                    'ros2', 'launch', 'turtlebot3_autorace_mission',
                    'mission_tunnel.launch.py'
                ])
                self.node.get_logger().info(
                    "Nodo de mision tunel lanzado (pid=%d)" % self.tunnel_process.pid
                )
                self.phase = 'RUNNING'

        elif self.phase == 'RUNNING':
            cmd_vel = Twist()  # el nodo externo toma control de /cmd_vel

        return self, cmd_vel