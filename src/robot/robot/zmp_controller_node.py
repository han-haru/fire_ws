import rclpy as rp
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration as rclpyDuration   # timeout용

imu 센서 sub

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class BaseBroad(Node):
# 실제 IMU센서를 읽어서 지지면이 움직이는 것을 확인하는 코드(준비물: IMU)

def __init__(self):  
    super().__init__("Basebroad_node")  

    # IMU구독용  
    self.imu_sub_base = self.create_subscription(Imu, "/base/imu/data",self.callback_imu_base,10)  
    self.qx = 0.0  
    self.qy = 0.0  
    self.qz = 0.0  
    self.qw = 1.0  
    self.imu_sub_track_R = self.create_subscription(Imu, "/track_right/imu/data", self.callback_imu_track_R,10)  
    self.qx_tr = 0.0  
    self.qy_tr = 0.0  
    self.qz_tr = 0.0  
    self.qw_tr = 1.0  
    self.imu_sub_track_L = self.create_subscription(Imu, "/track_left/imu/data", self.callback_imu_track_L,10)  
    self.qx_tl = 0.0  
    self.qy_tl = 0.0  
    self.qz_tl = 0.0  
    self.qw_tl = 1.0  
    # cmd_vel SUB  
    self.sub_cmd_vel = self.create_subscription(Twist,"/cmd_vel",self.callback_vel,10)  
    # motor state SUB  
    self.sub_motor_state = self.create_subscription(Twist,"/motor/cmd_vel", self.callback_motor_state,10)  
      
    time_period = 0.5 # 2HZ [모터의 디폴트 도달시간이 0.5초임으로 timer와 맞춤]  
    self.create_timer(time_period, self.re_cmd_vel)  
    # revised cmd_vel PUB  
    self.pub = self.create_publisher(Twist,"/revised/cmd_vel",10)  

    # broadcaster용  
    self.tf = tf2_ros.StaticTransformBroadcaster(self)  
    self.tf_base = tf2_ros.TransformBroadcaster(self)  
    self.tf_track_r = tf2_ros.TransformBroadcaster(self)  
    self.tf_track_l = tf2_ros.TransformBroadcaster(self)  
      
    #Marker용   
    self.tf_buffer = tf2_ros.Buffer()  
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  
    self.marker = self.create_publisher(Marker, 'visualization_marker', 10)  
      
    # ----- 속성값 --------  
    # 무게중심 위치  
    self.M = 25.0  
    x = -0.1  
    y = 0.0  
    h = 0.135  
    self.CoG_local = np.array([x, y, h])  
    # zmp service factor  
    self.sf_lin = 4.0 # default = 1.0  # 선형 가속도 zmp service factor  
    self.sf_ap = 2.0  # 회전 접선 가속도 zmp service factor  
    self.sf_cr = 5.0  # 구심력 zp service factor  
    # 제약(초기값)  
    self.xu = 0.295  
    self.xl = -0.295  
    self.yu = 0.309  
    self.yl = -0.309  
    # 속도,각속도,가속도,각가속도  
    self.v_max = 0.26  
    self.v_min = -0.26  
    self.a_max = 0.52  # 0.26m/s를 0.5초에 도달  
    self.a_min = -0.52  
    self.w_max = 1.04 # (v_r - v_l)/wheel width  
    self.w_min = -1.04  
    self.alpha_max = 2.08 # 1.04rad/s를 0.5초에 도달  
    self.alpha_min = -2.08  
    self.circle_ac_max = self.w_max*self.w_max*abs(x) # 구심력  
    self.del_t = 0.5    # re_cmd_vel의 주기와 동일  

    # 초기값  
    self.prev_vel = 0.0  
    self.prev_ang = 0.0  
    self.a_drive = 0.0  
    self.alpha_drive = 0.0  
    self.v_des = 0.0  
    self.w_des = 0.0  
    self.cmd_timeout = 2.0  
    self.last_cmd_time = self.get_clock().now()  

    # --- 모드 게이팅용 임계값/히스테리시스 ---  

    self.v_eps   = 0.02     # m/s, 직진 의도 최소치  
    self.w_eps   = 0.10     # rad/s, 선회 의도 최소치  
    self.mode_delta = 0.10  # 정규화 스코어 차이로 모드 결정 여유 (0~1)  
    self.motion_mode = "IDLE"   # ["IDLE","LINEAR","TURN"]  
    self.mode_hold_s = 0.5      # 모드 유지 최소 시간(플리커 방지)  
    self._last_mode_change = self.get_clock().now()  


    # static_broadcaster  
    transforms = []  
    transforms.append(self.robot_broad("CoG","base", -0.1, 0.0, 0.135))  
    transforms.append(self.robot_broad("base","robot_FL",  0.297, -0.25, 0.0))# 폭 500(휠간거리)+ 118(세그먼트 폭)mm, 길이 593.8mm  
    transforms.append(self.robot_broad("base","robot_FR",  0.297,  0.25, 0.0))  
    transforms.append(self.robot_broad("base","robot_RR", -0.297,  0.25, 0.0))  
    transforms.append(self.robot_broad("base","robot_RL", -0.297, -0.25, 0.0))  
    transforms.append(self.robot_broad("track_Right","trackR_FL", 0.297, -0.059, 0.0))  
    transforms.append(self.robot_broad("track_Right","trackR_FR", 0.297,  0.059, 0.0))  
    transforms.append(self.robot_broad("track_Right","trackR_RR",-0.297,  0.059, 0.0))  
    transforms.append(self.robot_broad("track_Right","trackR_RL",-0.297, -0.059, 0.0))  
    transforms.append(self.robot_broad("track_Left","trackL_FL", 0.297, -0.059, 0.0))  
    transforms.append(self.robot_broad("track_Left","trackL_FR", 0.297,  0.059, 0.0))  
    transforms.append(self.robot_broad("track_Left","trackL_RR",-0.297,  0.059, 0.0))  
    transforms.append(self.robot_broad("track_Left","trackL_RL",-0.297, -0.059, 0.0))  
    self.tf.sendTransform(transforms)  
    # 지지면 업데이트  
    self._bounds_initialized = False  
    self._bounds_ema_alpha = 0.3  # 0~1, 작을수록 부드럽게  
    self._bounds_margin = 0.0     # m, 보수적 여유가 필요하면 0.01~0.02 추천  

# ---- subscription ----  
def callback_imu_track_R(self,msg):  
    self.qx_tr = msg.orientation.x  
    self.qy_tr = msg.orientation.y  
    self.qz_tr = msg.orientation.z  
    self.qw_tr = msg.orientation.w  

def callback_imu_track_L(self,msg):  
    self.qx_tl = msg.orientation.x  
    self.qy_tl = msg.orientation.y  
    self.qz_tl = msg.orientation.z  
    self.qw_tl = msg.orientation.w  

def callback_imu_base(self,msg):  
    self.qx = msg.orientation.x  
    self.qy = msg.orientation.y  
    self.qz = msg.orientation.z  
    self.qw = msg.orientation.w  
    self.broad_base()  

def callback_motor_state(self,msg):  
    self.prev_vel = msg.linear.x  
    self.prev_ang = msg.angular.z  

def callback_vel(self,msg):  
    self.v_des = float(max(self.v_min, min(self.v_max, msg.linear.x)))  
    self.w_des = float(max(self.w_min, min(self.w_max, msg.angular.z)))  
    self.last_cmd_time = self.get_clock().now()  
# ---- end subscription ----  
# ---- broad ----   
def broad_base(self):                # tilted ground -> base (broadcaster)  

    t = TransformStamped()  
    t.header.stamp = self.get_clock().now().to_msg()  
    t.header.frame_id = "world"  
    t.child_frame_id = "base"  
    t.transform.translation.x = 0.0  
    t.transform.translation.y = 0.0  
    t.transform.translation.z = 0.0  
    t.transform.rotation.x = -self.qx  
    t.transform.rotation.y = self.qy  
    t.transform.rotation.z = self.qz  
    t.transform.rotation.w = self.qw  

    self.draw_marker("base","robtf", 1, -0.1, 0.0, 0.135, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 1.0, 0.0], 0.1)  
    self.broad_track_R()  
    self.broad_track_L()  
    self.draw_rectangle_marker()  
    self.draw_zmp_marker()  
        # ▼ 여기 추가: 트랙 꼭짓점들을 base로 정사영해서 xl/xu/yl/yu 갱신  
    self._update_support_polygon_bounds(  
        frames=["trackR_FL","trackR_FR","trackR_RR","trackR_RL",  
                "trackL_FL","trackL_FR","trackL_RR","trackL_RL"],  
        ema_alpha=self._bounds_ema_alpha,  
        margin=self._bounds_margin  
    )  
    self.tf_base.sendTransform(t)  

def broad_track_R(self):            

    t = TransformStamped()  
    t.header.stamp = self.get_clock().now().to_msg()  
    t.header.frame_id = "base"  
    t.child_frame_id = "track_Right"  
    t.transform.translation.x = 0.0  
    t.transform.translation.y = -0.25  
    t.transform.translation.z = 0.135  
    t.transform.rotation.x = -self.qy_tr  
    t.transform.rotation.y = -self.qx_tr  
    t.transform.rotation.z = 0.0  
    t.transform.rotation.w = self.qw_tr  

    self.tf_track_r.sendTransform(t)  
def broad_track_L(self):            

    t = TransformStamped()  
    t.header.stamp = self.get_clock().now().to_msg()  
    t.header.frame_id = "base"  
    t.child_frame_id = "track_Left"  
    t.transform.translation.x = 0.0  
    t.transform.translation.y = 0.25  
    t.transform.translation.z = 0.135  
    t.transform.rotation.x = -self.qy_tl  
    t.transform.rotation.y = -self.qx_tl  
    t.transform.rotation.z = 0.0  
    t.transform.rotation.w = self.qw_tl  

    self.tf_track_l.sendTransform(t)  

def robot_broad(self, parent, child, x, y, z):   # base -> support (static broadcaster)  
    t = TransformStamped()  
    t.header.stamp = self.get_clock().now().to_msg()  
    t.header.frame_id = parent  
    t.child_frame_id = child  
    t.transform.translation.x = x  
    t.transform.translation.y = y  
    t.transform.translation.z = z  
    t.transform.rotation.x = 0.0  
    t.transform.rotation.y = 0.0  
    t.transform.rotation.z = 0.0  
    t.transform.rotation.w = 1.0  
    return t  

def draw_marker(self, f_id,ns, marker_id, x, y, z, quat, marker_type, rgb, scale, lifetime=0):  
    m = Marker()  
    m.header.frame_id = f_id  
    m.header.stamp = self.get_clock().now().to_msg()  
    m.ns = ns                #namespace  
    m.id = marker_id         # id  
    m.type = marker_type  
    m.action = Marker.ADD  
    m.pose.position.x = x  
    m.pose.position.y = y  
    m.pose.position.z = z  
    m.pose.orientation.x = quat[0]  
    m.pose.orientation.y = quat[1]  
    m.pose.orientation.z = quat[2]  
    m.pose.orientation.w = quat[3]  
    m.scale.x = scale  
    m.scale.y = scale  
    m.scale.z = scale  
    m.color.r = rgb[0]  
    m.color.g = rgb[1]  
    m.color.b = rgb[2]  
    m.color.a = 1.0  
    m.lifetime = Duration(sec=lifetime)  
    self.marker.publish(m)  

def draw_rectangle_marker(self):  
    names = ["trackR_FL", "trackR_RL", "trackL_RR", "trackL_FR", "trackR_FL"]  # 직사각형   
    points = []  
    for name in names:  
        try:  
            tf = self.tf_buffer.lookup_transform("base", name, rp.time.Time(), timeout=rclpyDuration(seconds=0.2))  
            t = tf.transform.translation  
            points.append(Point(x=t.x, y=t.y, z=t.z))  
        except Exception as e:  
            self.get_logger().warn(f"TF lookup failed for {name}: {e}")  
            return  
    m = Marker()  
    m.header.frame_id = "base"  
    m.header.stamp = self.get_clock().now().to_msg()  
    m.ns = "rectangle"  
    m.id = 10  
    m.type = Marker.LINE_STRIP  
    m.action = Marker.ADD  
    m.scale.x = 0.02  
    m.color.r = 1.0  
    m.color.g = 1.0  
    m.color.b = 0.0  
    m.color.a = 1.0  
    m.points = points  
    m.pose.orientation.w = 1.0  
    self.marker.publish(m)  

def draw_zmp_marker(self):  
    # base frame기준으로 zmp 계산  
    g = 9.8  
    h = 0.135  
    M = 25.0  
    v = 0.0    # 최대속도 0.26m/s  
    w = 0.0    # 최대각속도 1.04rad/s  
    t = 0.5    # 도달시간 디폴드값  

    omega = np.array([0.0, 0.0, w])  
    alpha = np.array([0.0, 0.0, w / t])  
    CoG_a_local = np.array([v / t, 0.0, 0.0])  

    # 지면 기울기 반영 world에서 base의 좌표계 회전을 listen해서 th, psi갱신  

    try:  
        # world를 base기준으로 바라봄  
        tf = self.tf_buffer.lookup_transform("base", "world", rp.time.Time(), timeout=rclpyDuration(seconds=0.2))  
        rot = tf.transform.rotation  
        q = [rot.x, rot.y, rot.z, rot.w]  
        R = euler_from_quaternion(q)  
    except Exception as e:  
        self.get_logger().warn(f"TF lookup failed for world → base: {e}")  
        return  

    psi = R[0]  # Roll  
    th = R[1]   # pitch  
    yaw = R[2]  # yaw  

    R_pitch = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])  
    R_roll = np.array([[1, 0, 0], [0, math.cos(psi), -math.sin(psi)], [0, math.sin(psi), math.cos(psi)]])  
    R_yaw = np.array([[math.cos(yaw),-math.sin(yaw), 0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])  
    R_world_to_local = R_yaw @ R_pitch @ R_roll  # ros회전변환순서 ZYX  

    g_world = np.array([0.0, 0.0, g])  
    self.g_local = R_world_to_local @ g_world   
       
    # non_force zmp  
    self.x_nf = (M * self.g_local[2] * self.CoG_local[0] - M * self.g_local[0] * self.CoG_local[2]) / (M * self.g_local[2])  
    self.y_nf = (M * self.g_local[2] * self.CoG_local[1] - M * self.g_local[1] * self.CoG_local[2]) / (M * self.g_local[2])  
    #self.get_logger().info(f"g좌표: {self.g_local}")  

    # 구심가속도, 회전접선가속도  
    a_c_local = np.cross(omega, np.cross(omega, self.CoG_local))  
    circle_ac_local = np.cross(alpha,self.CoG_local)  
    # 다음 수식은 결과 값임으로 abs(self.g_local)을 사용해야함.  
    x_zmp_local = self.x_nf - h * (CoG_a_local[0] + a_c_local[0]) / abs(self.g_local[2])  
    y_zmp_local = self.y_nf - h * (CoG_a_local[1] + a_c_local[1] + circle_ac_local[1]) / abs(self.g_local[2])  
    zmp_local = np.array([x_zmp_local, y_zmp_local, 0.0])  
    self.draw_marker("base","zmp", 200, zmp_local[0], zmp_local[1], 0.0, [0., 0., 0., 1.], Marker.SPHERE, [0.0, 0.0, 1.0], 0.07)  
 
# ─────────────────────────────────────────────────────────────  
# ▼▼▼ 추가: 정사영 경계 갱신 함수들 (xu/xl/yu/yl 동적 업데이트) ▼▼▼  
# ─────────────────────────────────────────────────────────────  
def _lookup_xy_in_base(self, frame: str):  
    """frame(꼭짓점 프레임)의 위치를 base 프레임으로 가져와 (x,y)만 반환 (정사영)."""  
    try:  
        tf = self.tf_buffer.lookup_transform("base", frame, rp.time.Time(),  
                                             timeout=rclpyDuration(seconds=0.2))  
        t = tf.transform.translation  
        return float(t.x), float(t.y)  # z는 정사영이므로 무시  
    except Exception as e:  
        self.get_logger().warn(f"TF lookup failed for {frame}: {e}")  
        return None  

def _publish_bounds_marker(self, xl, xu, yl, yu, ns="proj_rect", mid=77):  
    """xl,xu,yl,yu로부터 직사각형 LINE_STRIP 마커 발행 (base z=0 정사영)."""  
    m = Marker()  
    m.header.frame_id = "base"  
    m.header.stamp = self.get_clock().now().to_msg()  
    m.ns = ns  
    m.id = mid  
    m.type = Marker.LINE_STRIP  
    m.action = Marker.ADD  
    m.scale.x = 0.02  
    m.color.r = 0.0  
    m.color.g = 1.0  
    m.color.b = 1.0  
    m.color.a = 1.0  
    m.pose.orientation.w = 1.0  

    corners = [(xu, yl), (xu, yu), (xl, yu), (xl, yl), (xu, yl)]  
    m.points = [Point(x=x, y=y, z=0.0) for (x,y) in corners]  
    self.marker.publish(m)  

def _update_support_polygon_bounds(self, frames, ema_alpha=0.3, margin=0.0):  
    """  
    frames: 꼭짓점 프레임 이름 리스트  
    ema_alpha: 0~1, 지수평활 계수 (작을수록 부드럽게)  
    margin: 경계 여유(m)  
    """  
    pts = []  
    for f in frames:  
        p = self._lookup_xy_in_base(f)  
        if p is not None:  
            pts.append(p)  

    if len(pts) < 3:  
        # 점이 부족하면 갱신 스킵  
        return False  

    xs = [p[0] for p in pts]  
    ys = [p[1] for p in pts]  
    xl_new = min(xs) - margin  
    xu_new = max(xs) + margin  
    yl_new = min(ys) - margin  
    yu_new = max(ys) + margin  

    if not self._bounds_initialized:  
        self.xl, self.xu, self.yl, self.yu = xl_new, xu_new, yl_new, yu_new  
        self._bounds_initialized = True  
    else:  
        self.xl = (1-ema_alpha)*self.xl + ema_alpha*xl_new  
        self.xu = (1-ema_alpha)*self.xu + ema_alpha*xu_new  
        self.yl = (1-ema_alpha)*self.yl + ema_alpha*yl_new  
        self.yu = (1-ema_alpha)*self.yu + ema_alpha*yu_new  

    # 시각화  
    self._publish_bounds_marker(self.xl, self.xu, self.yl, self.yu)  
    return True  
# ─────────────────────────────────────────────────────────────  
# ▲▲▲ 추가 끝 ▲▲▲  
# ─────────────────────────────────────────────────────────────  


  
# 선형 운동시 선형 가속도에 의한 zmp x축  
def lin_accel_bounds(self):  

    # 예외처리  
    try:   
        g_local = self.g_local  
        x_nf = self.x_nf  
    except AttributeError:  
        g_local = np.array([0.0,0.0,9.8])  
        x_nf = self.CoG_local[0]  

    # stability index  
    #   
    Sau = (1- (g_local[2]*(x_nf - self.xu))/(self.a_max*self.CoG_local[2]))/2  #음의 선형 가속도  
    Sal = (1+ (g_local[2]*(x_nf - self.xl))/(self.a_max*self.CoG_local[2]))/2  #양의 선형 가속도  

    # Sau,Sal이 1일 때가 zmp의 경계가 polygon에 접하는 순간  
    # 안전계수(self.sf)를 곱해 커스텀 가능  

    if Sau >= 1*self.sf_lin and Sal >= 1*self.sf_lin:  
        a_lower = self.a_min  
        a_upper = self.a_max  
        self.get_logger().info(f"[선형 안정], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")  
    elif Sau >0 and Sau <1*self.sf_lin and Sal >= self.sf_lin:  
        a_lower = (2/self.sf_lin)*(0.5*self.sf_lin-min(Sau,1*self.sf_lin))*self.a_max  
        a_upper = self.a_max  
        self.get_logger().warn(f"[선형 감속 제약], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")  
    elif Sau >= 1*self.sf_lin and Sal > 0 and Sal < 1*self.sf_lin :  
        a_lower = self.a_min  
        a_upper = (2/self.sf_lin)*(min(Sal,1*self.sf_lin)-0.5*self.sf_lin)*self.a_max  
        self.get_logger().warn(f"[선형 가속 제약], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")  
    elif Sau >0 and Sau < 1*self.sf_lin and Sal > 0 and Sal <1*self.sf_lin:  
        a_lower = (2/self.sf_lin)*(0.5*self.sf_lin-min(Sau,1*self.sf_lin))*self.a_max  
        a_upper = (2/self.sf_lin)*(min(Sal,1*self.sf_lin)-0.5*self.sf_lin)*self.a_max  
        self.get_logger().warn(f"[선형 가감속 제약], Sau: {Sau}, Sal: {Sal}, a_lower: {a_lower}, a_upper: {a_upper}")  
    else:  
        a_lower = 0.0  
        a_upper = 0.0  
        self.get_logger().warn(f"[전복], Sau: {Sau}, Sal: {Sal}")  
    return a_lower, a_upper, Sau, Sal  
  
# 회전운동시 회전접선가속도에 의한 y축 zmp  
def alpha_bounds(self):  
    # 예외처리  
    try:   
        g_local = self.g_local  
        y_nf = self.y_nf  
    except AttributeError:  
        g_local = np.array([0.0,0.0,9.8])  
        y_nf = self.CoG_local[1]  

    # stability index  
    Sapu = (1 - (g_local[2]*(y_nf - self.yu))/(self.alpha_max*self.CoG_local[2]))/2 # 양의 각가속도  
    Sapl = (1 + (g_local[2]*(y_nf - self.yl))/(self.alpha_max*self.CoG_local[2]))/2 # 음의 각가속도  

    if Sapu >= 1*self.sf_ap and Sapl >= 1*self.sf_ap:  
        alpha_lower = self.alpha_min  
        alpha_upper = self.alpha_max  
        self.get_logger().info(f"[회전 접선 안정], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")  
    elif Sapu >0 and Sapu <1*self.sf_ap and Sapl >= 1*self.sf_ap:  
        alpha_lower = self.alpha_min  
        alpha_upper = (2/self.sf_ap)*(min(Sapu,1*self.sf_ap)-0.5*self.sf_ap)*self.alpha_max  

        self.get_logger().warn(f"[양의 회전 접선 가속도 제약], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")  
    elif Sapu >= 1*self.sf_ap and Sapl > 0 and Sapl < 1*self.sf_ap :  
        alpha_lower = (2/self.sf_ap)*(0.5*self.sf_ap-min(Sapl,1*self.sf_ap))*self.alpha_max  
        alpha_upper = self.alpha_max  
        self.get_logger().warn(f"[음의 회전 접선 가속도 제약], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")  
    elif Sapu >0 and Sapu < 1*self.sf_ap and Sapl > 0 and Sapl <1*self.sf_ap:  
        alpha_lower = (2/self.sf_ap)*(0.5*self.sf_ap-min(Sapl,1*self.sf_ap))*self.alpha_max  
        alpha_upper = (2/self.sf_ap)*(min(Sapu,1*self.sf_ap)-0.5*self.sf_ap)*self.alpha_max  
        self.get_logger().warn(f"[양,음의회전 접선 가속도 제약], Sapu: {Sapu}, Sapl: {Sapl}, alpha_lower: {alpha_lower}, alpha_upper: {alpha_upper}")  
    else:  
        alpha_lower = 0.0  
        alpha_upper = 0.0  
        self.get_logger().warn(f"[전복], Sapu: {Sapu}, Sapl: {Sapl}")  
        w_lower,w_upper, Sw = 0.0,0.0,1.0  
        return w_lower, w_upper, Sw, alpha_lower, alpha_upper,Sapu, Sapl  
          
    w_lower,w_upper, Sw, _, _, _, _  = self.w_bounds(alpha_lower, alpha_upper, Sapu, Sapl)  
    return w_lower, w_upper, Sw, alpha_lower, alpha_upper, Sapu, Sapl  
  
# 회전운동시 구심가속도에 의한 x축 zmp  
def w_bounds(self,alpha_lower,alpha_upper,Sapu,Sapl):  

    # 예외처리  
    try:   
        g_local = self.g_local  
        x_nf = self.x_nf  
    except AttributeError:  
        g_local = np.array([0.0,0.0,9.8])  
        x_nf = self.CoG_local[0]  

    # stability index  
    Swl = (1+ (abs(g_local[2])*(x_nf - self.xl))/(self.circle_ac_max*self.CoG_local[2]))/2  
    Sw = Swl # 구심가속도는 하한에만 영향이 있음  


    if Sw >= 1*self.sf_cr:   
        circle_ac = self.circle_ac_max  
        self.get_logger().info(f"[회전 구심력 안정], Sw: {Sw},circle_ac: {circle_ac}")   
    elif Sw >0 and Sw <1*self.sf_cr:   
        circle_ac = max(0.0,(2/self.sf_cr)*(min(Sw,1*self.sf_cr)-0.5*self.sf_cr)*self.circle_ac_max)  
        self.get_logger().warn(f"[회전 속도 제약], Sw: {Sw},circle_ac: {circle_ac}")  
    else:  
        circle_ac = 0.0  
        self.get_logger().warn(f"[전복], Sw: {Sw}")  
    w_lower = max(self.w_min,-math.sqrt(circle_ac/abs(self.CoG_local[0])))  
    w_upper = min(self.w_max,math.sqrt(circle_ac/abs(self.CoG_local[0])))  
    return w_lower, w_upper, Sw, alpha_lower,alpha_upper,Sapu,Sapl  
def _select_motion_mode(self):  
    """/cmd_vel 의 v_des, w_des로부터 모드 결정 (히스테리시스 포함)"""  
    now = self.get_clock().now()  
    hold = (now - self._last_mode_change).nanoseconds * 1e-9 < self.mode_hold_s  

    vmag = abs(self.v_des)  
    wmag = abs(self.w_des)  

    if vmag < self.v_eps and wmag < self.w_eps:  
        new_mode = "IDLE"  
    else:  
        # 정규화 스코어 (0~1): 누가 '더 강한 명령'인지 비교  
        sv = vmag / max(self.v_max, 1e-6)  
        sw = wmag / max(self.w_max, 1e-6)  
        if sv - sw > self.mode_delta:  
            new_mode = "LINEAR"  
        elif sw - sv > self.mode_delta:  
            new_mode = "TURN"  
        else:  
            # 경계 영역에서는 기존 모드 유지(플리커 방지),  
            # 기존이 IDLE이면 우선순위: (TURN if sw>=sv else LINEAR)  
            if hold:  
                new_mode = self.motion_mode  
            else:  
                new_mode = "TURN" if sw >= sv else "LINEAR"  

    if new_mode != self.motion_mode:  
        self.motion_mode = new_mode  
        self._last_mode_change = now  
        self.get_logger().info(f"[mode] → {self.motion_mode}")  

def re_cmd_vel(self):  
    now = self.get_clock().now()  
    if (now - self.last_cmd_time).nanoseconds*1e-9 > self.cmd_timeout:  
        self.v_des = 0.0  
        self.w_des = 0.0  

    # 1) 모드 결정  
    self._select_motion_mode()  

    # 2) 선형 속도 처리 (모드에 따라 제약 분리)  
    a_cmd = (self.v_des - self.prev_vel) / self.del_t  

    if self.motion_mode in ("LINEAR", "IDLE"):  
        # ── LINEAR 모드: 선형 ZMP 제약만 적용 ──  
        a_lower, a_upper, Sau, Sal = self.lin_accel_bounds()  
        if Sau < 0 or Sal < 0:  
            ugv_lin_vel = 0.0  
        else:  
            self.a_drive = max(a_lower, min(a_upper, a_cmd))  
            ugv_lin_vel = self.prev_vel + self.a_drive * self.del_t  
        re_cmd_vel = max(self.v_min, min(self.v_max, ugv_lin_vel))  
    else:  
        # ── TURN 모드: 선형 제약 비활성(명령은 0으로 끌어내림만) ──  
        # 모터/안전상 급감 피하려면 a_max로만 감쇠  
        a_soft = max(self.a_min, min(self.a_max, (0.0 - self.prev_vel) / self.del_t))  
        ugv_lin_vel = self.prev_vel + a_soft * self.del_t  
        re_cmd_vel = max(self.v_min, min(self.v_max, ugv_lin_vel))  

    # 3) 각속도 처리 (모드에 따라 제약 분리)  
    alpha_cmd = (self.w_des - self.prev_ang) / self.del_t  

    if self.motion_mode == "TURN":  
        # ── TURN 모드: 회전(접선→구심) ZMP 제약 적용 ──  
        w_lower, w_upper, Sw, alpha_lower, alpha_upper, Sapu, Sapl = self.alpha_bounds()  
        if (Sapu <= 0) or (Sapl <= 0) or (Sw < 0):  
            re_ang_vel = 0.0  
        else:  
            self.alpha_drive = max(alpha_lower, min(alpha_upper, alpha_cmd))  
            ugv_ang_vel = self.prev_ang + self.alpha_drive * self.del_t  
            re_ang_vel = max(w_lower, min(w_upper, ugv_ang_vel))  
    else:  
        # ── LINEAR/IDLE 모드: 회전 ZMP 제약 비활성, 단 '0으로 감속'만 수행 ──  
        # (안전상 필요하면 여기서도 alpha_bounds()로 감속만 제한하도록 바꿔도 됨)  
        alpha_soft = max(self.alpha_min, min(self.alpha_max, (0.0 - self.prev_ang) / self.del_t))  
        ugv_ang_vel = self.prev_ang + alpha_soft * self.del_t  
        # TURN 모드가 아니므로 w 한계는 엔지니어링 리미트만 사용  
        re_ang_vel = max(self.w_min, min(self.w_max, ugv_ang_vel))  

    # 4) 퍼블리시  
    v = Twist()  
    v.linear.x = re_cmd_vel  
    v.angular.z = re_ang_vel  
    self.pub.publish(v)

def main(args=None):
rp.init(args=args)
node = BaseBroad()
try:
rp.spin(node)
except KeyboardInterrupt:
node.destroy_node()
rp.shutdown()

if name == "main":
main()

