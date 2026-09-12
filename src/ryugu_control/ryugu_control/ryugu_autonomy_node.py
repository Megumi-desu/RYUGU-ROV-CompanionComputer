#!/usr/bin/env python3
"""
ryugu_autonomy_node.py — Visual Servoing & Mode Switcher untuk RYUGU ROV

Node ini memiliki 2 fungsi utama:
1. Mode Switcher: Membaca input RC (Channel 7) dari remote. Jika ditekan/tinggi, 
   ia akan memanggil service MAVROS untuk mengubah mode terbang ke GUIDED (Autonomous).
   Jika rendah, kembali ke MANUAL.
2. Visual Servoing: Jika ROV berada di mode GUIDED, node ini akan membaca koordinat
   target dari kamera (hook_target) dan mengirim perintah kecepatan (Twist) ke Pixhawk
   agar ROV bergerak mengejar target.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class RyuguAutonomyNode(Node):
    def __init__(self):
        super().__init__('ryugu_autonomy_node')

        # ── Parameter ─────────────────────────────────────────────────
        # Sesuaikan channel remote yang dipakai untuk switch mode (0-based index)
        # 6 berarti Channel 7 di remote/QGC
        self.declare_parameter('auto_switch_channel', 6) 
        self.declare_parameter('rc_threshold', 1500) # Batas sinyal PWM (high/low)
        
        self.auto_ch = self.get_parameter('auto_switch_channel').value
        self.rc_threshold = self.get_parameter('rc_threshold').value

        # Status Internal
        self.current_mode = ''
        self.is_armed = False
        self.target_detected = False
        self.target_cx = 0.0
        self.target_cy = 0.0
        
        # Resolusi kamera (dari webcam_streamer/hook_detection)
        self.cam_width = 640.0
        self.cam_height = 640.0

        # ── Service Client (Ubah Mode) ────────────────────────────────
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ── Subscribers ───────────────────────────────────────────────
        # Pantau mode saat ini dari Pixhawk
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        
        # Pantau input remote kontrol (joystick/RC)
        self.rc_sub = self.create_subscription(
            RCIn, '/mavros/rc/in', self.rc_cb, qos_profile_sensor_data)
        
        # Terima koordinat target dari hook_detection_node
        self.vision_sub = self.create_subscription(
            Float32MultiArray, '/ryugu/vision/hook_target', self.vision_cb, 10)

        # ── Publishers ────────────────────────────────────────────────
        # Kirim perintah kecepatan pergerakan ROV
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Timer untuk kontrol pergerakan (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Ryugu Autonomy Node aktif. Menunggu mode GUIDED...')

    # ═══════════════════════════════════════════════════════════════════
    #  Callbacks
    # ═══════════════════════════════════════════════════════════════════
    def state_cb(self, msg: State):
        self.current_mode = msg.mode
        self.is_armed = msg.armed

    def rc_cb(self, msg: RCIn):
        """Mengecek tombol di remote untuk mengganti mode terbang."""
        if len(msg.channels) > self.auto_ch:
            pwm_value = msg.channels[self.auto_ch]
            
            # Jika sakelar dinaikkan (PWM > 1500) dan belum di mode GUIDED
            if pwm_value > self.rc_threshold and self.current_mode != 'GUIDED':
                self.change_mode('GUIDED')
            
            # Jika sakelar diturunkan (PWM < 1500) dan sedang di mode GUIDED
            elif pwm_value < self.rc_threshold and self.current_mode == 'GUIDED':
                self.change_mode('MANUAL')

    def vision_cb(self, msg: Float32MultiArray):
        """Menyimpan data target terbaru dari sistem AI."""
        if len(msg.data) >= 8:
            self.target_cx = float(msg.data[1])
            self.target_cy = float(msg.data[2])
            # Index 7 adalah detected_flag dari programmu sebelumnya
            self.target_detected = (msg.data[7] >= 0.5)

    # ═══════════════════════════════════════════════════════════════════
    #  Fungsi Mode & Kontrol (Proportional Controller)
    # ═══════════════════════════════════════════════════════════════════
    def change_mode(self, custom_mode: str):
        """Memanggil service MAVROS untuk mengubah mode terbang Pixhawk."""
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /mavros/set_mode tidak tersedia!')
            return

        req = SetMode.Request()
        req.custom_mode = custom_mode
        future = self.set_mode_client.call_async(req)
        self.get_logger().info(f'Meminta pergantian mode ke: {custom_mode}...')

    def control_loop(self):
        """Loop kontrol otonom berjalan pada 10 Hz."""
        cmd = Twist()

        # Hanya bergerak otomatis JIKA armed dan di mode GUIDED
        if not self.is_armed or self.current_mode != 'GUIDED':
            return

        if self.target_detected:
            # 1. Hitung Error (Titik tengah layar dikurangi posisi target)
            center_x = self.cam_width / 2.0
            center_y = self.cam_height / 2.0
            
            error_x = center_x - self.target_cx  # Kiri/Kanan (Yaw)
            error_y = center_y - self.target_cy  # Atas/Bawah (Depth)

            # 2. Proportional Gain (Kp) - Sesuaikan angka ini saat tuning di kolam!
            Kp_yaw = 0.002
            Kp_z = 0.002
            forward_speed = 0.3 # Kecepatan maju konstan (m/s) saat target terlihat

            # 3. Hitung kecepatan
            cmd.angular.z = error_x * Kp_yaw  # Rotasi ke arah target
            cmd.linear.z = error_y * Kp_z     # Menyelam/naik ke target
            cmd.linear.x = forward_speed      # Bergerak maju perlahan

            self.get_logger().debug(f'Mengejar Target! Yaw: {cmd.angular.z:.2f}, Z: {cmd.linear.z:.2f}')
        else:
            # Jika target hilang, ROV berhenti / hover
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
            self.get_logger().debug('Target hilang, ROV hover menunggu target...')

        # Kirim perintah kecepatan ke Pixhawk
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RyuguAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()