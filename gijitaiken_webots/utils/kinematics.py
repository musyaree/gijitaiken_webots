import math

class Kinematics:
    @staticmethod
    def calculate_ball_position_local(ball_pos, robot_pos, imu_yaw):
        dx_global = ball_pos[0] - robot_pos[0]
        dy_global = ball_pos[1] - robot_pos[1]
        
        local_x = dx_global * math.cos(imu_yaw) + dy_global * math.sin(imu_yaw)
        local_y = -dx_global * math.sin(imu_yaw) + dy_global * math.cos(imu_yaw)
        
        return local_x * 100.0, local_y * 100.0

    @staticmethod
    def calculate_vertical_fov(h_fov_rad, width, height):
        v_fov_rad = 2 * math.atan(math.tan(h_fov_rad / 2) * (height / width))
        return math.degrees(v_fov_rad)
    
    @staticmethod
    def quat_to_euler(x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z