import os
import sys
import subprocess
import threading
import time


REQUIRED_PACKAGES = [
    ('pygame', 'pygame'),
    ('serial', 'pyserial'),
    ('pyglet', 'pyglet'),
    ('moderngl', 'moderngl'),
    ('pyrr', 'pyrr'),
]

def setup_environment():
    in_venv = sys.prefix != sys.base_prefix
    current_dir = os.path.dirname(os.path.abspath(__file__))
    venv_dir = os.path.join(current_dir, 'venv')
    if not in_venv:
        python_exe = os.path.join(venv_dir, 'Scripts', 'python.exe') if os.name == 'nt' else os.path.join(venv_dir, 'bin', 'python')
        if not os.path.exists(python_exe):
            subprocess.check_call([sys.executable, '-m', 'venv', venv_dir])
        try:
            sys.exit(subprocess.call([python_exe] + sys.argv))
        except KeyboardInterrupt:
            sys.exit(0)
    missing_packages = []
    for module_name, package_name in REQUIRED_PACKAGES:
        try:
            __import__(module_name)
        except ImportError:
            missing_packages.append(package_name)

    if missing_packages:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--upgrade', 'pip'])
        # 去重后统一安装，确保主控界面和 3D 演示依赖一次装齐
        install_list = sorted(set(missing_packages))
        subprocess.check_call([sys.executable, '-m', 'pip', 'install'] + install_list)

setup_environment()

import pygame
import serial
import math

SERIAL_PORT = 'COM7'
BAUD_RATE = 115200
SEND_RATE_HZ = 50
DEADZONE = 10

# IMU姿态数据
imu_roll = 0.0
imu_pitch = 0.0
imu_yaw = 0.0

sensor_data = {
    '姿态': '等待数据...',
    '距离': '等待数据...',
    '灰度': '等待数据...',
    '光电': '等待数据...',
    '转速': '等待数据...',
    '视觉': '等待数据...'
}

control_mode_display = 0  # 0=遥控模式, 1=自动控制模式
angle_enabled_display = 0  # 角度环启用状态
prev_buttons_mask = 0
demo_proc = None

def serial_receive_thread(ser):
    global sensor_data, angle_enabled_display, imu_roll, imu_pitch, imu_yaw
    receive_buffer = ''
    while ser.is_open:
        try:
            if ser.in_waiting > 0:
                receive_buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                lines = receive_buffer.splitlines(keepends=True)

                receive_buffer = ''
                for line in lines:
                    if line.endswith('\n') or line.endswith('\r'):
                        line = line.strip()
                    else:
                        receive_buffer = line
                        continue

                    if not line:
                        continue

                    if line.startswith('VISION '):
                        try:
                            parts = line.split()
                            seq_value = ''
                            tag_type_value = ''
                            yaw_value = ''
                            for part in parts:
                                if part.startswith('seq='):
                                    seq_value = part.split('=', 1)[1]
                                elif part.startswith('tag_type='):
                                    tag_type_value = part.split('=', 1)[1]
                                elif part.startswith('yaw='):
                                    yaw_value = part.split('=', 1)[1]
                            sensor_data['视觉'] = f'SEQ: {seq_value}, TAG_TYPE: {tag_type_value}, YAW: {yaw_value}'
                        except Exception:
                            sensor_data['视觉'] = line
                    elif line.startswith('VISION_STAT'):
                        # 下位机每秒打印：rx / 成功帧 / CRC错 / 格式错 / 环缓冲溢出
                        sensor_data['视觉'] = line.replace('VISION_STAT ', '解析统计: ')
                    elif 'Roll:' in line or 'Pitch:' in line:
                        sensor_data['姿态'] = line
                        # 解析 IMU 数据: "Roll: x.xx, Pitch: y.yy, Yaw: z.zz, ..."
                        try:
                            parts = line.split(',')
                            for part in parts:
                                if 'Roll:' in part: imu_roll = float(part.split(':')[1].strip())
                                elif 'Pitch:' in part: imu_pitch = float(part.split(':')[1].strip())
                                elif 'Yaw:' in part: imu_yaw = float(part.split(':')[1].strip())
                        except: pass
                    elif 'IR Dist' in line: sensor_data['距离'] = line
                    elif 'Grey' in line: sensor_data['灰度'] = line
                    elif 'Photoelectric' in line or 'Photo' in line: sensor_data['光电'] = line
                    elif 'RPM' in line or 'Motor' in line: sensor_data['转速'] = line
                    elif 'Angle Control:' in line:
                        if 'Enabled' in line: angle_enabled_display = 1
                        elif 'Disabled' in line: angle_enabled_display = 0
        except: pass
        time.sleep(0.001)


def start_demo():
    """启动独立的 ModernGL 3D 演示窗口（非阻塞）。"""
    global demo_proc
    if demo_proc and demo_proc.poll() is None:
        return
    demo_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'modern_demo.py')
    try:
        demo_proc = subprocess.Popen([sys.executable, demo_path], cwd=os.path.dirname(demo_path))
    except Exception as e:
        print('Failed to start demo:', e)
        demo_proc = None


def stop_demo():
    """停止演示进程（如果存在）。"""
    global demo_proc
    try:
        if demo_proc:
            demo_proc.terminate()
    except Exception:
        pass
    demo_proc = None


def demo_running():
    return demo_proc is not None and demo_proc.poll() is None

# 3D 立方体顶点（单位化，-1 到 1之间）
CUBE_VERTICES = [
    (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),  # 后面
    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)       # 前面
]

CUBE_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # 后面
    (4, 5), (5, 6), (6, 7), (7, 4),  # 前面
    (0, 4), (1, 5), (2, 6), (3, 7)   # 连接边
]

# ROS 坐标轴定义（原点指向 1.0）
AXIS_VECTORS = [
    ((1, 0, 0), (255, 0, 0)),      # X轴 - 红色
    ((0, 1, 0), (0, 255, 0)),      # Y轴 - 绿色
    ((0, 0, 1), (0, 0, 255))       # Z轴 - 蓝色
]

def rotate_point(p, roll, pitch, yaw):
    """将点绕XYZ轴按欧拉角旋转 (ZYX顺序: Yaw-Pitch-Roll)"""
    x, y, z = p
    
    # 转换为弧度
    r = math.radians(roll)    # X轴
    p_rad = math.radians(pitch)  # Y轴
    ya = math.radians(yaw)    # Z轴
    
    # Yaw 旋转 (Z轴)
    x_new = x * math.cos(ya) - y * math.sin(ya)
    y_new = x * math.sin(ya) + y * math.cos(ya)
    x, y = x_new, y_new
    
    # Pitch 旋转 (Y轴)
    x_new = x * math.cos(p_rad) + z * math.sin(p_rad)
    z_new = -x * math.sin(p_rad) + z * math.cos(p_rad)
    x, z = x_new, z_new
    
    # Roll 旋转 (X轴)
    y_new = y * math.cos(r) - z * math.sin(r)
    z_new = y * math.sin(r) + z * math.cos(r)
    y, z = y_new, z_new
    
    return (x, y, z)

def project_3d(point_3d, distance=2.5):
    """3D点投影到2D屏幕"""
    x, y, z = point_3d
    scale = distance / (distance + z)
    x_2d = int(x * 50 * scale)
    y_2d = int(y * 50 * scale)
    return (x_2d, y_2d, scale)

def draw_attitude_cube(screen, x_center, y_center, roll, pitch, yaw):
    """绘制ROS风格的3D坐标轴"""
    # 原点投影（用于定位）
    origin = rotate_point((0, 0, 0), roll, pitch, yaw)
    origin_proj = project_3d(origin)

    # 绘制一个小立方体（线框）作为原点标记
    cube_scale = 0.25
    cube_points = []
    for vx, vy, vz in CUBE_VERTICES:
        sv = (vx * cube_scale, vy * cube_scale, vz * cube_scale)
        r = rotate_point(sv, roll, pitch, yaw)
        p = project_3d(r)
        cube_points.append((x_center + p[0], y_center + p[1]))
    # 绘制立方体边
    for a, b in CUBE_EDGES:
        pygame.draw.line(screen, (200, 200, 200), cube_points[a], cube_points[b], 2)

    # 绘制三个轴（加长）
    AXIS_LENGTH = 1.6
    for axis_vec, color in AXIS_VECTORS:
        scaled_axis = (axis_vec[0] * AXIS_LENGTH, axis_vec[1] * AXIS_LENGTH, axis_vec[2] * AXIS_LENGTH)
        rotated = rotate_point(scaled_axis, roll, pitch, yaw)
        proj = project_3d(rotated)

        # 从原点到轴端点的线
        pygame.draw.line(screen, color,
                        (x_center + origin_proj[0], y_center + origin_proj[1]),
                        (x_center + proj[0], y_center + proj[1]), 5)

        # 轴端点绘制为小正方形
        end_rect = pygame.Rect(x_center + proj[0] - 6, y_center + proj[1] - 6, 12, 12)
        pygame.draw.rect(screen, color, end_rect)

def main():
    global control_mode_display, angle_enabled_display, prev_buttons_mask, imu_roll, imu_pitch, imu_yaw
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    pygame.init()
    screen_w, screen_h = 1400, 850
    screen = pygame.display.set_mode((screen_w, screen_h))
    pygame.display.set_caption('FightRobot 控制台与监控')
    
    try: font = pygame.font.SysFont(['microsoftyahei', 'simhei', 'simsun'], 32)
    except: font = pygame.font.SysFont(None, 32)
    try: title_font = pygame.font.SysFont(['microsoftyahei', 'simhei', 'simsun'], 48, bold=True)
    except: title_font = pygame.font.SysFont(None, 48, bold=True)
    try: mode_font = pygame.font.SysFont(['microsoftyahei', 'simhei', 'simsun'], 40, bold=True)
    except: mode_font = pygame.font.SysFont(None, 40, bold=True)
    
    pygame.joystick.init()
    joystick = None
    joystick_status = '未连接'
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        joystick_status = f'已连接: {joystick.get_name()}'

    ser = None
    serial_status = '未连接'
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=0.1)
        serial_status = f'已连接: {SERIAL_PORT} @ {BAUD_RATE}'
        rx_thread = threading.Thread(target=serial_receive_thread, args=(ser,), daemon=True)
        rx_thread.start()
    except Exception as e:
        serial_status = f'连接失败: {SERIAL_PORT}'

    clock = pygame.time.Clock()
    running = True
    
    kbd_vx = 0.0
    kbd_vy = 0.0
    kbd_lt = 0.0
    kbd_rt = 0.0
    RAMP_UP = 5.0
    RAMP_DOWN = 10.0
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                # 菜单按钮区域（右上角）
                btn_rect = pygame.Rect(screen_w - 190, 15, 170, 40)
                if btn_rect.collidepoint((mx, my)):
                    if demo_running():
                        stop_demo()
                    else:
                        start_demo()
            elif event.type == pygame.JOYDEVICEADDED:
                joystick = pygame.joystick.Joystick(event.device_index)
                joystick.init()
                joystick_status = f'已连接: {joystick.get_name()}'
            elif event.type == pygame.JOYDEVICEREMOVED:
                if joystick and joystick.get_instance_id() == event.instance_id:
                    joystick.quit()
                    joystick = None
                    joystick_status = '未连接'

        # 键盘平滑控制逻辑
        keys = pygame.key.get_pressed()
        
        # 支持 WSAD 和方向键
        w_pressed = keys[pygame.K_w] or keys[pygame.K_UP]
        s_pressed = keys[pygame.K_s] or keys[pygame.K_DOWN]
        a_pressed = keys[pygame.K_a] or keys[pygame.K_LEFT]
        d_pressed = keys[pygame.K_d] or keys[pygame.K_RIGHT]
        q_pressed = keys[pygame.K_q]
        e_pressed = keys[pygame.K_e]

        if w_pressed: kbd_vx = min(100.0, kbd_vx + RAMP_UP)
        elif s_pressed: kbd_vx = max(-100.0, kbd_vx - RAMP_UP)
        else: kbd_vx = max(0.0, kbd_vx - RAMP_DOWN) if kbd_vx > 0 else min(0.0, kbd_vx + RAMP_DOWN)
            
        if a_pressed: kbd_vy = max(-100.0, kbd_vy - RAMP_UP)
        elif d_pressed: kbd_vy = min(100.0, kbd_vy + RAMP_UP)
        else: kbd_vy = max(0.0, kbd_vy - RAMP_DOWN) if kbd_vy > 0 else min(0.0, kbd_vy + RAMP_DOWN)
            
        if q_pressed: kbd_lt = min(100.0, kbd_lt + RAMP_UP)
        else: kbd_lt = max(0.0, kbd_lt - RAMP_DOWN)
            
        if e_pressed: kbd_rt = min(100.0, kbd_rt + RAMP_UP)
        else: kbd_rt = max(0.0, kbd_rt - RAMP_DOWN)

        vx_fwd_back = 0
        vy_left_right = 0
        lt_val = 0
        rt_val = 0
        buttons_mask = 0

        if joystick:
            try: vx_fwd_back = int(-joystick.get_axis(1) * 100)
            except: vx_fwd_back = 0
            if abs(vx_fwd_back) < DEADZONE: vx_fwd_back = 0
            
            try: vy_left_right = int(joystick.get_axis(2) * 100)
            except: vy_left_right = 0
            if abs(vy_left_right) < DEADZONE: vy_left_right = 0
            
            try: lt_val = int((joystick.get_axis(4) + 1.0) / 2.0 * 100)
            except: lt_val = 0
            try: rt_val = int((joystick.get_axis(5) + 1.0) / 2.0 * 100)
            except: rt_val = 0
            
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i): buttons_mask |= (1 << i)
                
            # 摇杆数据在死区内时，也能接收键盘输入作为补充
            if abs(vx_fwd_back) == 0: vx_fwd_back = int(kbd_vx)
            if abs(vy_left_right) == 0: vy_left_right = int(kbd_vy)
            if lt_val == 0: lt_val = int(kbd_lt)
            if rt_val == 0: rt_val = int(kbd_rt)
            if keys[pygame.K_RETURN]: buttons_mask |= 48
        else:
            # 手柄未连接，完全使用键盘模式
            vx_fwd_back = int(kbd_vx)
            vy_left_right = int(kbd_vy)
            lt_val = int(kbd_lt)
            rt_val = int(kbd_rt)
            if keys[pygame.K_RETURN]: buttons_mask |= 48

        packet = f'<{vx_fwd_back},{vy_left_right},{lt_val},{rt_val},{buttons_mask}>\n'
        
        # 检测按钮 48 的上升沿以切换模式显示
        if (buttons_mask & 48) and not (prev_buttons_mask & 48):
            control_mode_display = 1 - control_mode_display
        prev_buttons_mask = buttons_mask

        if ser and ser.is_open:
            try:
                ser.write(packet.encode('utf-8'))
                ser.flush()
                serial_status = f'已收发: {SERIAL_PORT}'
            except Exception as e:
                serial_status = f'通信中断: {e}'
        
        screen.fill((15, 15, 25))
        y = 25
        
        # ========== 标题行 ==========
        txt = title_font.render('>> FightRobot 控制台与监控 <<', True, (100, 200, 255))
        screen.blit(txt, (screen_w//2 - txt.get_width()//2, y))
        # 右上角菜单按钮：启动/停止独立 3D 演示
        btn_rect = pygame.Rect(screen_w - 190, 15, 170, 40)
        btn_color = (80, 200, 120) if demo_running() else (60, 140, 200)
        pygame.draw.rect(screen, btn_color, btn_rect, border_radius=6)
        try:
            btn_font = pygame.font.SysFont(['microsoftyahei', 'simhei'], 20, bold=True)
        except:
            btn_font = pygame.font.SysFont(None, 20, bold=True)
        btn_text = '停止 3D 视图' if demo_running() else '启动 3D 视图'
        btxt = btn_font.render(btn_text, True, (20, 20, 20))
        screen.blit(btxt, (btn_rect.x + (btn_rect.w - btxt.get_width())//2, btn_rect.y + (btn_rect.h - btxt.get_height())//2))
        y += 65
        
        # ========== 分隔线 ==========
        pygame.draw.line(screen, (100, 150, 200), (40, y), (screen_w - 40, y), 2)
        y += 25
        
        # ========== 模式显示区（并排显示） ==========
        # 遥控/自动模式
        mode_text = '【 遥控模式 】' if control_mode_display == 0 else '【 自动模式 】'
        mode_color = (0, 255, 120) if control_mode_display == 0 else (255, 165, 0)
        mode_display = mode_font.render(mode_text, True, mode_color)
        screen.blit(mode_display, (80, y))
        
        # 角度环状态
        angle_status = '[启用]' if angle_enabled_display else '[禁用]'
        angle_text = f'角度环: {angle_status}'
        angle_color = (0, 255, 150) if angle_enabled_display else (255, 100, 100)
        angle_display = pygame.font.SysFont(['microsoftyahei', 'simhei', 'simsun'], 36, bold=True).render(angle_text, True, angle_color)
        screen.blit(angle_display, (650, y))
        
        # 右上角绘制3D坐标轴（小化） — 背景为浅灰并带网格线
        mini_rect = pygame.Rect(1070, 100, 270, 220)
        pygame.draw.rect(screen, (240, 240, 240), mini_rect)  # 浅灰背景
        # 网格线
        grid_step = 20
        grid_color = (220, 220, 220)
        for gx in range(mini_rect.left, mini_rect.right + 1, grid_step):
            pygame.draw.line(screen, grid_color, (gx, mini_rect.top), (gx, mini_rect.bottom), 1)
        for gy in range(mini_rect.top, mini_rect.bottom + 1, grid_step):
            pygame.draw.line(screen, grid_color, (mini_rect.left, gy), (mini_rect.right, gy), 1)
        # 外边框
        pygame.draw.rect(screen, (100, 100, 100), mini_rect, 2)
        draw_attitude_cube(screen, mini_rect.centerx, mini_rect.centery, imu_roll, imu_pitch, imu_yaw)
        
        y += 70
        
        # ========== 分隔线 ==========
        pygame.draw.line(screen, (100, 150, 200), (40, y), (screen_w - 40, y), 1)
        y += 25
        
        # ========== 设备状态行（大字体） ==========
        c_joy = (0, 255, 100) if joystick else (255, 80, 80)
        joy_text = f'手柄: {joystick_status}'
        screen.blit(font.render(joy_text, True, c_joy), (80, y))
        y += 55
        
        c_ser = (0, 255, 100) if ser and ser.is_open else (255, 80, 80)
        ser_text = f'串口: {serial_status}'
        screen.blit(font.render(ser_text, True, c_ser), (80, y))
        y += 60
        
        # ========== 分隔线 ==========
        pygame.draw.line(screen, (100, 150, 200), (40, y), (screen_w - 40, y), 1)
        y += 20
        
        # ========== 下发指令 ==========
        cmd_text = f'下发指令: {packet.strip()}'
        screen.blit(font.render(cmd_text, True, (255, 200, 80)), (80, y))
        y += 60
        
        # ========== 分隔线 ==========
        pygame.draw.line(screen, (100, 150, 200), (40, y), (screen_w - 40, y), 2)
        y += 30
        
        # ========== 传感器数据监控 ==========
        header_txt = '=== 实时传感器数据监控 ==='
        screen.blit(pygame.font.SysFont(['microsoftyahei', 'simhei', 'simsun'], 36, bold=True).render(header_txt, True, (150, 200, 255)), (60, y))
        y += 65
        
        # 各传感器数据行
        data_colors = {
            '姿态': (100, 200, 255),      # 蓝色
            '距离': (150, 255, 100),      # 绿色
            '灰度': (255, 200, 100),      # 橙色
            '光电': (255, 150, 150),      # 红色
            '转速': (200, 150, 255),      # 紫色
            '视觉': (100, 255, 220)       # 青绿色
        }
        
        for label, value in sensor_data.items():
            color = data_colors.get(label, (220, 220, 220))
            label_text = f'>> {label}: '
            screen.blit(font.render(label_text, True, color), (100, y))
            screen.blit(font.render(value, True, (200, 200, 200)), (380, y))
            y += 60
            
        pygame.display.flip()
        clock.tick(SEND_RATE_HZ)

    if ser and ser.is_open: ser.close()
    # 停止 demo 进程（如果由本程序启动）
    try:
        stop_demo()
    except: pass
    pygame.quit()
    sys.exit(0)

if __name__ == '__main__':
    main()