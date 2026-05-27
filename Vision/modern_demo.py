"""ModernGL 演示：独立窗口显示 3D 立方体、坐标轴与网格

依赖: moderngl, pyglet, pyrr
运行: pip install -r requirements_3d.txt
      python modern_demo.py
"""
import math
import time
import pyglet
import moderngl
from pyrr import Matrix44, Vector3


VERT_SHADER = '''#version 330
in vec3 in_pos;
in vec3 in_color;
uniform mat4 mvp;
out vec3 v_color;
void main() {
    gl_Position = mvp * vec4(in_pos, 1.0);
    v_color = in_color;
}
'''

FRAG_SHADER = '''#version 330
in vec3 v_color;
out vec4 f_color;
void main() {
    f_color = vec4(v_color, 1.0);
}
'''


class DemoWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super().__init__(resizable=True, vsync=True, caption='Attitude ModernGL Demo', **kwargs)
        self.ctx = moderngl.create_context()
        self.program = self.ctx.program(vertex_shader=VERT_SHADER, fragment_shader=FRAG_SHADER)

        # 设置相机/模型参数
        self.camera_pos = Vector3([3.0, 3.0, 3.0])
        self.camera_target = Vector3([0.0, 0.0, 0.0])
        self.up = Vector3([0.0, 1.0, 0.0])

        # 初始角度
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # 几何体数据：单位立方体
        cube_positions = [
            # 前面
            -0.5, -0.5,  0.5,
             0.5, -0.5,  0.5,
             0.5,  0.5,  0.5,
            -0.5,  0.5,  0.5,
            # 后面
            -0.5, -0.5, -0.5,
             0.5, -0.5, -0.5,
             0.5,  0.5, -0.5,
            -0.5,  0.5, -0.5,
        ]
        cube_colors = [
            0.8, 0.8, 0.8,
        ] * 8

        cube_indices = [
            0,1,2, 2,3,0,
            4,5,6, 6,7,4,
            0,1,5, 5,4,0,
            2,3,7, 7,6,2,
            1,2,6, 6,5,1,
            3,0,4, 4,7,3,
        ]

        # 顶点缓冲
        import struct
        data = bytearray()
        for i in range(8):
            data += struct.pack('3f', cube_positions[i*3+0], cube_positions[i*3+1], cube_positions[i*3+2])
            data += struct.pack('3f', cube_colors[i*3+0], cube_colors[i*3+1], cube_colors[i*3+2])

        self.vbo = self.ctx.buffer(bytes(data))
        self.ibo = self.ctx.buffer(struct.pack(f'{len(cube_indices)}I', *cube_indices))
        self.vao = self.ctx.vertex_array(self.program, [(self.vbo, '3f 3f', 'in_pos', 'in_color')], self.ibo)

        # 坐标轴（线）
        axes = [
            0.0,0.0,0.0,  1.6,0.0,0.0,  # X (red)
            0.0,0.0,0.0,  0.0,1.6,0.0,  # Y (green)
            0.0,0.0,0.0,  0.0,0.0,1.6,  # Z (blue)
        ]
        axes_colors = [
            1.0,0.0,0.0, 1.0,0.0,0.0,
            0.0,1.0,0.0, 0.0,1.0,0.0,
            0.0,0.0,1.0, 0.0,0.0,1.0,
        ]
        axes_data = bytearray()
        for i in range(6):
            axes_data += struct.pack('3f', axes[i*3+0], axes[i*3+1], axes[i*3+2])
            axes_data += struct.pack('3f', axes_colors[i*3+0], axes_colors[i*3+1], axes_colors[i*3+2])

        self.axes_vbo = self.ctx.buffer(bytes(axes_data))
        self.axes_vao = self.ctx.vertex_array(self.program, [(self.axes_vbo, '3f 3f', 'in_pos', 'in_color')])

        # 网格（地面 xz 平面）
        grid_lines = []
        grid_colors = []
        size = 2.0
        step = 0.2
        c_gray = (0.85, 0.85, 0.85)
        for i in range(int(-size/step), int(size/step)+1):
            x = i * step
            # line parallel Z
            grid_lines += [x, 0.0, -size, x, 0.0, size]
            # line parallel X
            grid_lines += [-size, 0.0, x, size, 0.0, x]
            grid_colors += list(c_gray) * 4

        grid_data = bytearray()
        n_grid_verts = len(grid_lines)//3
        for i in range(n_grid_verts):
            grid_data += struct.pack('3f', grid_lines[i*3+0], grid_lines[i*3+1], grid_lines[i*3+2])
            grid_data += struct.pack('3f', c_gray[0], c_gray[1], c_gray[2])

        self.grid_vbo = self.ctx.buffer(bytes(grid_data))
        self.grid_vao = self.ctx.vertex_array(self.program, [(self.grid_vbo, '3f 3f', 'in_pos', 'in_color')])

        pyglet.clock.schedule_interval(self.update, 1/60.0)

    def on_resize(self, width, height):
        self.ctx.viewport = (0, 0, width, height)

    def update(self, dt):
        # 简易自动旋转（或可以通过按键改）
        # self.yaw += 20.0 * dt
        pass

    def on_draw(self):
        self.clear()
        # 背景浅灰
        self.ctx.clear(0.94, 0.94, 0.94)

        width, height = self.get_size()
        proj = Matrix44.perspective_projection(45.0, width / float(height), 0.1, 100.0)
        view = Matrix44.look_at(self.camera_pos, self.camera_target, self.up)

        # 模型由 roll/pitch/yaw 控制
        model = Matrix44.identity()
        model = model * Matrix44.from_x_rotation(math.radians(self.roll))
        model = model * Matrix44.from_y_rotation(math.radians(self.pitch))
        model = model * Matrix44.from_z_rotation(math.radians(self.yaw))

        mvp = proj * view * model
        self.program['mvp'].write(mvp.astype('f4').tobytes())

        # 绘制网格（在模型变换前用世界坐标）
        # 临时用单位模型
        # 直接使用 mvp
        self.program['mvp'].write(mvp.astype('f4').tobytes())
        self.grid_vao.render(mode=moderngl.LINES)

        # 绘制坐标轴（更粗）
        self.axes_vao.render(mode=moderngl.LINES)

        # 绘制立方体（半透明风格可改）
        self.vao.render()

    def on_key_press(self, symbol, modifiers):
        # 使用方向键微调角度，PgUp/PgDn 控制 roll
        if symbol == pyglet.window.key.LEFT:
            self.yaw -= 5.0
        elif symbol == pyglet.window.key.RIGHT:
            self.yaw += 5.0
        elif symbol == pyglet.window.key.UP:
            self.pitch += 5.0
        elif symbol == pyglet.window.key.DOWN:
            self.pitch -= 5.0
        elif symbol == pyglet.window.key.PAGEUP:
            self.roll += 5.0
        elif symbol == pyglet.window.key.PAGEDOWN:
            self.roll -= 5.0


if __name__ == '__main__':
    win = DemoWindow(width=640, height=480)
    pyglet.app.run()
