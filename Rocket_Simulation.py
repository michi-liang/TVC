import pygame
import numpy as np
import math
import random

# Initialize pygame
pygame.init()

# Screen setup
WIDTH, HEIGHT = 600, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Rocket Simulation with PID")

# Colors
BLUE = (135, 206, 235)
GREEN = (34, 139, 34)
WHITE = (255, 255, 255)
RED = (200, 50, 50)
YELLOW = (255, 200, 0)

# Physics
gravity = np.array([0, 0.2])
thrust_strength = 0.5
torque_strength = 0.1

# PID controller gains
Kp = 0.1
Ki = 0.00001
Kd = 0.65

integral_error = 0
previous_error = 0

# Rocket state
rocket_pos = np.array([WIDTH/2, HEIGHT - 50.0], dtype=float)
rocket_vel = np.array([0.0, 0.0])
angle = -math.pi/2
angular_vel = 0.0

# Stars
stars = [(random.randint(0, WIDTH*5), random.randint(-HEIGHT*10, HEIGHT*5)) for _ in range(1000)]

# Font for position
font = pygame.font.SysFont(None, 30)

clock = pygame.time.Clock()
running = True

# Upright timer
upright = False
upright_start_time = 0.0
upright_duration = 0.0
angle_threshold = 0.01 

# Wind
wind_force_strength = 0.05 
wind_torque_strength = 0.002
wind_interval = 60

wind_timer = 0
current_wind_torque = 0.0
current_wind_force = np.array([0.0, 0.0], dtype=float)
wind_torque_frames_left = 0
wind_force_frames_left = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()

    # Player input
    thrust = 0
    player_torque = 0
    if keys[pygame.K_UP]:
        thrust = thrust_strength
    if keys[pygame.K_LEFT]:
        player_torque -= torque_strength
    if keys[pygame.K_RIGHT]:
        player_torque += torque_strength

    # Wind
    wind_timer += 1
    if wind_timer > wind_interval:
        wind_timer = 0
        if wind_torque_frames_left <= 0:
            wind_torque_frames_left = random.randint(30, 120) 
            current_wind_torque = random.uniform(-wind_torque_strength, wind_torque_strength)
        if wind_force_frames_left <= 0:
            wind_force_frames_left = random.randint(30, 120)
            current_wind_force = np.array([random.uniform(-wind_force_strength, wind_force_strength), 0.0], dtype=float)

    if wind_torque_frames_left > 0:
        wind_torque_frames_left -= 1
    else:
        current_wind_torque = 0.0

    if wind_force_frames_left > 0:
        wind_force_frames_left -= 1
    else:
        current_wind_force[:] = 0.0

    # PID
    desired_angle = 0
    error = desired_angle - angle
    integral_error += error
    derivative_error = error - previous_error
    previous_error = error

    pid_torque = (Kp * error + Ki * integral_error + Kd * derivative_error)

    # Update angular velocity & angle
    angular_vel += pid_torque + player_torque + current_wind_torque
    angle += angular_vel

    # Thrust
    thrust_force = np.array([math.sin(angle), -math.cos(angle)]) * thrust

    # Apply acceleration
    acc = thrust_force + gravity + current_wind_force
    rocket_vel += acc
    rocket_pos += rocket_vel

    # Ground collision
    if rocket_pos[1] > HEIGHT - 50:
        rocket_pos[1] = HEIGHT - 50
        rocket_vel[1] = 0

    camera_x = rocket_pos[0] - WIDTH / 2
    camera_y = rocket_pos[1] - HEIGHT / 2
    screen.fill(BLUE)

    # Stars
    for sx, sy in stars:
        pygame.draw.circle(screen, WHITE, (int(sx - camera_x), int(sy - camera_y)), 4)

    # Ground
    ground_y = HEIGHT - camera_y
    pygame.draw.rect(screen, GREEN, (0 - camera_x, ground_y, WIDTH*5, HEIGHT))

    # Rocket dimensions
    body_width, body_height = 30, 100
    nose_height = 30
    fin_height, fin_width = 30, 20

    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    def rotate_point(px, py):
        return (px * cos_a - py * sin_a, px * sin_a + py * cos_a)

    def transform_point(local_x, local_y):
        rx, ry = rotate_point(local_x, local_y)
        return (rocket_pos[0] + rx - camera_x, rocket_pos[1] + ry - camera_y)

    # Body
    body_points = [
        transform_point(-body_width/2, body_height/2),
        transform_point(body_width/2, body_height/2),
        transform_point(body_width/2, -body_height/2),
        transform_point(-body_width/2, -body_height/2),
    ]
    pygame.draw.polygon(screen, WHITE, body_points)

    # Nose
    nose_points = [
        transform_point(0, -body_height/2 - nose_height),
        transform_point(-body_width/2, -body_height/2),
        transform_point(body_width/2, -body_height/2),
    ]
    pygame.draw.polygon(screen, WHITE, nose_points)

    # Fins
    left_fin = [
        transform_point(-body_width/2, body_height/2),
        transform_point(-body_width/2 - fin_width, body_height/2),
        transform_point(-body_width/2, body_height/2 - fin_height),
    ]
    right_fin = [
        transform_point(body_width/2, body_height/2),
        transform_point(body_width/2 + fin_width, body_height/2),
        transform_point(body_width/2, body_height/2 - fin_height),
    ]
    pygame.draw.polygon(screen, RED, left_fin)
    pygame.draw.polygon(screen, RED, right_fin)

    # Flame
    if thrust > 0:
        flame = [
            transform_point(-10, body_height/2),
            transform_point(10, body_height/2),
            transform_point(0, body_height/2 + 40 + random.randint(-5, 5)),
        ]
        pygame.draw.polygon(screen, YELLOW, flame)

    # Display position
    pos_text = font.render(f"X: {int(rocket_pos[0])-300}  Y: {750-int(rocket_pos[1])}", True, WHITE)
    screen.blit(pos_text, (10, 10))

    #show upright timer
    current_time = pygame.time.get_ticks() / 1000.0
    if abs(angle) < angle_threshold:
        if not upright:
            upright = True
            upright_start_time = current_time
        upright_duration = current_time - upright_start_time
    else:
        upright = False
        upright_duration = 0.0
    timer_text = font.render(f"Upright: {upright_duration:.2f}s", True, WHITE)
    screen.blit(timer_text, (10, 40))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()

