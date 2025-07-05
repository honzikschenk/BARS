# ========== WIP ==========
# This is a sample file used for experimenting
# with various neural network architectures and
# reinforcement learning environments.
# =========================


import gymnasium as gym

# Initialise the environment
env = gym.make("LunarLander-v3", render_mode="human")

# Reset the environment to generate the first observation
observation, info = env.reset(seed=42)
for _ in range(1000):
    # this is where you would insert your policy
    action = env.action_space.sample()

    # step (transition) through the environment with the action
    # receiving the next observation, reward and if the episode has terminated or truncated
    observation, reward, terminated, truncated, info = env.step(action)

    # If the episode has ended then we can reset to start a new episode
    if terminated or truncated:
        observation, info = env.reset()

env.close()

# import pygame
# import random

# pygame.init()

# TIME_STEP = 0.01  # Time step for the game loop

# # Element colors
# SKY_COLOR = (135, 206, 235)  # Light blue
# PLAYER_COLOR = (0, 128, 0)  # Dark green
# OBSTACLE_COLOR = (255, 0, 0)  # Red

# # Starting positions of elements
# player_pos = (100, 150)

# # Set up the game window
# screen = pygame.display.set_mode((400, 300))
# pygame.display.set_caption("Floppy Cube")

# # Obstacle properties
# OBSTACLE_WIDTH = 20

# GAP_RANGE = (50, 150)
# TOP_RANGE = (50, 150)

# MAX_OBSTACLES = 10

# obstacles = [] # (top, gap, x)

# TIME_BETWEEN_OBSTACLES_RANGE = (0.8, 1.5)
# time_between_obstacles = random.uniform(*TIME_BETWEEN_OBSTACLES_RANGE)
# time_since_last_obstacle = 0.0

# # Player properties
# PLAYER_SIZE = 10
# player_velocity = 0.0
# player_accel = 0.05

# # Game loop
# running = True
# while running:
#     # Logic
#     if player_velocity < 3:
#         player_velocity += player_accel
#     player_pos = (player_pos[0], player_pos[1] + player_velocity)

#     if player_pos[1] > 300 - PLAYER_SIZE:
#         player_pos = (player_pos[0], 300 - PLAYER_SIZE)
#         player_velocity = 0

#     if len(obstacles) < MAX_OBSTACLES and time_since_last_obstacle >= time_between_obstacles:
#         top = random.randint(*TOP_RANGE)
#         gap = random.randint(*GAP_RANGE)
#         x = 400
#         obstacles.append([top, gap, x])
#         time_between_obstacles = random.uniform(*TIME_BETWEEN_OBSTACLES_RANGE)
#         time_since_last_obstacle = 0.0

#     for obstacle in obstacles:
#         obstacle[2] -= 2
#         if obstacle[2] < -OBSTACLE_WIDTH:
#             obstacles.remove(obstacle)

#     # Check for collisions
#     for obstacle in obstacles:
#         top, gap, x = obstacle
#         if (player_pos[0] + PLAYER_SIZE > x and player_pos[0] < x + OBSTACLE_WIDTH) and (player_pos[1] < top or player_pos[1] + PLAYER_SIZE > top + gap):
#             print("Collision detected!")
#             player_pos = (100, 150)
#             player_velocity = 0.0
#             obstacles.clear()
#             time_since_last_obstacle = 0.0

#     # Drawing
#     screen.fill(SKY_COLOR)

#     pygame.draw.rect(screen, PLAYER_COLOR, (player_pos[0], player_pos[1], PLAYER_SIZE, PLAYER_SIZE))

#     for obstacle in obstacles:
#         top, gap, x = obstacle
#         pygame.draw.rect(screen, OBSTACLE_COLOR, (x, 0, OBSTACLE_WIDTH, top))
#         pygame.draw.rect(screen, OBSTACLE_COLOR, (x, top + gap, OBSTACLE_WIDTH, 300 - (top + gap)))

#     for event in pygame.event.get():
#         if event.type == pygame.KEYDOWN:
#             if event.key == pygame.K_UP:
#                 player_velocity = -2
#         if event.type == pygame.QUIT:
#             running = False

#     # Update
#     pygame.display.flip()
#     pygame.time.delay(int(TIME_STEP * 1000))
#     time_since_last_obstacle += TIME_STEP

# # Quit Pygame
# pygame.quit()
