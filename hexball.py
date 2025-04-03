import pygame
import sys
import math
import numpy as np
import random

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 600
FPS = 60 # I dont have a better display
BACKGROUND = (10, 10, 30)
HEXAGON_COLOR = (100, 200, 255)
BALL_COLOR = (255, 100, 100)
BALL_RADIUS = 15
BASE_HEXAGON_SIZE = 100  # Base size for the innermost hexagon
ROTATION_SPEED = 0.01  # radians per frame
# The more the harder, the less the boring -er
GRAVITY = 0.2 # Better not to change
FRICTION = 0.9 # Better not to change
RESTITUTION = 0.4  # Bouncy-ness, idk
JUMP_STRENGTH = -10  # Negative because y increases downward
SHAKE_DISTANCE = 15  # Pixels to shake
SHAKE_DURATION = 375  # In ms, change it to what you want
SPRING_CONSTANT = 0.2  # Controls the spring motion
DAMPING = 0.92  # Controls how quickly oscillation stops
GAP_SIZE = BALL_RADIUS * 3  # Size of gaps in the hexagons
HEXAGON_SPACING = BALL_RADIUS * 3  # Space between hexagons
WIN_COLOR = (50, 200, 50)  # Green color for win screen
WIN_TEXT_COLOR = (255, 255, 255)  # White text for win message

# Create the screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Escape the Hexagons")
clock = pygame.time.Clock()

class Hexagon:
    def __init__(self, center, size, gap_position=None):
        self.center = np.array(center, dtype=float)
        self.original_center = np.array(center, dtype=float)
        self.size = size
        self.angle = 0
        self.gap_position = gap_position  # Which side has a gap (0-5)
        self.vertices = self.calculate_vertices()
        self.edges = self.calculate_edges()
        self.is_shaking = False
        self.velocity = np.array([0.0, 0.0])
        self.shake_start_time = 0
        self.target_pos = None
        
    def calculate_vertices(self):
        vertices = []
        for i in range(6):
            angle = self.angle + i * (2 * math.pi / 6)
            x = self.center[0] + self.size * math.cos(angle)
            y = self.center[1] + self.size * math.sin(angle)
            vertices.append((x, y))
        return vertices
    
    def calculate_edges(self):
        # Create edges, but with a gap if specified
        edges = []
        for i in range(6):
            if self.gap_position is not None and i == self.gap_position:
                # Calculate the midpoint of this edge
                p1 = np.array(self.vertices[i])
                p2 = np.array(self.vertices[(i + 1) % 6])
                midpoint = (p1 + p2) / 2
                
                # Calculate the normalized direction of the edge
                edge_vec = p2 - p1
                edge_length = np.linalg.norm(edge_vec)
                if edge_length > 0:
                    edge_normalized = edge_vec / edge_length
                else:
                    edge_normalized = np.array([0, 0])
                
                # Calculate points for partial edges, creating a gap
                gap_half_size = GAP_SIZE / 2
                gap_start = midpoint - edge_normalized * gap_half_size
                gap_end = midpoint + edge_normalized * gap_half_size
                
                # Add two partial edges instead of one full edge
                if np.linalg.norm(p1 - gap_start) > 1:  # Make sure segment has length
                    edges.append((p1, gap_start))
                if np.linalg.norm(gap_end - p2) > 1:  # Make sure segment has length
                    edges.append((gap_end, p2))
            else:
                # Regular edge with no gap
                edges.append((self.vertices[i], self.vertices[(i + 1) % 6]))
        return edges
    
    def shake(self, direction):
        if not self.is_shaking:
            self.is_shaking = True
            shake_direction = np.array(direction, dtype=float)
            # Normalize and scale
            if np.linalg.norm(shake_direction) > 0:
                shake_direction = shake_direction / np.linalg.norm(shake_direction) * SHAKE_DISTANCE
            
            # Set target position
            self.target_pos = self.original_center + shake_direction
            # Initial push velocity
            self.velocity = (self.target_pos - self.center) * 0.3
            self.shake_start_time = pygame.time.get_ticks()
    
    def update(self, angle=None):
        # If angle is provided, use it (for synchronized rotation)
        if angle is not None:
            self.angle = angle
        else:
            self.angle += ROTATION_SPEED
        
        # Handle shaking with spring motion
        if self.is_shaking:
            current_time = pygame.time.get_ticks()
            elapsed = current_time - self.shake_start_time
            
            if elapsed < SHAKE_DURATION:
                # Spring physics for first part of shake
                displacement = self.original_center - self.center
                spring_force = displacement * SPRING_CONSTANT
                
                # Apply spring force to velocity
                self.velocity += spring_force
                self.velocity *= DAMPING  # Apply damping
                
                # Update position
                self.center += self.velocity
            else:
                # Return to center
                distance_to_original = np.linalg.norm(self.original_center - self.center)
                if distance_to_original < 0.5:
                    # Close enough, snap to original
                    self.is_shaking = False
                    self.center = np.copy(self.original_center)
                    self.velocity = np.array([0.0, 0.0])
                else:
                    # Spring physics for return
                    displacement = self.original_center - self.center
                    spring_force = displacement * SPRING_CONSTANT * 1.5
                    
                    self.velocity += spring_force
                    self.velocity *= DAMPING
                    self.center += self.velocity
        
        # Update vertices and edges after position/angle change
        self.vertices = self.calculate_vertices()
        self.edges = self.calculate_edges()
    
    def draw(self, surface):
        # Draw each edge segment instead of a full polygon
        for edge in self.edges:
            pygame.draw.line(surface, HEXAGON_COLOR, edge[0], edge[1], 3)

class Ball:
    def __init__(self, pos, radius):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array([0.0, 0.0])
        self.radius = radius
        self.on_ground = False
        
    def jump(self):
        # Only allow jumping if the ball is "on ground" (in contact with something)
        if self.on_ground:
            self.vel[1] = JUMP_STRENGTH
            self.on_ground = False
        
    def update(self, hexagons):
        # Apply gravity
        self.vel[1] += GRAVITY
        
        # Update position
        self.pos += self.vel
        
        # Reset on_ground flag
        self.on_ground = False
        
        # Check collision with all hexagon edges
        for hexagon in hexagons:
            for edge in hexagon.edges:
                p1 = np.array(edge[0])
                p2 = np.array(edge[1])
                
                # Vector from p1 to p2
                edge_vec = p2 - p1
                edge_length = np.linalg.norm(edge_vec)
                
                # Skip if edge length is too small
                if edge_length < 1:
                    continue
                    
                edge_normalized = edge_vec / edge_length
                
                # Vector from p1 to ball center
                to_ball = self.pos - p1
                
                # Project to_ball onto edge to find closest point on line
                projection = np.dot(to_ball, edge_normalized)
                projection = max(0, min(edge_length, projection))
                
                # Find closest point on line segment
                closest_point = p1 + projection * edge_normalized
                
                # Check distance to closest point
                distance_vector = self.pos - closest_point
                distance = np.linalg.norm(distance_vector)
                
                if distance < self.radius:
                    # Collision detected, bounce off the edge
                    self.pos = closest_point + (distance_vector / distance) * self.radius
                    
                    # Normal to the edge (perpendicular)
                    normal = np.array([-edge_normalized[1], edge_normalized[0]])
                    
                    # Reflect velocity along the normal
                    dot_product = np.dot(self.vel, normal)
                    self.vel = self.vel - (1 + RESTITUTION) * dot_product * normal
                    
                    # Apply friction along the edge
                    tangent = edge_normalized
                    dot_product_tangent = np.dot(self.vel, tangent)
                    self.vel = FRICTION * (self.vel - dot_product_tangent * tangent) + dot_product_tangent * tangent
                    
                    # If collision is somewhat vertical, consider the ball "on ground"
                    if abs(normal[1]) > 0.3:  # Check if normal has significant vertical component
                        self.on_ground = True
                
        # Constrain ball inside screen boundaries (backup)
        if self.pos[0] < self.radius:
            self.pos[0] = self.radius
            self.vel[0] = -self.vel[0] * RESTITUTION
        elif self.pos[0] > WIDTH - self.radius:
            self.pos[0] = WIDTH - self.radius
            self.vel[0] = -self.vel[0] * RESTITUTION
            
        if self.pos[1] < self.radius:
            self.pos[1] = self.radius
            self.vel[1] = -self.vel[1] * RESTITUTION
        elif self.pos[1] > HEIGHT - self.radius:
            self.pos[1] = HEIGHT - self.radius
            self.vel[1] = -self.vel[1] * RESTITUTION
            self.on_ground = True
    
    def draw(self, surface):
        pygame.draw.circle(surface, BALL_COLOR, (int(self.pos[0]), int(self.pos[1])), self.radius)
    
    def is_outside_hexagons(self, hexagons, center):
        # Get the distance from the center
        distance_from_center = np.linalg.norm(self.pos - center)
        
        # Get the radius of the outermost hexagon
        max_radius = max([h.size for h in hexagons])
        
        # The ball is outside all hexagons if its distance from center
        # is greater than the outermost hexagon radius plus ball radius
        return distance_from_center > max_radius + self.radius

def create_hexagons(center, num_hexagons):
    hexagons = []
    
    for i in range(num_hexagons):
        # Calculate size for this hexagon
        size = BASE_HEXAGON_SIZE + (i * HEXAGON_SPACING)
        
        # Create a random gap position that's different for each hexagon
        gap_position = random.randint(0, 5)  # Random position (0-5)
        
        # Create hexagon with random gap
        hexagon = Hexagon(center, size, gap_position)
        hexagons.append(hexagon)
    
    return hexagons

def reset_game(center):
    # Generate a new random number of hexagons
    num_hexagons = random.randint(3, 7)
    # Create new hexagons with new random gap positions
    hexagons = create_hexagons(center, num_hexagons)
    # Reset ball to center
    ball = Ball(center, BALL_RADIUS)
    return hexagons, ball, False  # False is for game_won state

# Create objects for initial game
center = (WIDTH//2, HEIGHT//2)
hexagons, ball, game_won = reset_game(center)

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                ball.jump()
            # Restart game if R is pressed and game is won
            if event.key == pygame.K_r and game_won:
                hexagons, ball, game_won = reset_game(center)
    
    # Skip updates if game is won
    if not game_won:
        # Handle arrow key inputs for shaking
        keys = pygame.key.get_pressed()
        shake_direction = None
        if keys[pygame.K_LEFT]:
            shake_direction = (-1, 0)
        elif keys[pygame.K_RIGHT]:
            shake_direction = (1, 0)
        elif keys[pygame.K_UP]:
            shake_direction = (0, -1)
        elif keys[pygame.K_DOWN]:
            shake_direction = (0, 1)
            
        # Apply shake to all hexagons
        if shake_direction:
            for hexagon in hexagons:
                hexagon.shake(shake_direction)
        
        # Update all hexagons with the same rotation angle
        base_angle = hexagons[0].angle + ROTATION_SPEED
        for hexagon in hexagons:
            hexagon.update(base_angle)
            
        # Update ball
        ball.update(hexagons)
        
        # Check win condition
        if ball.is_outside_hexagons(hexagons, np.array(center)):
            game_won = True
    
    # Draw
    if game_won:
        # Draw win screen
        screen.fill(WIN_COLOR)
        font = pygame.font.Font(None, 72)
        text = font.render("YOU WON!", True, WIN_TEXT_COLOR)
        text_rect = text.get_rect(center=(WIDTH//2, HEIGHT//2))
        screen.blit(text, text_rect)
        
        # Add restart instruction
        font_small = pygame.font.Font(None, 36)
        restart_text = font_small.render("Press R to restart", True, WIN_TEXT_COLOR)
        restart_rect = restart_text.get_rect(center=(WIDTH//2, HEIGHT//2 + 50))
        screen.blit(restart_text, restart_rect)
    else:
        # Regular game screen
        screen.fill(BACKGROUND)
        for hexagon in hexagons:
            hexagon.draw(screen)
        ball.draw(screen)
    
    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()
