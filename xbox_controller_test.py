import pygame
import sys

# Initialize Pygame
pygame.init()
pygame.joystick.init()

# Set up the display
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Xbox Controller Tester")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Font
font = pygame.font.Font(None, 24)
title_font = pygame.font.Font(None, 36)

# Check for joysticks
joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks detected: {joystick_count}")

if joystick_count == 0:
    print("No joystick/controller detected!")
    sys.exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Controller name: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")
print(f"Number of hats: {joystick.get_numhats()}")

# Main loop
clock = pygame.time.Clock()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    # Clear screen
    screen.fill(BLACK)

    # Title
    title = title_font.render("Xbox Controller Test", True, WHITE)
    screen.blit(title, (250, 20))

    # Controller info
    name_text = font.render(f"Controller: {joystick.get_name()}", True, GREEN)
    screen.blit(name_text, (20, 70))

    # Display axis values (joysticks and triggers)
    y_offset = 110
    num_axes = joystick.get_numaxes()
    
    axes_title = font.render("=== AXES (Joysticks & Triggers) ===", True, BLUE)
    screen.blit(axes_title, (20, y_offset))
    y_offset += 30

    axis_names = [
        "Left Stick X",      # Axis 0
        "Left Stick Y",      # Axis 1
        "Right Stick X",     # Axis 2
        "Right Stick Y",     # Axis 3
        "Left Trigger",      # Axis 4 (usually)
        "Right Trigger"      # Axis 5 (usually)
    ]

    for i in range(num_axes):
        axis_value = joystick.get_axis(i)
        axis_name = axis_names[i] if i < len(axis_names) else f"Axis {i}"
        
        # Color code based on value
        if abs(axis_value) > 0.1:
            color = GREEN
        else:
            color = WHITE
        
        axis_text = font.render(f"{axis_name} (Axis {i}): {axis_value:6.3f}", True, color)
        screen.blit(axis_text, (20, y_offset))
        
        # Draw a visual bar
        bar_x = 400
        bar_y = y_offset + 5
        bar_width = 200
        bar_height = 15
        
        # Background bar
        pygame.draw.rect(screen, (50, 50, 50), (bar_x, bar_y, bar_width, bar_height))
        
        # Value bar
        value_width = int((axis_value + 1) / 2 * bar_width)
        pygame.draw.rect(screen, GREEN, (bar_x, bar_y, value_width, bar_height))
        
        # Center line
        center_x = bar_x + bar_width // 2
        pygame.draw.line(screen, WHITE, (center_x, bar_y), (center_x, bar_y + bar_height), 2)
        
        y_offset += 35

    # Display button states
    y_offset += 20
    buttons_title = font.render("=== BUTTONS ===", True, BLUE)
    screen.blit(buttons_title, (20, y_offset))
    y_offset += 30

    num_buttons = joystick.get_numbuttons()
    button_names = [
        "A", "B", "X", "Y",           # 0-3
        "LB", "RB",                    # 4-5
        "Back", "Start",               # 6-7
        "L-Stick", "R-Stick"          # 8-9
    ]

    for i in range(min(num_buttons, 10)):  # Show first 10 buttons
        button_pressed = joystick.get_button(i)
        button_name = button_names[i] if i < len(button_names) else f"Button {i}"
        
        color = GREEN if button_pressed else WHITE
        status = "PRESSED" if button_pressed else "Released"
        
        button_text = font.render(f"Button {i} ({button_name}): {status}", True, color)
        screen.blit(button_text, (20, y_offset))
        y_offset += 25

    # Display D-pad (hat) state
    if joystick.get_numhats() > 0:
        y_offset += 10
        hat_title = font.render("=== D-PAD ===", True, BLUE)
        screen.blit(hat_title, (20, y_offset))
        y_offset += 30

        hat = joystick.get_hat(0)
        hat_text = font.render(f"D-Pad: X={hat[0]:2d}, Y={hat[1]:2d}", True, WHITE)
        screen.blit(hat_text, (20, y_offset))

    # Instructions
    instructions = font.render("Press ESC or close window to exit", True, WHITE)
    screen.blit(instructions, (20, 560))

    # Update display
    pygame.display.flip()
    clock.tick(60)  # 60 FPS

# Clean up
pygame.quit()
print("Controller test ended.")
