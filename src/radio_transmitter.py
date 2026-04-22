import pygame
import socket

ESP_IP = "10.227.33.197"
PORT = 4210
DEADZONE = 0.4
SEQ_AXIS = 2
SEQ_THRESHOLD = 0.5 # Trigger point (0.0 to 1.0)

# Initialize Pygame and Joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

controller = pygame.joystick.Joystick(0)
controller.init()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Track states
states = {
    "w": False, "s": False, "a": False, "d": False,
    "seq_active": False, "seq_latched": False
}

def send_udp(message):
    try:
        sock.sendto(bytes(message, "utf-8"), (ESP_IP, PORT))
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Error: {e}")

def handle_movement(axis_val, pos_key, neg_key):
    """Updates WASD states based on axis value."""
    pos_active = axis_val > DEADZONE
    if pos_active != states[pos_key]:
        states[pos_key] = pos_active
        send_udp(f"{pos_key.upper()}_{'ON' if pos_active else 'OFF'}")

    neg_active = axis_val < -DEADZONE
    if neg_active != states[neg_key]:
        states[neg_key] = neg_active
        send_udp(f"{neg_key.upper()}_{'ON' if neg_active else 'OFF'}")

print(f"Listening for Axis {SEQ_AXIS} for SEQ toggle.")

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    #assign axis values to activate states
    handle_movement(controller.get_axis(0), "a", "d")
    handle_movement(controller.get_axis(1), "w", "s")

    seq_val = controller.get_axis(SEQ_AXIS)

    # Check if axis is pushed past threshold
    if seq_val > SEQ_THRESHOLD:
        if not states["seq_latched"]:
            # Perform the toggle
            states["seq_active"] = not states["seq_active"]
            status = "ON" if states["seq_active"] else "OFF"
            send_udp(f"SEQ_{status}")

            # Lock the latch until the axis is released
            states["seq_latched"] = True
    else:
        # Reset the latch when the axis returns to neutral
        states["seq_latched"] = False

    # Limit loop speed to save CPU (approx 60Hz)
    clock.tick(60)

pygame.quit()
