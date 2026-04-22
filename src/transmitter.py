import socket
from XInput import *

#configs
ESP_IP = "SET YOUR IP"  
PORT = "SET YOUR PORT"
DEADZONE = 0.4  #deadzone for stick

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Track current states so we don't spam the ESP32
pressed_keys = {"w": False, "s": False, "a": False, "d": False, "seq": False}

def send_udp(message):
    try:
        sock.sendto(bytes(message, "utf-8"), (ESP_IP, PORT))
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Error: {e}")

def check_axis(key, current_val, threshold, is_positive):
    """
    Determines if a direction is active based on stick position
    and updates the robot.
    """
    is_active = (current_val > threshold) if is_positive else (current_val < -threshold)

    if is_active != pressed_keys[key]:
        pressed_keys[key] = is_active
        msg = f"{key.upper()}_{'ON' if is_active else 'OFF'}"
        send_udp(msg)

def getInput():
    events = get_events()

    for event in events:
        if event.type == EVENT_CONNECTED:
            print("Controller detected")

        elif event.type == EVENT_DISCONNECTED:
            print("Controller lost")

        #automation button
        elif event.type == EVENT_BUTTON_PRESSED:
            if event.button == "A":
                # Flip the boolean value (True -> False or False -> True)
                pressed_keys["seq"] = not pressed_keys["seq"]

                # Determine which message to send based on the new state
                status = "ON" if pressed_keys["seq"] else "OFF"
                send_udp(f"SEQ_{status}")


        elif event.type == EVENT_STICK_MOVED:
            # We are using the LEFT stick for movement
            if event.stick == LEFT:
                # event.x and event.y are already normalized (-1.0 to 1.0)
                lx = event.x
                ly = event.y

                # Process Y-axis 
                check_axis('w', ly, DEADZONE, True)
                check_axis('s', ly, DEADZONE, False)

                # Process X-axis 
                check_axis('a', lx, DEADZONE, True)
                check_axis('d', lx, DEADZONE, False)

print(f"Controller Bridge Active. Sending to {ESP_IP}:{PORT}")

while True:
    getInput()
