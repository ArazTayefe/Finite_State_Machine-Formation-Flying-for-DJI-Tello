import socket
import time

# IP and port of the Tello drone's command interface
tello_address = ('192.168.10.1', 8889)

# Create a UDP connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind to an address and port to receive messages from the drone
sock.bind(('', 9000))  # Ensure this port is available and not blocked by your firewall

# Command to switch the Tello to station mode: command to connect to your Wi-Fi
command = 'command'
wifi_command = f'ap your_SSID your_password.'
# Send commands
sock.sendto(command.encode(), tello_address)
time.sleep(1)  # Wait for the drone to be ready to receive next command

response, _ = sock.recvfrom(1024)  # Waiting for response from drone
print(f'Received: {response.decode()}')

sock.sendto(wifi_command.encode(), tello_address)
time.sleep(1)  # Give the drone some time to process the command

response, _ = sock.recvfrom(1024)  # Waiting for response from drone
print(f'Received: {response.decode()}')

sock.close()  # Close the socket when done
