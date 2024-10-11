import socket
import signal
import sys

# Define the IP address and port to listen on
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12345  # Replace with your desired port

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Flag to keep track of running status
is_running = True


def signal_handler(sig, frame):
    """Handle SIGINT for graceful shutdown."""
    global is_running
    print("\nGracefully shutting down...")
    is_running = False
    sock.close()  # Close the socket
    sys.exit(0)  # Exit the program


# Attach the SIGINT handler
signal.signal(signal.SIGINT, signal_handler)

print(f"Listening for UDP data on {UDP_IP}:{UDP_PORT}...")

while is_running:
    try:
        # Use non-blocking receive with timeout
        sock.settimeout(1)  # 1-second timeout
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        print(f"Received message from {addr}: {data.decode('utf-8', errors='ignore')}")
    except socket.timeout:
        # This ensures we periodically check the is_running flag
        continue
    except Exception as e:
        print(f"Error: {e}")
