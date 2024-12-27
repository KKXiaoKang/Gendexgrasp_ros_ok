import socket
import sys  # Import sys to access command-line arguments

def listen_for_broadcasts(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))  # Listen on all interfaces
    print(f"Listening for broadcasts on port {port}")

    while True:
        data, addr = sock.recvfrom(1024)
        print(f"Received message: {data.decode()} from {addr}")

if __name__ == "__main__":
    # Check if a port number is provided as a command-line argument, otherwise use 11000 as default
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 11000
    listen_for_broadcasts(port)
