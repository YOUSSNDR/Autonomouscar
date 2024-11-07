import socket
import time
# Set up the client
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#server_ip = '192.168.1.172'  # Replace with your Pico's IP address
server_ip = "192.168.181.88"
server_port = 80
message = "GET"  # Example message to send
client_socket.connect((server_ip, server_port))
#client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

def receive():
    time.sleep(0.1)
        # Send data (for example, an RGB command)
    client_socket.sendall(message.encode('utf-8'))
        #print("Sent:", message)

        # Receive the response
    response = client_socket.recv(1024)
    print("Response received:", response.decode('utf-8'))


receive()
client_socket.close()
