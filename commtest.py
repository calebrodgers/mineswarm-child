# Jake Delyster, August 2024
from machine import UART, Pin
from zumo_2040_robot import robot
import time

# Configure UART connection
uart1 = UART(1, baudrate=115200, tx=Pin(20), rx=Pin(21))

# Sends an AT command
def send_at_command(cmd, wait_time=1):
    print("Sending:", cmd)
    cmd += '\r\n'  # Append carriage return and newline to the command
    uart1.write(cmd.encode('utf-8'))  # Encode the command as UTF-8 before sending
    time.sleep(wait_time)
    response = uart1.read()
    if response:
        try:
            print("Received:", response.decode('utf-8'))  # Try to decode response as UTF-8
        except Exception as e:  # Catch any exception during decoding
            print("Received (raw):", response)  # Print raw response if decoding fails
            print("Decode Error:", e)  # Print error message
    return response

def setup_wifi(ssid, password):
    # Test AT command
    send_at_command("AT+RST", 5)  # Reset ESP8266
    send_at_command("ATE0")       # Disable echo
    send_at_command("AT")         # Test AT command
    # Set mode to STA (Station)
    send_at_command("AT+CWMODE=1")
    send_at_command("AT+CWDHCP=0,0")
    #固定小机器人的ip
    send_at_command("AT+CIPSTA=\"192.168.0.35\",\"192.168.0.1\",\"255.255.255.0\"")

    # Connect to AP
    send_at_command("AT+CWJAP=\"{}\",\"{}\"".format(ssid, password), 20)
    send_at_command("AT+CIFSR", 10)


def setup_udp_client(remote_ip, remote_port):
    """
    Setup UDP client to send data to a specific remote IP and port.
    """
    #command = 'AT+CIPSTART="UDP","{}",{},{}'.format(remote_ip, remote_port, local_port)
    command = 'AT+CIPSTART="UDP","{}",{}'.format(remote_ip, remote_port)
    response = send_at_command(command)
    if "OK" in response.decode('utf-8'):
        print("UDP client setup successful. Target IP: {} on port: {}".format(remote_ip, remote_port))
        return True
    else:
        print("Failed to set up UDP client:", response)
        return False


def send_udp_data(data):
    """
    Send data over UDP to the previously specified IP and port.
    """
    length = len(data)
    send_command = 'AT+CIPSEND={}'.format(length)
    response = send_at_command(send_command, 0.5)  # Increase time if needed
    if ">" in response.decode('utf-8'):  # Check if ready to receive data
        print("Ready to send data.")
        response = send_at_command(data, 0.5)  # Send the actual data
        if "SEND OK" in response.decode('utf-8'):
            print("Data sent successfully")
        else:
            print("Failed to send data:", response)
    else:
        print("Failed to initiate send:", response)


def setup_udp_server(local_port):
    """
    Setup a UDP server on a specified local port to listen for incoming data.
    """
    response = send_at_command("AT+CIPSTART=\"UDP\",\"192.168.0.35\",0,{},2".format(local_port))
    if "OK" in response.decode('utf-8'):
        print("UDP server setup successful on local port: {}".format(local_port))
        return True
    else:
        print("Failed to set up UDP server:", response)
        return False


def listen_udp():
    """
    Listen for incoming UDP data continuously for a specified duration in seconds.
    """

    while True:
        response = uart1.read()
        if response:
            try:
                decoded_response = response.decode('utf-8')
                if "+IPD" in decoded_response:
                    parts = decoded_response.split(':')
                    if len(parts) > 1:
                        data = parts[1].strip()
                        print(data)
                        return data
                return decoded_response.strip()
            except Exception as e:  # Handle decoding exceptions
                print("Received (raw):", response)  # Print raw response if decoding fails
                print("Decode Error:", e)


# Wifi SSID and password
ssid = 'TP-Link_EBC6'
password = '58221471'

# Store connection variables
remote_ip = "192.168.0.107"
remote_port = "50000"
local_port = "1112"

# Establish connection 
setup_wifi(ssid, password)
print("\n\nWifi connection established!\n\n")

setup_udp_client(remote_ip, remote_port)
time.sleep(1)
send_udp_data("Hello world")

print("Setting up UDP connection...")
if setup_udp_server(local_port):
    print("\n\nSet up UDP server\n\n")

time.sleep(1)

print("Waiting for data")
data = listen_udp()
print(data)