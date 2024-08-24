import time


def send_at_command(cmd, uart, wait_time=1):
	print("Sending:", cmd)
	cmd += '\r\n'  # Append carriage return and newline to the command
	uart.write(cmd.encode('utf-8'))  # Encode the command as UTF-8 before sending
	time.sleep(wait_time)
	response = uart.read()
	if response:
		try:
			print("Received:", response.decode('utf-8'))  # Try to decode response as UTF-8
		except Exception as e:  # Catch any exception during decoding
			print("Received (raw):", response)  # Print raw response if decoding fails
			print("Decode Error:", e)  # Print error message
	return response


def reset_esp8266(uart):
	uart.write("AT+RESTORE\r\n")
	time.sleep(1)  # Wait to ensure command is sent and module is reset


def setup_wifi(ssid, password, local_ip, uart):
	# Test AT command
	send_at_command("AT+RST", uart, 5)  # Reset ESP8266
	send_at_command("ATE0", uart)       # Disable echo
	send_at_command("AT", uart)         # Test AT command
	# Set mode to STA (Station)
	send_at_command("AT+CWMODE=1", uart)
	send_at_command("AT+CWDHCP=0,0", uart)
	#固定小机器人的ip
	send_at_command("AT+CIPSTA=\"{}\",\"192.168.0.1\",\"255.255.255.0\"".format(local_ip), uart)

	# Connect to AP
	send_at_command("AT+CWJAP=\"{}\",\"{}\"".format(ssid, password), uart, 20)
	send_at_command("AT+CIFSR", uart, 10)

	# Enable multiple connections 
	send_at_command("AT+CIPMUX=1", uart)

#192.168.0.35
def setup_udp_server(local_ip, local_port, uart):
	"""
	Setup a UDP server on a specified local port to listen for incoming data.
	"""
	response = send_at_command("AT+CIPSTART=1,\"UDP\",\"{}\",0,{},2".format(local_ip, local_port), uart)
	if "OK" in response.decode('utf-8'):
		print("UDP server setup successful on local port: {}".format(local_port), uart)
		return True
	else:
		print("Failed to set up UDP server:", response)
		return False


def setup_udp_client(remote_ip, remote_port, uart):
	"""
	Setup UDP client to send data to a specific remote IP and port.
	"""
	#command = 'AT+CIPSTART="UDP","{}",{},{}'.format(remote_ip, remote_port, local_port)
	command = 'AT+CIPSTART=0,"UDP","{}",{}'.format(remote_ip, remote_port)
	response = send_at_command(command, uart)
	if "OK" in response.decode('utf-8'):
		print("UDP client setup successful. Target IP: {} on port: {}".format(remote_ip, remote_port))
		return True
	else:
		print("Failed to set up UDP client:", response)
		return False


def send_udp_data(data, uart):
	length = len(data)
	send_command = 'AT+CIPSEND=0,{}'.format(length)
	response = send_at_command(send_command, uart, 0.5)  # Increase time if needed
	if ">" in response.decode('utf-8'):  # Check if ready to receive data
		print("Ready to send data.")
		response = send_at_command(data, uart, 0.5)  # Send the actual data
		if "SEND OK" in response.decode('utf-8'):
			print("Data sent successfully")
		else:
			print("Failed to send data:", response)
	else:
		print("Failed to initiate send:", response)


def close_connection(uart):
	"""
	Close the current connection.
	"""
	print("Closing any existing connection...")
	response = send_at_command("AT+CIPCLOSE", uart)
	if "OK" in response.decode('utf-8') or "ERROR" in response.decode('utf-8'):
		print("Connection closed successfully or no active connection.")
	else:
		print("Failed to close connection:", response)


def listen_udp(uart):
	"""
	Listen for incoming UDP data continuously for a specified duration in seconds.
	"""

	while True:
		response = uart.read()
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