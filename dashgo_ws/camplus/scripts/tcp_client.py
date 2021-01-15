import socket
import time

#HOST, PORT = "10.0.2.15", 8888
HOST, PORT = "192.168.112.134", 8888

host112_list = ["131","132", "133","134"]
data = "_check_msg_"
is_shutdown = False
k = 0

try:
    while not is_shutdown:
        # Create a socket (SOCK_STREAM means a TCP socket)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        
        # Connect to server and send data
        HOST = "192.168.112."
        HOST += host112_list[k]

        try:
            sock.connect((HOST, PORT))
               
            print((HOST, PORT, data))
            sock.sendall(data + "\n")
            
            # Receive data from the server and shut down
            received = sock.recv(1024)
            sock.close()
            
            print "Received: {}".format(received)  
            
        except socket.timeout:
            print("connect to %s timeout\n" % HOST)
            
        except socket.error:
            print("connect to %s refused" % HOST)
        
        k = (k + 1) % len(host112_list)
        time.sleep(4)

except KeyboardInterrupt:
    print "stop on ctrl-C"
    is_shutdown = True
    #raise
