#!/usr/bin/env python
#import rospy
#from std_msgs.msg import String
import SocketServer
import redis

interface_data = "$POT"
active_pass = 0
def callback_interface(data):
    global interface_data
    interface_data = data.data

class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server,
    implemented communication to the client.
    """
    def handle(self):
        global interface_data
        global active_pass
        self.r= redis.Redis('localhost')
        self.data = self.request.recv(1024).strip()

        if self.data == "_check_msg_":
            active_count = int(self.r.get('pot_mb_active_count'))
            if active_count != active_pass:
                active_pass = active_count
                interface_data = self.r.get('interface_msg')
            else:
                interface_data = "$POT"
            self.request.sendall(interface_data)
            
            

        elif self.data == "_get_count_":
            interface_data = self.r.get('interface_count')
            self.request.sendall(interface_data)
            interface_data = "$POT"

        elif "_set_param_" in self.data:
            response = self.r.get('interface_msg')[:17]
            self.r.set('write_param', self.data)
            self.request.sendall(response)
            
        elif self.data == "_clr_count_":
            self.r.set('clear_count', True)
            self.request.sendall("OK")


if __name__ == "__main__":
    try:
        #rospy.init_node('pot_socket', anonymous=True)  
        #rospy.Subscriber('/interface',String, callback_interface)

        #HOST, PORT = "192.168.31.200", 8888
        HOST, PORT = "localhost", 8888

        print((HOST, PORT))
        # Create the server, binding to localhost on port 9999
        server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()

    except:
        print("tcp is close")

    finally:
        print("socket is shutdown")
