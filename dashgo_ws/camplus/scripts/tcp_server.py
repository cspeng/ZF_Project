#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import SocketServer
import memcache

interface_data = "$ACAM"

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
        self.data = self.request.recv(1024).strip()
        if self.data == "_check_msg_":
            self.request.sendall(interface_data)
            interface_data = "$ACAM"
            res= memcache.Client(["127.0.0.1:11211"])
            if res.get("arr_ishalt"):
                interface_data = "$ACAM,_QUIT_\r\n"                
            else:
                interface_data = "$ACAM"

if __name__ == "__main__":
    try:
        rospy.init_node('camplus_socket', anonymous=True)  
        rospy.Subscriber('/interface',String, callback_interface)

        #HOST, PORT = "192.168.146.93", 8888
        HOST, PORT = "192.168.31.200", 8888

        print((HOST, PORT))
        server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

        # Activate the server; this will keep running until interrupt
        server.serve_forever()

    except rospy.ROSInterruptException:
        rospy.loginfo("tcp is close")

    finally:
        rospy.loginfo("socket is shutdown")
