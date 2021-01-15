#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import SocketServer

interface_data = "$AGV"

def callback_interface(data):
    global interface_data
    interface_data = data.data

class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        global interface_data
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        #print self.data
        if self.data == "_check_msg_":
            #print "{} wrote:".format(self.client_address[0])
            #self.request.sendall("$AGV,1200,stopAA,______")
            self.request.sendall(interface_data)
            interface_data = "$AGV"

if __name__ == "__main__":
    try:
        rospy.init_node('triogo_socket', anonymous=True)  
        rospy.Subscriber('/interface',String, callback_interface)

        HOST, PORT = "192.168.31.200", 8888
        #HOST, PORT = "localhost", 8888

        print((HOST, PORT))
        # Create the server, binding to localhost on port 9999
        server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()

    except rospy.ROSInterruptException:
        rospy.loginfo("tcp is close")

    finally:
        rospy.loginfo("socket is shutdown")
