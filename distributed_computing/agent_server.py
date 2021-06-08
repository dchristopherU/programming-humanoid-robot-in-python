'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py

https://github.com/marcelTUB
'''

from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

#from multiprocessing import Pool, cpu_count, freeze_support

# add PYTHONPATH
import socketserver
import threading
import concurrent.futures

import numpy as np

import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def __init__(self):
        InverseKinematicsAgent.__init__(self)
        self.ip = "127.0.0.1"
        self.port = 8000
        self.setServer()


    def setServer(self):
        #server = SimpleXMLRPCServer((self.ip, self.port),allow_none=True)
        server = SimpleXMLRPCServer((self.ip, self.port), allow_none=True, requestHandler=RequestHandler)
        """server.register_function(self.get_angle, "get_angle")
        server.register_function(self.set_angle, "set_angle")
        server.register_function(self.get_posture, "get_posture")
        server.register_function(self.execute_keyframes, "execute_keyframes")
        server.register_function(self.get_transform, "get_transform")
        server.register_function(self.set_transform, "set_transform")"""
        server.register_introspection_functions()
        server.register_instance(self)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()

        print("Server ready for function calls")




    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]

    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transform[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        if type(transform) == list:
            transform = np.array(transform)
        self.set_transforms(effector_name, transform)


if __name__ == '__main__':
    agent = ServerAgent()
    #T = np.identity(4)
    #T[-1, 1] = 0.05
    #T[-1, 2] = 0.26
    #agent.set_transform('LLeg', T)
    agent.run()


