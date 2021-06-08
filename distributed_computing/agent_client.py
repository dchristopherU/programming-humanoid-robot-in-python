'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

from xmlrpc.client import ServerProxy

from keyframes import wipe_forehead
import threading

from multiprocessing import Pool, cpu_count, freeze_support
import numpy as np

import weakref

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''

    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        self.server_proxy = ServerProxy('http://localhost:8000')

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = threading.Thread(target=self.server_proxy.execute_keyframes, args=[keyframes])
        thread.start()
        thread.join()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        print("PorstHandler")
        transform = transform.tolist()
        thread = threading.Thread(target=self.server_proxy.set_transform, args=[effector_name, transform])
        thread.start()
        thread.join()




class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.proxy = ServerProxy('http://localhost:8000')
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.proxy.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        self.post.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()

    #proxy = ServerProxy('http://localhost:8000')
    #print(agent.proxy.system.listMethods())
    #print(proxy.get_angle('HeadYaw'))
    # TEST CODE HERE
    #keyframes = wipe_forehead(0)
    #print(keyframes)

    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transform('LLeg', T)


