'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import wipe_forehead

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import *


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception): #TODO
        target_joints = {}
        # YOUR CODE HERE
        for i in range(len(keyframes[0])):
            names = keyframes[0][i]
            times = np.asarray(keyframes[1][i])
            angles = np.asarray([row[0] for row in keyframes[2][i]])
            yp = np.asarray([row[1][2] for row in keyframes[2][i]])
            time = perception.time

            '''plt.plot(times, angles, 'o')
            plt.show()'''

            x_i = np.linspace(np.min(times),np.max(times),100)

            if len(angles) > 3:
                f_spline = interp1d(times, angles, kind='cubic')
                y_i = f_spline(x_i)
            else:
                f_spline = interp1d(times, angles, kind='slinear')
                y_i = f_spline(x_i)

            '''plt.plot(x_i,y_i,'--')
            plt.plot(times, angles, 'o')
            plt.title(names)
            plt.show()'''

            target_joints[names] = f_spline(max(time % times[-1],min(times)))

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = wipe_forehead(0)
    agent.run()
