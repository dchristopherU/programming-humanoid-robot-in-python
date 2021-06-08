'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, dot, array, matrix
import numpy as np
from math import atan2

import matplotlib.pyplot as plt


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE


        m_step = 0.15
        lambda_ = 1
        eps = np.random.random(len(self.chains[effector_name])) * 1e-5

        v = matrix([self.from_trans(transform)])
        effector_chains = self.chains[effector_name]

        for i in range(1000):
            self.forward_kinematics(self.perception.joint)

            '''X = np.arange(0, len(eps), 1)
            plt.plot(X, eps, 'o')
            plt.show()'''


            ecl = []
            for e in effector_chains:
                ecl.append(self.transforms[e])

            Te = matrix([self.from_trans(ecl[-1])]).T

            e = v - Te
            e[e > m_step] = m_step
            e[e < -m_step] = -m_step
            T = matrix([self.from_trans(i) for i in ecl]).T
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1
            theta = lambda_ * np.linalg.pinv(J) * e


            eps += np.asarray(theta.T)[0]


            i = 0
            for e in effector_chains:
                joint_angles[e] = eps[i]
                i = i + 1
                if (i == len(eps)):
                    break

            if np.linalg.norm(theta) < 1e-4:
                break

        return joint_angles


    def set_transforms(self, effector_name, transform):
            '''solve the inverse kinematics and control joints use the results
            '''
            # YOUR CODE HERE
            joint_angles = self.inverse_kinematics(effector_name, transform)

            names = self.chains[effector_name]
            times = [[0.0, 3.0]] * len(names)
            keys = []
            i = 0
            for name in names:
                keys.append([[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])
                i += 1

            self.keyframes = (names, times, keys)


    def from_trans(self, m):
            eps=0
            if m[1, 1] == m[2, 2]:
                eps = atan2(m[2, 1], m[1, 1])

            elif m[0, 0] == m[2, 2]:
                eps = atan2(m[0, 2], m[0, 0])

            elif m[0, 0] == m[1, 1]:
                eps = atan2(m[0, 1], m[0, 0])

            return m[3, 0], m[3, 1], m[3, 2], eps

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
