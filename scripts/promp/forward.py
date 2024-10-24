
import numpy as np
import tf

class FrankaKinematics:
    def __init__(self):
        self.dh_param = None
    
    def _tf_matrix(self, i, dh):
        a = dh[i][0]
        d = dh[i][1]
        alpha = dh[i][2]
        theta = dh[i][3]
        q = theta

        return np.array([[np.cos(q), -np.sin(q), 0, a],
                        [np.sin(q) * np.cos(alpha), np.cos(q) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
                        [np.sin(q) * np.sin(alpha), np.cos(q) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                        [0, 0, 0, 1]])
        
    def _dh_param(self, joint_angles):
        dh_params = [[0, 0.333, 0, joint_angles[0]],
                     [0, 0, -np.pi/2, joint_angles[1]],
                     [0, 0.316, np.pi/2, joint_angles[2]],
                     [0.0825, 0, np.pi/2, joint_angles[3]],
                     [-0.0825, 0.384, -np.pi/2, joint_angles[4]],
                     [0, 0, np.pi/2, joint_angles[5]],
                     [0.088, 0, np.pi/2, joint_angles[6]],
                     [0, 0.107, 0, 0],
                     [0, 0, 0, -np.pi/4],
                     [0.0, 0.1034, 0, 0]]
        return dh_params
    
    def forward_kinematics(self, q):
        dh_params = self._dh_param(q)
        T = np.eye(4)
        for i in range(len(dh_params)):
           T = T @ self._tf_matrix(i, dh_params)
        O_T_EE = np.array(T)
        return O_T_EE

    # def __rotMatToEul(self,rotMat):
    #     eul = np.zeros(3)
    #     eul[0] =  np.arctan2(-rotMat[2,1],rotMat[2,2])
    #     eul[1] =  np.arctan2(rotMat[2,0],np.sqrt(rotMat[2,1]**2+rotMat[2,2]**2))
    #     eul[2] =  np.arctan2(-rotMat[1,0],rotMat[0,0])
    #     return eul

    def __end_eff(self, q, As):
        # end_eff = As
        # pos = end_eff[0:3,3]
        # orientation = self.__rotMatToEul(end_eff[0:3,0:3].transpose())
        
        orientation = tf.transformations.quaternion_from_matrix(As) # from the other repo
        pos = tf.transformations.translation_from_matrix(As)
        return pos, orientation
    
    def end_effector(self,q):
        As = self.forward_kinematics(q)
        # end_eff = As
        # pos = end_eff[0:3,3]
        # orientation = self.__rotMatToEul(end_eff[0:3,0:3].transpose())
        
        # orientation = tf.transformations.euler_from_matrix(As) # from the other repo
        # pos = tf.transformations.translation_from_matrix(As)
        return self.__end_eff(q, As)
        # return pos, orientation
    
    def end_eff_trajectory(self, Q):
        pos = []
        orientation = []
        for t in range(len(Q)):
            post, ort = self.end_effector(Q[t])
            pos.append(post)
            orientation.append(ort)
        return np.array(pos), np.array(orientation)
    

    def jacobian(self, q):
        O_T_EE = self.forward_kinematics(q)
        #j1 = self.__num_jac(q,1e-6)
        J = self.__analytic_jac(q, O_T_EE)
        #print "diff = ", np.linalg.norm(j1 - j2[0:3,:])
        return J

    def __analytic_jac(self, q, O_T_EE):
        dh_params = self._dh_param(q)
        J = np.zeros((6, len(dh_params)))
        T = np.identity(4)
        
        for i in range(len(dh_params)):
            T = T @ self._tf_matrix(i, dh_params)
            p = O_T_EE[:3, 3] - T[:3, 3]
            z = T[:3, 2]
            J[:3, i] = np.cross(z, p)
            J[3:, i] = z
        return J[:, :7]

    def position_and_jac(self, q):
        As = self.forward_kinematics(q)
        jac = self.__analytic_jac(q, As)
        pos, ori = self.__end_eff(q, As)
        return pos, jac, ori
    
    
    


class BarrettKinematics:
    """ Forward kinematics object for the Barrett Wam
    This class implements the forwark kinematics functionality for the
    Barrett Wam arm used in the table tennis setup at the MPI. The end
    effector position can be changes with the endeff parameter received
    in the constructor.
    """

    def __init__(self, endeff = [0.0, 0.0, 0.3, 0.0, 0.0, 0.0]):
        self.ZSFE = 0.346
        self.ZHR = 0.505
        self.YEB = 0.045
        self.ZEB = 0.045
        self.YWR = -0.045
        self.ZWR = 0.045
        self.ZWFE = 0.255
        self.endeff = endeff

    def _link_matrices(self,q):
        cq = np.cos(q)
        sq = np.sin(q)

        sa=np.sin(self.endeff[3])
        ca=np.cos(self.endeff[3])

        sb=np.sin(self.endeff[4])
        cb=np.cos(self.endeff[4])

        sg=np.sin(self.endeff[5])
        cg=np.cos(self.endeff[5])

        hi00 = np.array([[1,0,0,0],
                         [0,-1,0,0],
                         [0,0,-1,0],
                         [0,0,0,1]])
        # hi00 = np.array([[1,0,0,0],
        #                  [0,1,0,0],
        #                  [0,0,1,0],
        #                  [0,0,0,1]])
        hi01 = np.array([[cq[0],    -sq[0], 0,  0],
                         [sq[0],    cq[0],  0,  0],
                         [0,        0,      1,  self.ZSFE],
                         [0,        0,      0,      1]])
        hi12 = np.array([[0,        0,      -1,     0],
                         [sq[1],    cq[1],  0,      0],
                         [cq[1]     ,-sq[1],0,      0],
                         [0,        0,      0,      1]])
        hi23 = np.array([[0,        0,      1,      self.ZHR],
                         [sq[2],    cq[2],  0,      0],
                         [-cq[2],   sq[2],  0,      0],
                         [0,0,0,1]])
        hi34 = np.array([[0,        0,      -1,     0],
                         [sq[3],    cq[3],  0,      self.YEB],
                         [cq[3],    -sq[3], 0,      self.ZEB],
                         [0,        0,      0,      1]])
        hi45 = np.array([[0,        0,      1,      self.ZWR],
                         [sq[4],    cq[4],  0,      self.YWR],
                         [-cq[4],   sq[4],  0,      0],
                         [0,        0,      0,      1]])
        hi56 = np.array([[0,        0,      -1,     0],
                         [sq[5],    cq[5],  0,      0],
                         [cq[5],    -sq[5], 0,      self.ZWFE],
                         [0,        0,      0,      1]])
        hi67 = np.array([[0,        0,      1,      0],
                         [sq[6],    cq[6],  0,      0],
                         [-cq[6],   sq[6],  0,      0],
                         [0,        0,      0,      1]])
        hi78 = np.array([[cb*cg,                -(cb*sg),           sb,         self.endeff[0]], 
                         [cg*sa*sb + ca*sg,     ca*cg - sa*sb*sg,   -(cb*sa),   self.endeff[1]], 
                         [-(ca*cg*sb) + sa*sg,  cg*sa + ca*sb*sg,   ca*cb,      self.endeff[2]], 
                         [0,                    0,                  0,          1]])
        return [hi00,hi01,hi12,hi23,hi34,hi45,hi56,hi67,hi78]

    def forward_kinematics(self,q):
        H = self._link_matrices(q)
        A = H[0]
        ans = []
        for i in range(1, len(H)):
            A = np.dot(A, H[i])
            ans.append(A)
        return ans

    def __rotMatToEul(self,rotMat):
        eul = np.zeros(3)
        eul[0] =  np.arctan2(-rotMat[2,1],rotMat[2,2])
        eul[1] =   np.arctan2(rotMat[2,0],np.sqrt(rotMat[2,1]**2+rotMat[2,2]**2))
        eul[2] =   np.arctan2(-rotMat[1,0],rotMat[0,0])
        return eul

    def __end_eff(self, q, As):
        end_eff = As[-1]
        pos = end_eff[0:3,3]
        orientation = self.__rotMatToEul(end_eff[0:3,0:3].transpose())
        return pos, orientation

    def end_effector(self,q):
        As = self.forward_kinematics(q)
        return self.__end_eff(q, As)

    def __num_jac(self,q,eps):
        ans = np.zeros((3,len(q)))
        fx,ori = self.end_effector(q)
        for i in range(len(q)):
            q[i] += eps
            fxh,ori = self.end_effector(q)
            ans[:,i] = (fxh - fx) / eps
            q[i] -= eps
        return ans

    def __analytic_jac(self, q, As):
        ans = np.zeros((6,len(q)))
        pe = As[-1][0:3,3]
        for i in range(len(q)):
            zprev = As[i][0:3,2]
            pprev = As[i][0:3,3]
            ans[0:3,i] = np.cross(zprev, pe - pprev)
            ans[3:6,i] = zprev
        return ans

    def jacobian(self, q):
        As = self.forward_kinematics(q)
        #j1 = self.__num_jac(q,1e-6)
        j2 = self.__analytic_jac(q, As)
        #print "diff = ", np.linalg.norm(j1 - j2[0:3,:])
        return j2

    def position_and_jac(self, q):
        As = self.forward_kinematics(q)
        jac = self.__analytic_jac(q, As)
        pos, ori = self.__end_eff(q, As)
        return pos, jac, ori

    def end_eff_trajectory(self, Q):
        pos = []
        orientation = []
        for t in range(len(Q)):
            post, ort = self.end_effector(Q[t])
            pos.append(post)
            orientation.append(ort)
        return np.array(pos), np.array(orientation)
