import rospy
import baxter_interface
from baxter_pykdl import baxter_kinematics
import numpy as np

# give endpoint velocity commands to baxter 

class velocity(object):
    def __init__(self):
        self.limb_name = 'left'
        self.other_limb_name = 'right'
        self.limb = baxter_interface.Limb('left')
        self.kinematics = baxter_kinematics('left')
        self.jinv = self.kinematics.jacobian_pseudo_inverse()
        self.Vx = np.array([0.01, 0, 0, 0, 0, 0])

    def compute_joint_velocities(self, cartesian_velocities, jinv):
        joint_v = np.squeeze(np.asarray(self.jinv.dot(cartesian_velocities)))
        jinv = baxter_kinematics(self.limb_name).jacobian_pseudo_inverse()
        joint_v = np.squeeze(np.asarray(jinv.dot(cartesian_velocities)))
    	joint_v_dict = {'{}_{}'.format(self.limb_name, joint_name): val for
            joint_name, val in zip(['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'],
                                    joint_v)}
    	joint_v_dict.update({'{}_{}'.format(self.other_limb_name, joint_name): 0.0 for
            joint_name in ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']})
    	joint_v_dict.update({'head_nod':0.0 , 'head_pan':0.0, 'torso_t0':0.0})
        return joint_v_dict

def main():
    for i in range(100):
        joint_velo = v.compute_joint_velocities(v.Vx,v.jinv)
        v.limb.set_joint_velocities(joint_velo)

if __name__ == '__main__':
    rospy.init_node('velotrial')
    v = velocity()
    main()
    #rospy.spin()
