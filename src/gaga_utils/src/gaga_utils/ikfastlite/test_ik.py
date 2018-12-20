from lis_pr2_pkg.uber_controller import Uber
import rospy
from gaga_utils.ikfastlite import ikLeft, ikRight
import tf.transformations as tfx
import numpy as np


class ikWrapper:

    def __init__(self, uc = None):

        self.FK = {  'l': ikLeft.leftFK,
                     'r': ikRight.rightFK
                    }

        self.IK = {  'l': ikLeft.leftIK,
                     'r': ikRight.rightIK
                    }

        if uc is None:
            self.uc = Uber()
        else:
            self.uc = uc

    def get_curr_fk(self, arm):
        joints = self.uc.get_joint_positions(arm)
        torso = self.uc.get_torso_pose()
        return self.get_fk(arm, torso, joints)

        
    def get_fk(self, arm, torso, joints):
        torso_arm_joints = [torso] + joints
        pos, rot = self.FK[arm](torso_arm_joints)

        rot_full = tfx.identity_matrix() 
        rot_full[0:3, 0:3] = rot
        quat = tfx.quaternion_from_matrix(rot_full).tolist()
        return pos,quat

    def get_ik(self, arm, torso, joints, (pos, quat)):
        # quat to 3x3 rotation matrix
        rot = tfx.quaternion_matrix(quat)[0:3, 0:3]
        rot = rot.tolist()
        upper_arm_roll_joint = joints[2]

        free_joints = [torso, upper_arm_roll_joint]
        joint_values = self.IK[arm](rot, pos, free_joints)
        if joint_values is None:
            rospy.logerr("No IK solution found")
            return None

        if len(joint_values) > 1:
            # remove torso and compute difference
            diff = np.array(joint_values)[:, 1:]  - np.array(joints)
            diff_sum = np.sum(np.abs(np.arctan2(np.sin(diff), np.cos(diff))), axis=1)
            best = np.argmin(diff_sum)
            joint_values = joint_values[best][1:]
        else:
            import pdb; pdb.set_trace()
        return joint_values

    def get_curr_ik(self, arm, (pos, quat)):
        joints = self.uc.get_joint_positions(arm)
        torso = self.uc.get_torso_pose()
        return self.get_ik(arm, torso, joints, (pos, quat))









if __name__ == "__main__":

    rospy.init_node("testik")
    uc = Uber()
    ikw = ikWrapper(uc)
    
    sign = 1

    steps =20 

    for i in range(steps):
        if (i % steps/4) == 0:
            sign *= -1

        pos, quat = ikw.get_curr_fk('l')
        print "forward kinematics solution", (pos,quat)

        print  "tf forward kinematics should agree: ", uc.return_cartesian_pose('l')
        x,y,z = pos
        y += sign*0.1
        z += sign*0.1
        pos = [x,y,z]
        

        print "ik solution", ikw.get_curr_ik('l', (pos, quat))

        joints = ikw.get_curr_ik('l', (pos, quat))
        if joints is None: continue
        uc.command_joint_pose('l', joints, time=2, blocking=True)


    raw_input("test complete")
