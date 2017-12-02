#!/usr/bin/env python
# KDL Libraries
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import Robot
from pykdl_utils import *
from kdl_parser_py import *
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

# Python imports
from PyKDL import *

# Python Libs
import numpy as np
# Ros Libs
import rospy
import math
from math import *

# Math Utils
from math_utils import *

f = file('', 'r')
robot = Robot.from_xml_string(f.read("/home/mike/catkin_ws/src/csl_underwater_sim_gazebo/robot_description/lbv150_uvms.urdf"))
f.close()

# Get Tree
tree = kdl_tree_from_urdf_model(robot)
#tree = treeFromUrdfModel(robot)

# Get Chain
arm_chain = tree.getChain("base_link", "arm_tool_link")

# Create A Chain For the Platform
rov_chain = Chain()

#From Global Frame to Platform Vehicle Center Frame (when * is third)
#Translation along X axis
joint_xgf = Joint(Joint.TransX)
frame_xgf=Frame(Rotation.EulerZYX(0.0, 0.0, 0.0),Vector(0.0, 0.0, 0.0))
segment_xgf=Segment(joint_xgf,frame_xgf)
rov_chain.addSegment(segment_xgf)

#Translation along Y axis
joint_ygf = Joint(Joint.TransY)
frame_ygf=Frame(Rotation.EulerZYX(0.0, 0.0, 0.0),Vector(0.0, 0.0, 0.0))
segment_ygf=Segment(joint_ygf,frame_ygf)
rov_chain.addSegment(segment_ygf)

#Rotation about Z axis (*)
joint_zrgf = Joint(Joint.RotZ)
frame_zrgf=Frame(Rotation.EulerZYX(0.0, 0.0, 0.0),Vector(0.0, 0.0, 0.0))
segment_zrgf=Segment(joint_zrgf,frame_zrgf)
rov_chain.addSegment(segment_zrgf)

#From Platform Vehicle Center Frame to Arm Base Frame
joint_pvcf = Joint(Joint.None)
frame_pvcf=Frame(Rotation.EulerZYX(0.0, 0.0, 0.0),Vector(0.0, 0.0, 0.0))
segment_pvcf=Segment(joint_pvcf,frame_pvcf)
rov_chain.addSegment(segment_pvcf)

# Connect Chains
rov_chain.addChain(arm_chain)


# UVMS Chain
uvms_chain  = rov_chain

''' LBV Kinematics Class '''
class lbv_uvms_kinematics:
	
	def __init__(self):
		print "LBV UVMS CLASS INITIALIZED"
	
	
	''' Calculate Forward Kinemtics '''
	def fk(self, jointAngles):
			if isinstance (jointAngles, JntArray):
					joints = jointAngles
			elif isinstance (jointAngles, np.ndarray):
						joints = JntArray(8)
						for i in range (0,8):
							joints[i] = jointAngles[i]
			fk=ChainFkSolverPos_recursive(uvms_chain)
			finalFrame=Frame()
			fk.JntToCart(joints,finalFrame)
			return finalFrame

	''' Calculate Inverse Kinemtics 
	inputs: joint vector, position vector, orientation vector
	output: joint vector
	'''
	def ik(self, jointAngles, desVector,desRot):
			if isinstance (jointAngles, JntArray):
					joints = jointAngles
			elif isinstance (jointAngles, np.ndarray):
					joints = JntArray(8)
					for i in range (0,8):
						joints[i] = jointAngles[i]
			fk=ChainFkSolverPos_recursive(uvms_chain)
			vik=ChainIkSolverVel_pinv(uvms_chain)
			ik=ChainIkSolverPos_NR(uvms_chain,fk,vik)
			desiredFrame = Frame(desRot,desVector)
			q_out=JntArray(8)
			ik.CartToJnt(joints,desiredFrame,q_out)
			return q_out


	''' Calculate Jacobian Matrix'''
	def jac(self, jointAngles):
			if isinstance (jointAngles, JntArray):
					joints = jointAngles
			elif isinstance (jointAngles, np.ndarray):
					joints = JntArray(8)
					for i in range (0,8):
						joints[i] = jointAngles[i]
			jacobian = Jacobian(8)
			solver = ChainJntToJacSolver(uvms_chain)
			solver.JntToJac(joints,jacobian)
			return jacobian


