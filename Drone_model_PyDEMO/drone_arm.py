# coding : utf-8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

def Rot(theta, rot_axe):
	if rot_axe == "x":
		Rot_array = np.array([[1,0,0], [0,np.cos(theta),-np.sin(theta)], [0,np.sin(theta),np.cos(theta)]])
	elif rot_axe == "y":
		Rot_array = np.array([[np.cos(theta),0,np.sin(theta)], [0,1,0], [-np.sin(theta),0,np.cos(theta)]])
	elif rot_axe == "z":
		Rot_array = np.array([[np.cos(theta),-np.sin(theta),0], [np.sin(theta),np.cos(theta),0], [0,0,1]])
	else:
		print ("Not enough input")

	return Rot_array

class Drone_arm:

	def __init__(self, DRONE_ARM_LENGTH, INIT_ANGLE): # constructor
		init_angle = np.deg2rad(INIT_ANGLE)
		
		self.vec_init = np.array([DRONE_ARM_LENGTH, 0, 0])
		self.vec_org=np.dot(Rot(init_angle,"z"), self.vec_init)
		self.posi_offset=np.zeros(self.vec_init.shape)
		self.lift_force = 0
		self.motor_torque = 0
		self.motor_speed = 0
		self.motor_current = 0
		self.d_arm_angle = np.array([0,0,INIT_ANGLE])

	def __del__(self): # de-constructor
		print ("delite class")


	##########################################################################
	### method
	def Mcurrent2Mdynamics(self):
		
		# process #

		# result
		self.motor_torque = 0
		self.motor_speed = 0

	def Mcurrent2Mdynamics(self, m_current):
		self.motor_current = m_current
		
		# process #
		
		# result
		self.motor_torque = 0
		self.motor_speed = 0

	def Mdynamics2LiftForce(self):
		
		# process #

		# result
		self.lift_force = 0

	def Mdynamics2LiftForce(self, m_torque, m_speed): 
		self.motor_torque = m_torque
		self.motor_speed = m_speed

		# process #
		
		# result
		self.lift_force = 0

	def calc_rot_arm(self, rot_angle_array): # deg
		# deg2rad
		self.d_arm_angle = self.d_arm_angle + rot_angle_array
		rot_angle_array_rad=np.deg2rad(self.d_arm_angle)


		Rx=Rot(rot_angle_array_rad[0], "x")
		Ry=Rot(rot_angle_array_rad[1], "y")
		Rz=Rot(rot_angle_array_rad[2], "z")

		# result
		self.vec_rot = np.dot(Rz, self.vec_init)
		self.vec_rot = np.dot(Rx, self.vec_rot)
		self.vec_rot = np.dot(Ry, self.vec_rot)
		
	def calc_trans_arm(self, r_trans_array):
		self.posi_offset = self.posi_offset + r_trans_array
		# result
		self.vec_org = self.vec_rot + self.posi_offset

	def calc_move_arm(self, r_trans_array, rot_angle_array):
		pass

def disp_vec_3D(g_obj,posi_vec, vec, col):
	arm_vec = vec - posi_vec
	g_obj.quiver(posi_vec[0], posi_vec[1], posi_vec[2], arm_vec[0], arm_vec[1], arm_vec[2],color = col, length = 1,arrow_length_ratio = 0.1)

def make_3Dgraph_asset(g_obj, x_range, y_range, z_range, title):
	g_obj.grid()
	g_obj.set_title(title)
	g_obj.set_xlabel("x", fontsize = 16)
	g_obj.set_ylabel("y", fontsize = 16)
	g_obj.set_zlabel("z", fontsize = 16)
	g_obj.set_xlim(x_range[0], x_range[1])
	g_obj.set_ylim(y_range[0], y_range[1])
	g_obj.set_zlim(z_range[0], z_range[1])


if __name__ == "__main__":
	try:
		arm1 = Drone_arm(10,0)
		arm2 = Drone_arm(10,90)
		arm3 = Drone_arm(10,180)
		arm4 = Drone_arm(10,270)
		arm = [arm1, arm2, arm3, arm4]
		color=["red", "orange", "green", "blue"]
		print ("Arm vector\n", "arm1: ", arm[0].vec_org, "\narm2: ", arm[1].vec_org, "\narm3: ", arm[2].vec_org, "\narm4: ", arm[3].vec_org)

		fig = plt.figure(figsize = (6, 6))
		ax = fig.add_subplot(111, projection='3d')
		plt.pause(.1)

		theta = np.zeros(arm1.vec_org.shape)
		trans = np.zeros(arm1.vec_org.shape)
		pre_theta = np.zeros(arm1.vec_org.shape)
		pre_trans = np.zeros(arm1.vec_org.shape)		
		
		
		i = 0
		j = 0
		theta_dev = 1.0
		trans_dev = 5

		input("Are u ready?:")

		while True:
			# *** derive arm vector process is so BAD! not correct! 2020/06/28 

			dtrans= trans - pre_trans
			dtheta= theta - pre_theta

			plt.cla()
			make_3Dgraph_asset(ax, [-30,30], [-30,30], [-30,30], "t="+str(i/10)+" [sec]")
			for ii in range(len(arm)):	
				arm[ii].calc_rot_arm(dtheta)
				arm[ii].calc_trans_arm(dtrans)
				disp_vec_3D(ax, trans, arm[ii].vec_org, color[ii])
				disp_vec_3D(ax, [0,0,0], arm[ii].vec_rot, color[ii])
				print ("arm["+str(ii)+"]: ", arm[ii].vec_org, ", vec norm: ",np.linalg.norm(arm[ii].vec_org),", angle: ", arm[ii].d_arm_angle)
			plt.pause(0.1)
			# plt.show()
			print (i,"\ntrans: ", trans, " , theta: ",theta)
			print ("pre_trans: ", pre_trans, " , pre_theta: ",pre_theta)
			print ("dtrans: ", dtrans, " , dtheta: ",dtheta)

			pre_trans = np.copy(trans)
			pre_theta = np.copy(theta)
			# pre_trans = trans
			# pre_theta = theta

			trans[0] = 5*trans_dev*np.cos((i/25)*np.pi)
			trans[1] = 5*trans_dev*np.sin((i/25)*np.pi)
			trans[2] = 5*trans_dev*(1-np.exp(-i/50))
			theta[0] = 5*np.sin(j/30*np.pi)
			theta[1] = 20*np.sin(j/30*np.pi)
			theta[2] = 10*np.sin(j/25*np.pi)

			trans = trans + 2*(np.random.rand(3) - 0.5*np.ones(trans.shape))
			theta = theta + 5*(np.random.rand(3) - 0.5*np.ones(trans.shape))

			i += 1
			j += 2

		# 	ani = animation.FuncAnimation(fig, interval=500)
		# ani.save("plot2.gif",writer='imagemagick')
		# plt.cla()
		# make_3Dgraph_asset(ax, [-30,30], [-30,30], [-30,30])
		# for ii in range(len(arm)):	
		# 	arm[ii].calc_trans_arm(trans)
		# 	arm[ii].calc_rot_arm(theta)
		# 	disp_vec_3D(ax, trans, arm[ii].vec, "red")
		# 	print ("arm["+str(ii)+"]: \n", arm[ii].vec, "\n", arm[ii].d_arm_angle)
		# plt.pause(0.1)

		# trans[0] = 0
		# trans[1] = 0
		# trans[2] = 15
		# theta[2] = 45

		# print ("\ntrans: ", trans, "\ntheta: ",theta)

		# # plt.cla()
		# make_3Dgraph_asset(ax, [-30,30], [-30,30], [-30,30])
		# for ii in range(len(arm)):	
		# 	arm[ii].calc_trans_arm(trans)
		# 	arm[ii].calc_rot_arm(theta)
		# 	disp_vec_3D(ax, trans, arm[ii].vec, "blue")
		# 	print ("arm["+str(ii)+"]: \n", arm[ii].vec, "\n", arm[ii].d_arm_angle)
		# plt.pause(0.1)
		# plt.show()


	finally:
		pass
