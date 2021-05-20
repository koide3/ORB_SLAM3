#!/usr/bin/python3
import os
import csv
import time
import numpy
from scipy.spatial.transform import Rotation

import tf
import rospy
import rosbag
import geometry_msgs.msg

from pyridescence import *


def load_ptx(filename):
	if os.path.exists(filename + '.npz'):
		print('use cached binary')
		data = numpy.load(filename + '.npz')

		trans = data['trans']
		points = data['points']

		return trans, points


	with open(filename, 'r') as f:
		print('parsing header')
		width = int(f.readline())
		height = int(f.readline())

		for i in range(4):
			f.readline()

		trans = []
		trans += f.readline().split(' ')
		trans += f.readline().split(' ')
		trans += f.readline().split(' ')
		trans += f.readline().split(' ')
		trans = numpy.float64(trans).reshape(4, 4).T

		print('parsing points')
		points = numpy.loadtxt(f)
		print(points.shape)

	numpy.savez(filename + '.npz', trans=trans, points=points)
	return trans, points



def read_vodom_traj(bag_filename):
	frames = []
	with rosbag.Bag(bag_filename, 'r') as bag:
		for topic, msg, stamp in bag.read_messages():
			time = int(msg.header.stamp.secs) + float(msg.header.stamp.nsecs) / 1e9
			pos = msg.pose.position
			quat = msg.pose.orientation
			rot = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])

			pose = numpy.identity(4)
			pose[:3, 3] = [pos.x, pos.y, pos.z]
			pose[:3, :3] = rot.as_matrix()

			frames.append((time, pose))

	return frames


def main():
	trans, map_points = load_ptx('/home/koide/datasets/oishi_bag/scan031_2x2.ptx')
	map_points = trans[:3, :3].dot(map_points[:, :3].T).T + trans[:3, 3] + [0, 0, 70]
	map_points = map_points[map_points[:, 2] < 0.0]

	viewer = guik.LightViewer.instance()
	viewer.set_draw_xy_grid(False)
	viewer.update_drawable('map', glk.create_pointcloud_buffer(map_points), guik.Rainbow())

	map_trans = [1.000000, 0.000000, 0.000000, 2.646897, 0.000000, 1.000000, 0.000000, 1.102584, 0.000000, 0.000000, 1.000000, 70.574593, 0.000000, 0.000000, 0.000000, 1.000000]
	vodom_trans = [2.224298, -0.089656, 2.435025, -0.571398, -2.435777, -0.171206, 2.218681, -2.630915, 0.066068, -3.293557, -0.181617, -0.226525, 0.000000, 0.000000, 0.000000, 1.000000]

	map_trans = numpy.array(map_trans).reshape(4, 4)
	vodom_trans = numpy.array(vodom_trans).reshape(4, 4)
	trans = numpy.linalg.inv(map_trans).dot(vodom_trans)

	vodom_traj = read_vodom_traj('/home/koide/datasets/oishi_bag/vodom.bag')

	frames = []
	for i, frame in enumerate(vodom_traj):
		stamp, pose = frame
		matrix = trans.dot(pose)
		frames.append((stamp, matrix))

		matrix[:3, 3] += [0, 0, 70]
		matrix[:3, :3] *= 0.2

		viewer.update_drawable('frame_%d' % i, glk.primitives.coordinate_system(), guik.VertexColor(matrix))

	viewer.spin()

	rospy.init_node('pub_traj')

	tf_broadcaster = tf.TransformBroadcaster()

	while not rospy.is_shutdown():
		if len(frames) == 0:
			break

		now = rospy.get_time() + 0.1
		stamp, matrix = frames[0]
		if stamp > now:
			time.sleep(1e-3)
			continue

		frames = frames[1:]

		print(stamp)
		print(matrix)

		pos = matrix[:3, 3]
		quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
		tf_broadcaster.sendTransform(pos, quat, rospy.Time(stamp), 'camera', 'map')




if __name__ == '__main__':
	main()
