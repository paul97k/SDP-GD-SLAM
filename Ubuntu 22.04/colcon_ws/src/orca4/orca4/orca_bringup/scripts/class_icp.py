#!/usr/bin/env python

import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

class Align2D:

	# params:
	#   source_points: numpy array containing points to align to the target set
	#                  points should be homogeneous, with one point per row
	#   target_points: numpy array containing points to which the source points
	#                  are to be aligned, points should be homogeneous with one
	#                  point per row
	#   initial_T:     initial estimate of the transform between target and source
	def __init__(self, source_points, target_points, prev_T):
		self.source = source_points
		self.target = target_points
		self.prev_T = prev_T
		self.target_tree = KDTree(target_points[:,:2])
		self.transform = self.AlignICP(20, 0.0000000001)

	# uses the iterative closest point algorithm to find the
	# transformation between the source and target point clouds
	# that minimizes the sum of squared errors between nearest
	# neighbors in the two point clouds
	# params:
	#   max_iter: int, max number of iterations
	#   min_delta_err: float, minimum change in alignment error

	def AlignICP(self, max_iter, min_delta_err):

		# src =  np.dot(self.source,self.prev_T.T)

		plotBool = True
		plotBool = False

		if plotBool: 
			plt.figure()
			plt.clf()
			plt.subplot(221)
			plt.plot(self.source[:,0],self.source[:,1], 'or',label="first scan")
			plt.plot(self.target[:,0],self.target[:,1], 'ob', label="second scan") 
			plt.title("Before initial guess")
			plt.legend()  # Add legend to differentiate the plots


			# plt.subplot(222)
			# plt.plot(src[:,0], src[:,1], 'or', label="first scan")  # Red circles for source points
			# plt.plot(self.target[:,0], self.target[:,1], 'ob', label="second scan")  # Blue circles for target points
			# plt.title("After initial guess")
			# plt.legend()  # Add legend to differentiate the plots


		mean_sq_error = 1.0e6 # initialize error as large number
		delta_err = 1.0e6    # change in error (used in stopping condition)
		T = self.prev_T
		num_iter = 0         # number of iterations
		tf_source = self.source
		new_err = 100

		# plt.figure(figsize=(8, 6))
		while  new_err > 0.3 and num_iter < max_iter:
		# while  delta_err > min_delta_err and num_iter < max_iter:
			# plt.clf() 
			# find correspondences via nearest-neighbor search
			matched_trg_pts,matched_src_pts,indices = self.FindCorrespondences(tf_source)

			# find alingment between source and corresponding target points via SVD
			# note: svd step doesn't use homogeneous points
			new_T = self.AlignSVD(matched_src_pts, matched_trg_pts)

			# update transformation between point sets
			
			T = np.dot(T,new_T)
			# T = new_T


			# apply transformation to the source points
			# tf_source = np.dot(self.source,T.T)
			tf_source = self.source @ T.T

			
			# plt.plot(self.target[:,0],self.target[:,1], 'or')
			# plt.plot(tf_source[:,0],tf_source[:,1], 'ob')
			# plt.title("after ICP")

			# find mean squared error between transformed source points and target points
			new_err = 0
			# for i in range(len(indices)):
			# 	if indices[i] != -1:
			# 		diff = tf_source[i,:1] - self.target[indices[i],:1]
			# 		new_err += np.dot(diff,diff.T)
    
			# new_err /= float(len(matched_trg_pts))

			# new_err = np.mean((tf_source[i,:2] - self.target[indices[i],:2]) ** 2)
			new_err = np.sqrt(np.mean((tf_source[:,:2].flatten() - self.target[:,:2].flatten()) ** 2))

			# update error and calculate delta error
			delta_err = abs(mean_sq_error - new_err)
			mean_sq_error = new_err
			# print(f"Iteration {num_iter}, Error: {new_err}, Delta Error: {delta_err}")

			# plt.pause(0.5)  
			num_iter += 1


		if plotBool: 
			plt.subplot(223)

			plt.plot(tf_source[:,0],tf_source[:,1], 'or', label="first scan")
			plt.plot(self.target[:,0],self.target[:,1], 'ob', label="second scan")
			plt.title("after ICP")
			# plt.xlim([-6,6])
			# plt.ylim([-6,6])
			plt.legend()  # Add legend to differentiate the plots
			# plt.show()
			# plt.pause(0.1)


		if new_err >10:
			# print(f"Iteration {num_iter}, Error: {new_err}, Delta Error: {delta_err}")
			tf_source = self.source
			T = self.prev_T
			# print("tf_source = self.source")
		
		# print(f"Iteration {num_iter}, Error: {new_err}, Delta Error: {delta_err}")
		return tf_source,new_err, T

	# finds nearest neighbors in the target point for all points
	# in the set of source points
	# params:
	#   src_pts: array of source points for which we will find neighbors
	#            points are assumed to be homogeneous
	# returns:
	#   array of nearest target points to the source points (not homogeneous)
	def FindCorrespondences(self,src_pts):

		# get distances to nearest neighbors and indices of nearest neighbors
		matched_src_pts = src_pts[:,:2]
		dist,indices = self.target_tree.query(matched_src_pts)

		# remove multiple associatons from index list
		# only retain closest associations
		unique = False
		while not unique:
			unique = True
			for i in range(len(indices)):
				if indices[i] == -1:
					continue
				for j in range(i+1,len(indices)):
					if indices[i] == indices[j]:
						if dist[i] < dist[j]:
							indices[j] = -1
						else:
							indices[i] = -1
							break
		# build array of nearest neighbor target points
		# and remove unmatched source points
		point_list = []
		src_idx = 0
		for idx in indices:
			if idx != -1:
				point_list.append(self.target[idx,:])
				src_idx += 1
			else:
				matched_src_pts = np.delete(matched_src_pts,src_idx,axis=0)

		matched_pts = np.array(point_list)

		return matched_pts[:,:2],matched_src_pts,indices

	# uses singular value decomposition to find the
	# transformation from the target to the source point cloud
	# assumes source and target point clounds are ordered such that
	# corresponding points are at the same indices in each array
	#
	# params:
	#   source: numpy array representing source pointcloud
	#   target: numpy array representing target pointcloud
	# returns:
	#   T: transformation between the two point clouds
	def AlignSVD(self, source, target):

		# first find the centroids of both point clouds
		src_centroid = self.GetCentroid(source)
		trg_centroid = self.GetCentroid(target)

		# get the point clouds in reference to their centroids
		source_centered = source - src_centroid
		target_centered = target - trg_centroid
		# Compute translation (centroid difference)
		t = trg_centroid - src_centroid
		
		# Construct transformation matrix (identity + translation)
		T = np.identity(3)
		T[:2, 2] = t  # Apply translation to the last column

		return T


		# # get cross covariance matrix M
		# M = np.dot(target_centered.T,source_centered)

		# # get singular value decomposition of the cross covariance matrix
		# U,W,V_t = np.linalg.svd(M)

		# # get rotation between the two point clouds
		# R = np.dot(U,V_t)

		# # get the translation (simply the difference between the point clound centroids)
		# t = np.expand_dims(trg_centroid,0).T - np.dot(R,np.expand_dims(src_centroid,0).T)

		# # assemble translation and rotation into a transformation matrix
		# T = np.identity(3)
		# T[:2,2] = np.squeeze(t)
		# T[:2,:2] = R

		# return T

	def GetCentroid(self, points):
		point_sum = np.sum(points,axis=0)
		return point_sum / float(len(points))
