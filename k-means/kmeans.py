"""
==========================================
K-means for continuous feature recognition
==========================================

"""

# Author: Ying Siu Liang <yingsiu@liang.at>
# Last update: April 2017

import argparse
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

def km(x, y, n, k):
	"""
	 Calculates k-means.
	 Returns average squared intra-cluster distance
	 x,y : sample coordinates
	 k : number of clusters
	"""

	km = KMeans(n_clusters=k)

	k_fit = km.fit(zip(x,y))

	cc = k_fit.cluster_centers_
	labels = k_fit.labels_

	fig = plt.figure(k, figsize=(5, 5))
	plt.clf() # clear figure
	sp = 111 #"1%d%d" % (int(np.sqrt(n/2)),k)
	ax1 = fig.add_subplot(int(sp))

	# -- For testing --
	v = [ 0., 0.] #np.random.rand(2)
	ax1.scatter(v[0],v[1], s=50, c='100', marker="o", label='centers')
	# -- End testing --

	ax1.scatter(cc.T[0],cc.T[1], s=50, c='001', marker="o", label='centers')
	ax1.scatter(x,y, s=50, c=labels.astype(np.float), marker="o", label='samples')
	ax1.set_xlabel('x-coordinate')
	ax1.set_ylabel('y-coordinate')
	#plt.legend(loc='upper left');
	
	# Calculate average squared intra-cluster distance for cluster k
	eucl_dist = k_fit.transform(zip(x,y))
	temp = np.array([min(s) for s in eucl_dist])**2
	H = sum(temp) / (k*n)

	#print "**** " 
	#print eucl_dist
	#print temp

	return (H,cc)


def est_feature(x, y, n, H_max):
	"""
	Estimates feature's variance using k-means for different cluster sizes;
	Returns variances and clusters for which variance H < H_max
	 n : number of samples to generate randomly
	"""

	# runs k-means for clusters from 2 to k = sqrt(n/2)
	k = max(2,int(np.sqrt(n/2)))

	print "*** Estimate Feature:"
	print "    H_max = %f" % (H_max)

	for i in range(k):
		(H,c) = km(x, y, n, i+1)
		print "    %d clusters: variance %f " % (i+1,H)
		
		if H < H_max:
			return (H,c)
			break

	return (-1,[])


def check_predicate(cc, v, dmax):
	"""
	Create a set of logical predicates for relevant feature clusters
	Returns a boolean if the predicate is true for a value v
	dmax : max. distance allowed to a centroid, i.e. variation allowed during execution
	c : cluster center

	e.g. 
	dmax = 3cm for distances
	dmax = 20deg for angles
	"""
	v_distances = [np.linalg.norm(c-v) for c in cc]
	print "Check predicate: distances to clusters:"
	print v_distances
	if min(v_distances) <= dmax:
		return min(v_distances)
	else:
		return -1

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	#parser.add_argument('--h', help='help')
	parser.add_argument('-f', help='feature', default='position')
	parser.add_argument('-n', help='number of samples')
	parser.add_argument('-hmax', help='maximal accepted variance', default = 0.1)
	
	#parser.add_argument('-k', help='number of clusters', default = 3)
	args = parser.parse_args()

	# Step 1: Learning phase: create n samples
	if args.n :
		np.random.seed(4)
		n = int(args.n)
		x = np.random.rand(n)
		y = np.random.rand(n)
	else :
		X = np.array([[0, 2], [0, 4], [0, 0],[4, 2], [4, 4], [4, 0]])/4.
		x,y,n = X.T[0],X.T[1],6


	(H,c) = est_feature(x,y,n,float(args.hmax))

	print args
	print "** Learning result: "
	print "H = %f (%s) with %d clusters:" % (H, args.f, len(c))
	print c
	# Step 2: Execution: check if value v is true for relevant predicates
	np.random.seed(4)

	v = [ 0., 0.] #np.random.rand(2)
	dmax = 0.9
	v_distance = check_predicate(c,v,dmax)
	print v_distance
	print "v holds true" if v_distance>-1 else "v holds false"

	plt.show()
