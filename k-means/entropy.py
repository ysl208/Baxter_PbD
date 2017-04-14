"""
========================================
Entropy for discrete feature recognition
========================================
Calculates relevant features based on the entropy of the values obtained
e.g. block colour: red, yellow

"""

# Author: Ying Siu Liang <yingsiu@liang.at>
# Last update: April 2017

import argparse
import numpy as np
from collections import Counter


def entropy(l, H_max):

	""" 
	Computes entropy of label distribution. 
	Returns entropy and value with maximum frequency
	l : sequence of labels e.g. 'rggrb'
	"""
	# dictionary of unique labels with their counts
	c = Counter(l)
	n_labels = float(len(l))

	# if there is only one value for this feature
	if n_labels <= 1:
		return 0
	
	# dictionary with unique labels with their probabilities
	probs = {x: c[x]/n_labels for x in c}

	print "**** Probabilities for labels:" 
	print c
	print probs

	# Compute standard entropy
	print "Calculating entropy..."
	entropy = -sum([probs[x]*np.log2(probs[x]) for x in probs])
	print entropy

	""" Checks if the feature is relevant """
	if entropy < H_max:
		print "relevant < %f with value =" % H_max
		v_f = max(probs, key=probs.get)
		print v_f
		return (entropy,v_f)
	else:
		print "not relevant > %f" % H_max
		return (-1,None)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-f', help='feature', default='colour')
	parser.add_argument('-hmax', help='maximal accepted variance', default = 0.1)
	parser.add_argument('-l', help='labels', default='112')

	args = parser.parse_args()
	print args
	(H,v_f) = entropy(args.l, float(args.hmax))
	print "*** Result: "
	print "H = %f (%s = %s)" % (H, args.f, v_f)

	print "H is relevant" if H > -1 else "H is not relevant"

