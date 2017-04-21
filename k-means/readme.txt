======================================================
Feature recognition for continuous and discrete values
======================================================
# Author: Ying Siu Liang <yingsiu@liang.at>
# Last update: April 2017
# Path /ros_ws/src/git/baxter_tasker/k-means
#======================================================

kmeans.py :
 - runs k-means for n randomly generated 2-dimensional vectors
 - uses sklearn: sudo apt-get install python-sklearn  
 - optional parameters: 
  -f <feature name> : default 'position'
  -n <number of feature values> : if not specified uses:
     np.array([[0, 2], [0, 4], [0, 0],[4, 2], [4, 4], [4, 0]])/4.
  -hmax <variance threshold> : default 0.1

python src/git/baxter_tasker/k-means/kmeans.py

#======================================================

entropy.py :
 - calculates entropy for a string of single character elements (e.g. 'rrg')

 - optional parameters: 
  -f <feature name> : default 'colour'
  -l <string of feature values> : default '112'
  -hmax <entropy threshold> : default 0.1

python src/git/baxter_tasker/k-means/entropy.py


