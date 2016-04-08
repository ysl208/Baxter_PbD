#!/usr/bin/env python


########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.
#
#############################################################################
# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##############################################################################


"""
Baxter RSDK Joint Trajectory Action Server

Main changes from original Joint Trajectory Action Server:
    _goal_error  , _error_threshold and _velocity_error are obtained from the trajectory action message
    fixed check for _error_threshold  (>0 and abs)
    added check for _velocity_error 
    fixed double check of _goal_error at end of trajectory (sometimes trajectory would finish properly but the second check would make it fail)
    uses last trajectory point for set_joint_position when stopping properly
    (custom part) PID values are adjusted during trajectory execution: P=4 for 80% of trajectory (move fast), P=1 for remaining 20% (slow down to avoid oscillations), P=2 for the remaining time (big enough to reach goal)
"""

import bisect
from copy import deepcopy
import math
import operator

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from std_msgs.msg import (
    UInt16,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_control
import baxter_dataflow
import baxter_interface


class JointTrajectoryActionServerHR(object):
    def __init__(self, limb, rate=200.0):
        rospy.loginfo("Starting custom trajectory server")
        self.limb=limb
        
        self._ns = 'robot/limb/' + limb + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()
        self._limb = baxter_interface.Limb(limb)

        # Action Feedback/Result
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Controller parameters from arguments and dynamic reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': dict(), 'ki': dict(), 'kd': dict()}
        self._goal_time = 0.0
        self._goal_error = dict()
        self._velocity_error = dict()
        self._error_threshold = dict()
        self._dflt_vel = dict()
        
        self.max_kp=4
        self.min_kp=1
        self.final_kp=2
         
        

        # Create our PID controllers
        self._pid = dict()
        for joint in self._limb.joint_names():
            self._pid[joint] = baxter_control.PID()

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate', UInt16,queue_size=1)
        while not rospy.is_shutdown() and self._pub_rate.get_num_connections()  < 1:
            rospy.sleep(0.01)
        self._pub_rate.publish(self._control_rate)


    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_velocity(self, joint_names):
        return [self._limb.joint_velocity(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        #print "error" , error
        #print "zip" ,  zip(joint_names,error)
        return zip(joint_names, error)

        
    def _get_current_residual_velocity(self, joint_names):
        current = self._get_current_velocity(joint_names)
        error = map(operator.abs, current) 
        return zip(joint_names, error)

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position(jnt_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                     )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_stop(self, joint_names,joint_values=None):
        if joint_values is None:
            velocities = [0.0] * len(joint_names)
            cmd = dict(zip(joint_names, velocities))
            self._limb.set_joint_velocities(cmd)
            self._limb.set_joint_positions(self._limb.joint_angles())
        else:
            self._limb.set_joint_positions( dict(zip(joint_names, joint_values)))

    def _command_velocities(self, joint_names, positions):
        velocities = []
        if self._server.is_preempt_requested():
            self._command_stop(joint_names)
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            return False
        deltas = self._get_current_error(joint_names, positions)


        for delta in deltas:
            threshold=self._error_threshold[delta[0]]
            if (abs(delta[1]) > threshold and threshold > 0.0):
                self._command_stop(joint_names)
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                return False
            velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        cmd = dict(zip(joint_names, velocities))

        self._limb.set_joint_velocities(cmd)
        return True

        
    def _on_trajectory_action(self, goal):        
        """ Main function that performs trajectory"""
        #~ print "goal", goal
        id="Trajectory"
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points

        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))

        # Load parameters for trajectory
        #~ print "goal_tolerance", goal.goal_tolerance
        for i in range(0,len(goal.goal_tolerance)):            
            jnt=goal.goal_tolerance[i].name
            #~ print "jnt" , jnt
            #~ print " goal_tolerance" , goal.goal_tolerance[i].position 
            self._goal_error[jnt] = goal.goal_tolerance[i].position
            self._velocity_error[jnt] = goal.goal_tolerance[i].velocity

        for j in joint_names:
            self._error_threshold[j]=-1
        for i in range(0,len(goal.path_tolerance)):
            jnt=goal.path_tolerance[i].name
            self._error_threshold[jnt] = goal.path_tolerance[i].position
            
            
            


        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)

        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return

        # If all time_from_start are zero,
        # interject these based on default velocities
        last = JointTrajectoryPoint()
        if all(pt.time_from_start.to_sec() == 0.0 for pt in trajectory_points):
            last.positions = self._get_current_position(joint_names)
            move_time = 0.0
            for point in trajectory_points:
                diffs = map(operator.sub, point.positions,
                            last.positions)
                diffs = map(operator.abs, diffs)
                dflt_vel = [self._dflt_vel[jnt] for jnt in joint_names]
                move_time = move_time + max(map(operator.div, diffs, dflt_vel))
                point.time_from_start = rospy.Duration(move_time)
                last.positions = point.positions

        def interp(a, b, pct):
            return a + (b - a) * pct

        def interp_positions(p1, p2, pct):
            return map(interp, p1.positions, p2.positions, [pct] *
                       len(p1.positions))

        end_time = trajectory_points[-1].time_from_start.to_sec()
        control_rate = rospy.Rate(self._control_rate)

        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]

        # Reset feedback/result
        start_point = JointTrajectoryPoint()
        start_point.positions = self._get_current_position(joint_names)
        self._update_feedback(deepcopy(start_point), joint_names,
                              rospy.get_time())

        # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()
        baxter_dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float('inf')
        )

        start_header=goal.trajectory.header.stamp.to_sec()
        now=rospy.get_time()
        #~ print "now=%f start_header=%f end_time=%f goal_time=%f time_tolerance=%f"%(now,start_header,end_time,self._goal_time,goal.goal_time_tolerance.to_sec())
        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        now_from_start = rospy.get_time() - start_time
        while now_from_start < end_time + (1.0 / self._control_rate):
            idx = bisect.bisect(pnt_times, now_from_start)

            if idx == 0:
                # If our current time is before the first specified point
                # in the trajectory, then we should interpolate between
                # our start position and that point.
                p1 = deepcopy(start_point)
            else:
                p1 = deepcopy(trajectory_points[idx - 1])

            if idx != num_points:
                p2 = trajectory_points[idx]
                pct = ((now_from_start - p1.time_from_start.to_sec()) /
                       (p2.time_from_start - p1.time_from_start).to_sec())
                point = interp_positions(p1, p2, pct)
                p1.positions = point
            else:
                # If the current time is after the last trajectory point,
                # just hold that position.
                point = p1.positions

            # Update PID: 80% of trajectory uses max_kp, last 20% use min_kp to slow down
            pct_trajectory=now_from_start/end_time
            if pct_trajectory<0.8:
                pct_trajectory=0
            kp=interp(self.max_kp,self.min_kp,pct_trajectory)
            for jnt in joint_names:                
                self._pid[jnt].set_kp(kp)

            
            # Perform velocity control
            if not self._command_velocities(joint_names, point):
                return

            control_rate.sleep()
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(p1), joint_names, now_from_start)

        # Main trajectory over
        delay=rospy.get_time()-start_time
        rospy.loginfo("%s: Finished main trajectory after %f s"%(self._action_name,delay))

                
        
        # Keep trying to meet goal until goal_time constraint expired
        
        last = JointTrajectoryPoint()
        last.positions = trajectory_points[-1].positions
        last_time = trajectory_points[-1].time_from_start.to_sec()

        def check_goal_state():
            """Check if goal state is reached """
            #~ print "any of goal state is:",any(self._goal_error)
            if any(self._goal_error) is False:
                return None
            for error in self._get_current_error(joint_names, last.positions):
                    if (self._goal_error[error[0]] > 0 and self._goal_error[error[0]] < math.fabs(error[1])): # If position is outside of tolerance
                        #print "Position Error",error[0],":",error[1]
                        self.error = error[1]
                        return error[0]
            else:
                return None

        def check_goal_velocity_state():
            """Check if goal velocity is reached """
            #~ print "any of goal velocity is:",any(self._velocity_error)
            if any(self._velocity_error) is False:
                return None
            for error in self._get_current_residual_velocity(joint_names):
                if (self._velocity_error[error[0]] > 0
                    and self._velocity_error[error[0]] < math.fabs(error[1])): # If velocity is outside of tolerance
                    #print "Velocity Error",error[0],":",error[1]
                    return error[0]
            else:
                return None


        # Set final PID value
        for jnt in joint_names:
                self._pid[jnt].set_kp(self.final_kp)
        
        # Loop until goal reached or goal_tolerance delay expired
        self.error=None
        while not rospy.is_shutdown():
            # Goal reached
            if check_goal_state() is None and check_goal_velocity_state() is None:
                delay=rospy.get_time()-start_time
                rospy.loginfo("%s Successfully finished complete trajectory after %f s"%(id,delay))
                self._command_stop(goal.trajectory.joint_names,last.positions)
                self._result.error_code = self._result.SUCCESSFUL
                self._server.set_succeeded(self._result)
                return
                    

            # Perform velocity control to target
            if not self._command_velocities(joint_names, last.positions):
                rospy.logerr("%s: Exiting after failed command_velocities"%self._action_name)
                return
            
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
            #~ print start_time + last_time + self._goal_time+goal.goal_time_tolerance.to_sec()
            #~ print rospy.get_time()
            if rospy.get_time() > start_time + last_time + self._goal_time+goal.goal_time_tolerance.to_sec():
                #~ print "breaking refinement loop"
                break
            control_rate.sleep()
        # Timed out
        delay=rospy.get_time()-start_time
        rospy.logerr("%s Failed to complete trajectory after %f s"%(self._action_name,delay))
        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
        # Verify goal constraint
        result = check_goal_state()
        
        self._command_stop(goal.trajectory.joint_names,last.positions)

        rospy.logerr("%s: Timeout, Exceeded Goal Threshold Error %s" %
                     (self._action_name, result,))
        rospy.logerr("Error: %f"%(self.error))
        self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
        self._server.set_aborted(self._result)
