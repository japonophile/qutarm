#!/usr/bin/env python
import rospy
import actionlib
from operator import sub
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

class QutarmController(object):

    def __init__(self, name, action_name):
        self.name = name
        self.action_name = action_name
        #rospy.init_node(self.name, log_level=rospy.DEBUG)
        rospy.init_node(self.name)
        self.action_server = actionlib.SimpleActionServer(self.name+"/"+self.action_name, \
            FollowJointTrajectoryAction, self.execute_cb, auto_start=False)
        self.pub_trajpoint = rospy.Publisher('trajpoint', JointTrajectoryPoint, queue_size=10)
        rospy.Subscriber("joint_states", JointState, self.jointstate_cb)
        self.latestpositions = None
        self.action_server.start()

    def execute_cb(self, goal):
        rospy.loginfo("Received new goal:\n%s"%goal)
        success = True
        start_time = rospy.Time.now()

        i = 1
        previouspoint = None
        for point in goal.trajectory.points:
            
            time_to_wait = start_time + point.time_from_start - rospy.Time.now()
            if time_to_wait > rospy.Duration(0):
                rospy.loginfo("Sleeping for %s seconds..."%time_to_wait.to_sec())
                rospy.sleep(time_to_wait)
            
            # only publish feedback if we have received some position recently
            if previouspoint and self.latestpositions:
                fb = FollowJointTrajectoryFeedback()
                fb.desired = previouspoint
                fb.actual = JointTrajectoryPoint()
                fb.actual.positions = self.latestpositions
                fb.error = JointTrajectoryPoint()
                fb.error.positions = map(sub, fb.desired.positions, fb.actual.positions)
                self.action_server.publish_feedback(fb)
            
            # only use positions and velocities (velocities are always positive)
            point.velocities = map(abs, point.velocities)
            point.accelerations = []
            point.effort = []
            
            rospy.loginfo("Go to point %d:\n%s"%(i, point))
            self.latestpositions = None
            self.pub_trajpoint.publish(point)
            
            previouspoint = point
            i += 1

        if success:
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self.result = FollowJointTrajectoryResult()
            self.result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
    	    self.action_server.set_succeeded(self.result)

    def jointstate_cb(self, jointstate):
        self.latestpositions = jointstate.position


if __name__ == '__main__':
    QutarmController('qutarm_controller', 'follow_joint_trajectory')
    rospy.spin()

