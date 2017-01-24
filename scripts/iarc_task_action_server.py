#!/usr/bin/env python
import threading
import rospy
import actionlib
from takeoff_task import TakeoffTask
from iarc7_motion.msg import QuadMoveAction

class IarcTaskActionServer:
    def __init__(self):
        self._action_name = "motion_planner_server"


        self.action_server = actionlib.ActionServer(self._action_name,
                                          QuadMoveAction,
                                          self.new_goal,
                                          cancel_cb=self.cancel_request,
                                          auto_start = False)
        self.goal_tasks = []
        self.current_task = None
        self.current_goal = None
        self.cancel_requested = False
        self.lock = threading.RLock()
        # Start action server last to avoid race condition
        self.action_server.start()

    # Private method
    def new_goal(self, goal):
        with self.lock:
            rospy.loginfo("new_goal: %s", goal.get_goal_id().id)

            task_request = goal.get_goal()

            if task_request.movement_type == "takeoff" :
                new_task = TakeoffTask()
            else:
                rospy.logerror("Goal has invalid movement_type: %s", movement_type)
                goal.set_rejected()
                return

            # Support simple queue destroying preempting for now
            if task_request.preempt :
                if len(self.goal_tasks) > 0:
                    for goal, _ in  goal_tasks:
                        goal.set_cancel_requested()
                        goal.set_canceled()
                    self.goal_tasks = []
                if self.current_goal:
                    self.cancel_requested = True
                    self.current_goal.set_cancel_requested()

            self.goal_tasks.append([goal, new_task])

    # Private method
    def cancel_request(self, cancel):
        with self.lock:
            rospy.logdebug("cancel_request")

            if cancel == self.current_goal :
                rospy.logdebug("Cancel requested on current goal")
                self.current_goal.set_cancel_requested()
                self.cancel_requested = True
                return

            for goal, _ in self.goal_tasks:
                if goal == cancel:
                    rospy.logdebug("Cancel requested on queued goal")
                    goal.set_cancel_requested()
                    goal.set_canceled()
                    return

            rospy.logerror("Attempt to cancel goal but goal did not exist")

    # Function for task runner to use
    def set_succeeded(self, result=None, text=""):
        with self.lock:
            if self.current_goal:
                rospy.logdebug("Current task succeeded")
                self.current_goal.set_succeeded()
            else:
                rospy.logdebug("There was not task to succeed")

            self.current_task = None
            self.current_goal = None
            self.cancel_requested = False

    def set_aborted(self):
        with self.lock:
            if self.current_goal:
                rospy.logdebug("Current task aborted")
                self.current_goal.set_aborted()
            else:
                rospy.logdebug("There was not task to abort")

            self.current_task = None
            self.current_goal = None
            self.cancel_requested = False

    def set_canceled():
        with self.lock:
            if self.current_goal :
                rospy.logdebug("Current task cancelled")
                self.current_goal.set_canceled()
            else:
                rospy.logdebug("There was not a task to cancel")

            self.current_task = None
            self.current_goal = None
            self.cancel_requested = False

    def is_canceled(self):
        with self.lock:
            return self.cancel_requested

    def get_new_task(self):
        with self.lock:
            if len(self.goal_tasks) == 0:
                return None
            
            self.current_goal, self.current_task= self.goal_tasks.pop(0)
            self.cancel_requested = False

            rospy.logdebug("New task accepted")
            self.current_goal.set_accepted()

            return self.current_task

    def has_new_task(self):
        with self.lock:
            return (len(self.goal_tasks) > 0)
