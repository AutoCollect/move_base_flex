#!/usr/bin/env python
#
# move_base legacy relay node:
# Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
# Also relays the simple goal topic, make_plan service, and dynamic reconfiguration calls.
#
# Author: Jorge Santos
# License: 3-Clause BSD

import actionlib
import copy

import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped
from move_base.cfg import MoveBaseConfig


# Global variables for planners.
bgp = None    # Active global planner (selected based on dynamic reconfigure)
blp = None    # Base local planner

def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg, planner=bgp, controller=blp))
    rospy.logdebug("Relaying move_base_simple/goal pose to mbf")

def mb_execute_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose, planner=bgp, controller=blp),
                        feedback_cb=mbf_feedback_cb)
    rospy.logdebug("Relaying legacy move_base goal to mbf")
    mbf_mb_ac.wait_for_result()
    status = mbf_mb_ac.get_state()
    result = mbf_mb_ac.get_result()
    rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.MoveBaseResult.SUCCESS:
        mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
    else:
        mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.message)

def make_plan_cb(request):
    mbf_gp_ac.send_goal(mbf_msgs.GetPathGoal(start_pose=request.start, target_pose=request.goal,
                                             use_start_pose=bool(request.start.header.frame_id),
                                             planner=bgp, tolerance=request.tolerance))
    rospy.logdebug("Relaying legacy make_plan service to mbf get_path action server")
    mbf_gp_ac.wait_for_result()
    status = mbf_gp_ac.get_state()
    result = mbf_gp_ac.get_result()
    rospy.logdebug("MBF get_path execution completed with result [%d]: %s", result.outcome, result.message)
    if result.outcome == mbf_msgs.GetPathResult.SUCCESS:
        return nav_srvs.GetPlanResponse(plan=result.path)

def mbf_feedback_cb(feedback):
    mb_as.publish_feedback(mb_msgs.MoveBaseFeedback(base_position=feedback.current_pose))

def mb_reconf_cb(config, level):
    global bgp, blp  # Declare global variables at the very beginning
    rospy.logdebug("Relaying legacy move_base reconfigure request to mbf")
    
    if not hasattr(mb_reconf_cb, "default_config"):
        mb_reconf_cb.default_config = copy.deepcopy(config)
    if config.get('restore_defaults'):
        config = mb_reconf_cb.default_config

    mbf_config = copy.deepcopy(config)

    # Process base_local_planner.
    if 'base_local_planner' in mbf_config:
        blp = mbf_config.pop('base_local_planner')
        rospy.loginfo("Using base_local_planner: %s", blp)

    # Process other non-global parameters.
    for param in ['controller_frequency', 'controller_patience', 'max_controller_retries',
                  'planner_frequency', 'planner_patience']:
        if param in mbf_config:
            mbf_config[param] = mbf_config.pop(param)
    # Remove unsupported parameter max_planning_retries.
    if 'max_planning_retries' in mbf_config:
        rospy.loginfo("Removing unsupported parameter: max_planning_retries")
        mbf_config.pop('max_planning_retries')
    # Process recovery_behavior_enabled mapping.
    if 'recovery_behavior_enabled' in mbf_config:
        mbf_config['recovery_enabled'] = mbf_config.pop('recovery_behavior_enabled')

    # Process all global planner parameters.
    # Look for any key ending with "_planner" except "base_local_planner" and "active_global_planner".
    global_planners = {}
    for key in list(mbf_config.keys()):
        if key.endswith("_planner") and key not in ["base_local_planner", "active_global_planner"]:
            value = mbf_config.pop(key)
            if value.upper() != "NONE":
                global_planners[key] = value
                rospy.logerr("Loaded global planner parameter: %s = %s", key, value)
            else:
                rospy.logerr("Ignoring global planner parameter %s because it is set to NONE", key)
    
    if global_planners:
        rospy.logerr("All loaded global planners: %s", global_planners)
    else:
        rospy.logwarn("No global planner parameters loaded!")

    # Get the active global planner key from the configuration.
    active_key = mbf_config.pop("active_global_planner", "NONE")
    rospy.logerr("Active global planner selection: %s", active_key)
    if active_key.upper() != "NONE" and active_key in global_planners:
        bgp = global_planners[active_key]
        rospy.logerr("Using active global planner: %s = %s", active_key, bgp)
    elif global_planners:
        # Default to the first available global planner.
        active_key, bgp = next(iter(global_planners.items()))
        rospy.logerr("Active global planner: %s = %s", active_key, bgp)
    else:
        bgp = None
        rospy.logwarn("No valid global planner configured!")

    # Remove any unsupported parameters.
    for param in ['conservative_reset_dist', 'clearing_rotation_allowed', 
                  'make_plan_add_unreachable_goal', 'make_plan_clear_costmap']:
        if param in mbf_config:
            mbf_config.pop(param)
    
    rospy.loginfo("Updating MBF dynamic reconfigure with: %s", mbf_config)
    mbf_drc.update_configuration(mbf_config)
    return config

if __name__ == '__main__':
    rospy.init_node("move_base")

    # move_base_flex get_path and move_base action clients
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_gp_ac = actionlib.SimpleActionClient("move_base_flex/get_path", mbf_msgs.GetPathAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))
    mbf_gp_ac.wait_for_server(rospy.Duration(10))

    # move_base_flex dynamic reconfigure client
    mbf_drc = Client("move_base_flex", timeout=10)

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)
    mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
    mb_as.start()

    # move_base make_plan service
    mb_mps = rospy.Service('~make_plan', nav_srvs.GetPlan, make_plan_cb)

    # move_base dynamic reconfigure server
    mb_drs = Server(MoveBaseConfig, mb_reconf_cb)

    rospy.spin()