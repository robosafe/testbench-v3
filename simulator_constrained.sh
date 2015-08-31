#!/bin/bash

# To run this script, open a terminal, navigate to the testbench folder
# (created when you cloned this git), and type "./simulator_constrained.sh".
#
# Code coverage results will be summarised in your/path/testbench/stats.txt
# and detailed in various files in your/path/testbench/covhtml/.
# Assertion monitor results will be saved in text files in your/path/testbench/.

(roscore & echo $! >> /tmp/mainpids) &
(sleep 5; roslaunch bert2_simulator bert2_gazebo.launch & echo $! >> /tmp/mainpids) &
(sleep 10; roslaunch bert2_moveit move_group.launch & echo $! >> /tmp/mainpids) &
sleep 5
COUNTER=1
while [ $COUNTER -lt 101 ]; do
        echo "** test_counter="$COUNTER
	sleep 10
	rm -f /tmp/rospids

        # Remove old coverage file so that we only check fresh ones: 
        rm -f $PWD$"/covhtml/bert2_simulator_scripts_robot_py.html"

	(rosrun bert2_simulator object.py & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator pressure.py & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator sensors.py >> /tmp/sensorsout10$COUNTER & echo $! >> /tmp/rospids) &
        (rosrun bert2_simulator contacts_bouncer & echo $! >> /tmp/rospids) &
        (rosrun bert2_simulator trigger_bouncer & echo $! >> /tmp/rospids) &
        (rosrun bert2_simulator assertion_monitor_manager.py myAMs.txt 10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor2.py 10$COUNTER>> /tmp/monitor2out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor3.py 10$COUNTER>> /tmp/monitor3out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor4.py 10$COUNTER>> /tmp/monitor4out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor5.py 10$COUNTER>> /tmp/monitor5out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor6.py 10$COUNTER>> /tmp/monitor6out10$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor_collision_speed.py $COUNTER>> /tmp/monitor_collision_speed_outr$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor_collision_speed_human.py $COUNTER>> /tmp/monitor_collision_speed_human_outr$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator monitor_collision_speed_self.py $COUNTER>> /tmp/monitor_collision_speed_self_outr$COUNTER & echo $! >> /tmp/rospids) &
	(rosrun bert2_simulator human.py stimulus_10$COUNTER >> /tmp/humanout10$COUNTER) &
	(rosrun bert2_simulator robot.py >> /tmp/robotout10$COUNTER)
        echo "The robot has finished. Sleeping to allow monitors to check late events."
	sleep 10
        echo "Reawoken.  Checking coverage."
        python $PWD$"/bert2_simulator/scripts/check_code_coverage.py"
        echo "Coverage checking complete.  Killing anything that's left."
	cat /tmp/rospids | xargs kill
	
        let COUNTER=COUNTER+1
done

cat /tmp/mainpids | xargs kill
