# testbench
CDV simulator-based testbench with test templates

----------------CONTENTS------------------------------------------------
- bert2_moveit ROS package --> configuration for the path planning
- bert2_simulator ROS package --> simulator and testbench 
- simulator_ *.sh --> bash scripts for the testing in batch mode
- Inside the bert2_simulator package --> BERT2 robot model, human model and object model, for Gazebo
- Inside bert2_simulator/scripts --> simulator nodes, assertion monitor nodes, coverage collector module, "stimulus" (test templates with high-level human actions) for pseudorandom, constrained and model-based test generation in requirements and cross-product coverage
- Inside bert2_simulator/scripts/testgens --> test generator modules, UPPAAL PTA model (6 PTA automata), CTL properties for model checking and model-based test generation
- Inside bert2_simulator/scripts/mbtg_xprod --> "stimulus" (test templates with high-level human actions) for model-based test generation
 in cross-product and code coverage
 - Example_test_reports_mbtg_xproduct folder --> example of generated assertion monitor reports, coverage reports and test run reports, for a model-based test template (stimulus_legiblembga-1 or GPL = ~1,~1,1) in the cross-product coverage experiments. 


----------------BEFORE USE----------------------------------------------
Assumptions:
- Full installation and knowledge of ROS, Gazebo and MoveIt! (Packages usage, compilation, configuration).
- Compilation of the packages in a Catkin workspace.
- UPPAAL knowledge (to use model checking and model-based test generation).

----------------RUNNING THE TESTBENCH-----------------------------------
- To be run in ROS Indigo with Gazebo and MoveIt! installed.
- Use the bash scripts, to run the testing in batch mode with visuals in Gazebo. Reports are generated in the /tmp/ folder.


Questions, bugs, comments: dejanira.araizaillan@bristol.ac.uk, david.western@bristol.ac.uk
