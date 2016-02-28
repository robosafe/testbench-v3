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
- Full installation of ROS, Gazebo and MoveIt! (Packages usage, compilation, configuration). Currently this testbench version only works on ROS Hydro and Gazebo 1.9, running in Ubuntu Precise, Quantal or Raring. Support for ROS Indigo will be added soon.
- Compilation of the packages in a Catkin workspace (e.g., with the name catkin_ws), particularly to use the assertion monitors for Gazebo speeds (C++ source files). 
- For the model-based test generation, UPPAAL 4.0.14 (recommended), CoVer 1.4 http://www.hessel.nu/CoVer/. For an UPPAAL (TA or PTA) model (model.xml) and CTL properties saved in model.q, to generate traces (test templates in model-based test generation) from command line, do: 
./cover -t 0 -G -f output_file_name model.xml model.q
This will generate a number of traces (one for each property in the model.q file) with the file name output_file_name-#.xtr. Then, use the the provided Python scripts (in /scripts/testgens) to translate into suitable stimulus files. 
-Python Coverage modules installed via Pip from https://coverage.readthedocs.org/en/coverage-4.0.3/.

----------------RUNNING THE TESTBENCH-----------------------------------
- To be run in ROS Hydro with Gazebo and MoveIt! installed.
- Use the bash scripts (one for each type of stimulus generation: model-based, random, constrained), to run the testing in batch mode with visuals in Gazebo. Reports are generated in the /tmp/ folder. The bash scripts will start ROS (the core), Gazebo, MoveIt and will run all the Python scripts that pull the stimulus, execute the environment and robot codes, and produce reports (assertion coverage, code coverage). 

----------------------LICENSE--------------

The code we developed is covered by GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007


Questions, bugs, comments: dejanira.araizaillan@bristol.ac.uk, david.western@bristol.ac.uk
