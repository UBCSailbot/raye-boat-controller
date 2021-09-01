# Boat Controller

[![CircleCI](https://circleci.com/gh/UBCSailbot/boat_controller.svg?style=svg)](https://circleci.com/gh/UBCSailbot/boat_controller)

The purpose of the boat controller is to get the boat from point A to point B on a local scale (~1km). We achieve this by adjusting sail and rudder angles based. 

The main controller code can be found in the `python` directory. The boat_controller_node subscribes to the `/sensors` topic and `/desired_heading_degrees` topic, and publishes to the `/rudder_winch_actuation_angle` topic. See [this](https://ubcsailbot.atlassian.net/wiki/spaces/ADA2/pages/1195147292/ROS+Topic+Names) confluence page for more details.

## How to run

1. Install ROS Melodic on a Ubuntu 18.04 (or similar). http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

2. Find the location that you want to create a ROS workspace (eg. `cd ~`)

3. Type the commands `mkdir -p catkin_ws/src` `cd catkin_ws` `catkin_make`.

4. Clone this repository in the src folder: `cd src` `git clone https://github.com/UBCSailbot/boat_controller.git`. 

5. Clone the sailbot-msgs repository in the src folder: `git clone https://github.com/UBCSailbot/sailbot-msg.git`.

6. Go back to catkin\_ws and build and source. `cd ..` `catkin_make` `source devel/setup.bash`.

### Running boat_controller main loop

The easiest way to run boat_controller main loop is to first `source ~/catkin_ws/devel/setup.bash` before running other commands (you can put this in your `~/.bashrc` file as well to do this automatically). Then `rosrun boat_controller boat_controller_node.py`
​

### Running tests

To run the tests, navigate to the `catkin_ws` and run `catkin_make run_tests`. This command runs the tests associated with each package in your catkin workspace (i.e. if you have packages other than this one, the tests for those packages will run as well)

## Contributing
Submit your additional code as a pull request.  Include tests and add the relevant Jira task link in the commit's description. Use the `unittest` framework with rostest wrapper, and register your test to either [integration.test](test/integration.test) or [unit.test](test/unit.test) as applicable. See existing files in `test` folder for examples.

To ensure that the codebase stays clean, we will be using [flake8](https://flake8.pycqa.org/en/latest/) to enforce our style guide, which is mostly based off of [pep8](https://www.python.org/dev/peps/pep-0008/). To automate most of this process, you can use [autopep8](https://github.com/hhatto/autopep8), which is a tool that automatically resolves most style issues. Example usage is shown below. Our team has agreed upon a 120 character line limit.

`pip install flake8`

`pip install --upgrade autopep8`

`autopep8 --in-place --aggressive --max-line-length 120 --aggressive <path_to_file>`

`flake8 --statistics --max-line-length 120 <path_to_file>`