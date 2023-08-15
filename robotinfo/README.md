# Robot definition files and controllers

These examples show how to write robot definition files and controllers in a format compatible with Klampt control and grasp planning utilities. In the future we plan to include  navigation planning utilities as well. If you have a robot that is currently unsupported by Klampt, you will need to write your own RobotInfo files and/or controllers in order for Klampt to understand its structure.

## Writing RobotInfo files

The [RobotInfo](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/klampt.model.robotinfo.html#module-klampt.model.robotinfo) file format is a simple JSON structure.

A minimal RobotInfo file for a fixed-base simulated robot looks like the following:
 
```
{
    "name":"MyRobot",
    "modelFile":"data/myrobot.urdf",
    "baseType":"fixed",
    "controllerFile":"klampt.control.simrobotinterface",
	"properties" : {
        "description": "My cool robot"
    }
}
```

It points to a URDF or .rob model, describes the base time, and refers to a controller. In this case, the `controllerFile` is set to a default module that sets up a kinematic simulation that emulates a connection to a real robot controller.

Besides references to the model and controller, a RobotInfo file can also contain:
- end effector information so that a controller can automatically be configured to support Cartesian control.
- resources for named configurations and keypoints on the robot.
- grippers that support the (experimental) Klampt gripper modeling API.
- any other properties that you might want to check in your application logic.

## Writing and testing controllers

Controllers will adhere to the [Robot Interface Layer](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/Manual-Control.html) format, which specifies a Python script with a function `make(robotModel)` that returns a subclass of `RobotInterfaceBase`.

With this `make` function properly defined, you should run `klampt_control ROBOTINFOFILE.json` or `klampt_control CONTROLLER.py MODEL.urdf` to debug your robot's controller. You will be able to send position commands, velocity commands, Cartesian commands, etc. through this interface.


## Directory format

We recommend that you follow a consistent directory format similar to the one given below, particularly if you are interested in contributing to the robots supported by Klampt.  We suggest putting all of a manufacturer's robots into one directory, since many of the models and driver code will be shared amongst robot variants.  The RobotInfo file for a given model variant will refer to items in the subdirectories of the manufacturer's folder.

- Manufacturer/
    - README.md: describe the robot variants supported, dependency installation, etc.
    - myrobot_variant1.json: RobotInfo file for variant 1.
    - myrobot_variant2.json: RobotInfo file for variant 2, e.g., a different gripper.
    - controller/: contains 
        - myrobot_controller_var1.py: Robot Interface Layer compatible controller script, with `make()` function.
        - myrobot_controller_var2.py: Same.
		- other supporting files: Other driver code provided by robot manufacturer
	- resources/ : Klampt resources such as configurations, keypoints, and transforms that can be referred to by name.
	    - variant1/ : Resources for variant 1
		    - home.config
			- packed.config
			- vertical.config
		- variant2/ : Resources for variant 2
		    - home.config
			- packed.config
			- vertical.config
	- simulation/ : Any emulators for the simulation of this robot (see mirobot/simulation)
	    - robot_emulator.py
	- data/: Other data, like URDF and model files, which may be useful
	    - myrobot_var1.urdf
		- myrobot_var2.urdf
		- stl/
		    - part1.stl
			- part2.stl

For this example, `myrobot_variant1.json` might look something like this:

```
{
    "name":"MyRobot",
    "modelFile":"data/myrobot_var1.urdf",
    "baseType":"fixed",
    "controllerFile":controller/myrobot_controller_var1.py",
	"resources" : "resources/variant1",
	"simulatorFile":"simulation/robot_emulator.py",
	"properties" : {
        "description": "Variant 1 of my cool robot, with parallel gripper version A"
		"variant": "1"
    }
}
```