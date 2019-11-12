# System Tests

This is a 'top level' system test which will use Gazebo to simulate a drone arming, taking off, moving from an known initial starting position to a goal pose, returning to launch and landing.

## To run the test
First, you must build `drone_demo` including this package:

```
colcon build --merge-install
```

Then you can run all the system tests:

```
colcon test --packages-select drone_demo_system_tests
```

## Running the test

Usage:

```bash
usage: tester_node.py [-h] [-r x y z heading] [-tol tolerance_position]
                      [-t time_out]

System-level drone demo tester node

optional arguments:
  -h, --help            show this help message and exit
  -r x y z heading, --robot x y z heading
                        The robot final position.
  -tol tolerance_position, --tolerance_position tolerance_position
                        Tolerance position.
  -t time_out, --timeout time_out
```

For example, the following command shows how to test the drone with waypoints, each one contains the X, Y, Z, heading, timeout and tolerance.

```bash
python3 /home/ahcorde/drone_demo_ros2/src/drone_demo/drone_demo_system_tests/src/system/tester_node.py __ns:=/iris_0 -r 4.0 5.0 3.0 1.57 -t 40 -tol 0.4 -r 4.0 -5.0 3.0 1.57 -t 40 -tol 0.4
```

## Notes
 * The test normally takes less than a minute to run, with a timeout of 2 minutes

## Future Work
  * Add additional goal poses if the first one successfully passes
