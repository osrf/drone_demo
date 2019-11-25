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

## Usage

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
  -f [file], --file [file]
                      Filename with waypoints [x, y, z, heading, timeout,
                      tolerance]
```

There are two different ways to run the `tester_node.py`:

### Argument in the command line

For example, the following command shows how to test the drone with some waypoints, each one contains the X, Y, Z, heading, timeout and tolerance.

```bash
python3 /home/ahcorde/drone_demo_ros2/src/drone_demo/drone_demo_system_tests/src/system/tester_node.py __ns:=/iris_0 -r 4.0 5.0 3.0 1.57 -t 40 -tol 0.4 -r 4.0 -5.0 3.0 1.57 -t 40 -tol 0.4
```

It's possible to repeat `-r`, `-t`, `-tol` to include as many waypoints as you want.

### File

Adding many waypoints using argument could be cumbersome, if you need to add many waypoints maybe a better options it's to use a file with all the waypoints. Using the option `-f` we can select a file with some waypoints. The structure of the file is:

```text
x, y, z, heading, timeout, tolerance
x, y, z, heading, timeout, tolerance
x, y, z, heading, timeout, tolerance
...
```

You can find an example of this file in the waypoints folder.

## Adding your own test

You have to add three things to add your own test.

 - Include you new waypoints file inside the `waypoints` folder.
 - Create a launch file like `test_system_yosemite_launch.py` adding the corresponding arguments to `tester_node.py` node which the waypoints (using arguments or the file).

```python
    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester_node.py'),
             '-f', waypoints_dir + '/waypoints/waypoints.txt',
             '--ros-args', '--remap', '__ns:=/iris_0'],
        name='tester_node',
        output='screen')
```

 - then add a new `ament_add_test` to CMakeLists.txt inside the folder `src/system/`.
   - In the `COMMAND` argument you have to write the name of your new launche file created in the step before
   - Pay attention to `TIMEOUT`, be sure that you have enough time to fly over all you waypoints.

```cmake
 ament_add_test(test_arm_take_off_land
   GENERATE_RESULT_FOR_RETURN_CODE_ZERO
   COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_yosemite_launch.py"
   WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
   TIMEOUT 120
   ENV
     TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
 )
```

## Notes
 * The test normally takes less than a minute to run, with a timeout of 2 minutes

## Future Work
  * Add additional goal poses if the first one successfully passes
