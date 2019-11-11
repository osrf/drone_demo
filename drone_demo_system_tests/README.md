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

## Notes
 * The test normally takes less than a minute to run, with a timeout of 2 minutes

## Future Work
  * Add additional goal poses if the first one successfully passes
