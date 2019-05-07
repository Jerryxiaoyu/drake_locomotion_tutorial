
##[LCM](https://lcm-proj.github.io/tutorial_general.html) Tutorial

### Use
* define the messages in a file `robot_states_t.lcm` as follows :
```buildoutcfg
package lcmt;

struct robot_state_t {
  // The timestamp in milliseconds.
  int64_t timestamp;
  int32_t num_joints;

  // The following variable defines which robot each joint is
  // associated with. The range of the values is [0, num_robots).
  double joint_position[num_joints]; //q
  double joint_velocity[num_joints]; //v
}
```

* generate the Python interface using `lcm-gen -p robot_states_t.lcm`
