# Publication metadata is not evidence that the chassis applied this command.
# command_id is scoped to one configured lifecycle; published_stamp disambiguates it.
uint64 command_id
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time computed_stamp
builtin_interfaces/Time published_stamp
builtin_interfaces/Time valid_until
swerve_gazebo_sim/ChassisCommand command
