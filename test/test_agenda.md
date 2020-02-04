# Unit Testing
For each basic functional unit there should be a set of tests using pytest.
Functions using I/O can be tested by using mock-ups.

# Integration Testing

## Coverage
Full coverage needs to be achieved in order to test all the functionality.

## Mutation Testing
Once a set of passing tests is available for a functional unit, mutation is applied using mutmut.
If no coverage is reached at this point, mutmut may be modified to use coverage.

# System Testing
System testing is carried out manually using the robot. A set of standard application tasks needs to be defined.


