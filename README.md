## Quick guide

***

### ROS enabled environment

***

#### KB_builder.py
Run this script to create a KB based on the current running ROS architecture. Expects an cvs file (provided) and output an n3 file.
The repository provides a reasonably popilated and already built KB, but `KB_builder.py` should be run at the beginning when working with a new system.

#### server.py
Starts a server that interact with the ROS architecture. Provides a list of remote API to analyze and interact with the system.

- `/capabilities`: provides the list of capabilities of the system with their associated topic
- `/execute`: receives a list of instructions in JSON format and execute them
- `/run`: receive a single command in JSON format and execute it
- `/read`: receive a list of topics in JSON format and provides the newst message from that topic

***

### General environment

***
#### onto_rob_ros_interface.py
Contains the `OntoRobRosInterface` class, it sets up a connection with the server created by `server.py`. The class provides various method to interact with the ROS system:

- `get_topic_list()` Provides the list of available topics
- `get_capability_list()` Lists all the capabilities avaliable
- `topic_in_capability(topic, capability)` Tells if a topic evokes a specific capability
- `get_message_fields(topic, capability)` Provides the list of fields of a specific message on a specific topic for a specific capability
- `get_message_type(topic)` Provides the type of the message on a specific topic
- `publish_on_topic(topic, capability, message)` Publish a message on a topic associated with a specific capability. The message is a dictionary with the same structure provided by `get_message_fields(topic, capability)`
- `read_from_topic(topic, capability)`Provides the newest message received on the specified topic.


***

## Examples

***
####

#### onto_rob_ros_interface.py

```python
>>> from onto_rob_ros_interface import OntoRobRosInterface
>>> intr = OntoRobRosInterface()

>>> intr.get_topic_list()
['/turtle1/pose', '/turtle1/cmd_vel']

>>> intr.get_capability_list()
['Robot_position', 'Directional_Movement', 'Robot_speed']

>>> intr.topic_in_capability('/turtle1/pose', 'Robot_speed')
True
>>> intr.topic_in_capability('/turtle1/pose', 'Directional_Movement')
False


>>> intr.get_message_fields('/turtle1/pose', 'Robot_position')
['x', 'y', 'theta']

>>> intr.get_message_fields('/turtle1/pose', 'Robot_speed')
['angular_velocity', 'linear_velocity']

>>> intr.get_message_type('/turtle1/pose')
('turtlesim', 'Pose')
>>> intr.get_message_type('/turtle1/cmd_vel')
('geometry_msgs', 'Twist')

>>> intr.read_from_topic('/turtle1/pose', 'Robot_position')
[{u'capability': u'Robot_position', u'topic': u'/turtle1/pose', u'parameters': {u'y': 5.544444561004639, u'theta': 0.0, u'linear_velocity': 0.0, u'x': 5.544444561004639, u'angular_velocity': 0.0}}]

>>> intr.get_message_fields('/turtle1/cmd_vel', 'Directional_Movement')
['linear.z', 'angular.z', 'angular.x', 'angular.y', 'linear.x', 'linear.y']
>>> msg = {'linear.z', 'angular.z', 'angular.x', 'angular.y', 'linear.x', 'linear.y'}
>>> intr.publish_on_topic('/turtle1/cmd_vel', 'Directional_Movement', msg)
>>> empty_msg = {}
>>> intr.publish_on_topic('/turtle1/cmd_vel', 'Directional_Movement', empty_msg)

>>> intr.read_from_topic('/turtle1/pose', 'Robot_position')
[{u'capability': u'Robot_position', u'topic': u'/turtle1/pose', u'parameters': {u'y': 5.796990871429443, u'theta': 0.5040000081062317, u'linear_velocity': 0.0, u'x': 6.509308815002441, u'angular_velocity': 0.0}}]
```
