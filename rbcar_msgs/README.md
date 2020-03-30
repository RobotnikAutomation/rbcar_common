This package contains message definitions for integration with the Robotnik Car platform.

Communication within Robotnik Car platform is done through Kafka, using data types defined as Avro schemas.

Avro schemas are in the avro folder, which is a copy of the following repository: https://steps.everis.com/git/ROBORDPROY/avroschemas.git

Avro schemas are converted to ROS messages using a conversion tool: https://github.com/RobotnikAutomation/ros-kafka-connector (branch: develop)

Communication with Kafka is done through a ROS-Kafka bridge: https://github.com/RobotnikAutomation/ros-kafka-connector (branch: develop)
