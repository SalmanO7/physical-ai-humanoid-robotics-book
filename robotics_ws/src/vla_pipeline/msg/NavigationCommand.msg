# NavigationCommand.msg
# Defines a navigation command for the robot

# The text of the navigation command (e.g., "navigate to kitchen", "move to target")
string command_text

# The interpreted high-level intent and parameters extracted from the command
string semantic_meaning

# Timestamp of when the command was created
builtin_interfaces/Time timestamp

# Source of the command (e.g., user, autonomous planner)
string source