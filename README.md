# Advanced DSR Command
Distance sensor resets are used to instantaneously correct a robot's pose along a given axis by taking a distance sensor reading from the robot to the wall and correctly displacing the bot's known pose.

Compared to a traditional DSR, this command:
- Filters outliers using a median of multiple readings
- Automatically chooses the axis to align given the chosen sensor
- Accounts for non-parallel (or orthogonal) correction using basic trigonometry

## Usage
All the user must do is configure the ports and the sensor placement relative to the bot's center, then use this command anywhere:

`distanceSensorCorrection(DistanceSensorId::{SIDE});`

Example:

`distanceSensorCorrection(DistanceSensorId::FRONT);`

The example above resets the pose along the bot's forward axis, which is then mapped onto the coordinate grid.
