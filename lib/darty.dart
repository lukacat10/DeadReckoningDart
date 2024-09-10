import 'dart:math';
import 'package:darty/external_functions.dart';

int calculate() {
  return 6 * 7;
}

class SensorEvent {
  // The sensor type (gravity, magnetic field, gyroscope, etc.)
  final int sensorType;

  // The timestamp of the event
  final int timestamp;

  // The values provided by the sensor
  final List<double> values;

  SensorEvent({
    required this.sensorType,
    required this.timestamp,
    required this.values,
  });
}

class GyroscopeDeltaOrientation {
  // Previous timestamp for tracking time deltas
  int previousTimestamp = 0;

  // Previous sensor values for tracking orientation changes
  List<double> previousValues = [0.0, 0.0, 0.0];

  // Constructor
  GyroscopeDeltaOrientation(double sensitivity, List<double> bias);

  // Method to calculate the delta orientation based on timestamp and sensor values
  List<double> calcDeltaOrientation(int timestamp, List<double> values) {
    if (previousTimestamp == 0) {
      previousTimestamp = timestamp;
      previousValues = values;
      return [0.0, 0.0, 0.0]; // No delta at first run
    }

    double deltaTime = (timestamp - previousTimestamp) *
        1e-9; // Convert nanoseconds to seconds
    previousTimestamp = timestamp;

    // Calculate delta orientation based on sensor values (simple integration)
    List<double> deltaOrientation = [
      values[0] * deltaTime,
      values[1] * deltaTime,
      values[2] * deltaTime
    ];

    previousValues = values;

    return deltaOrientation;
  }
}

class GyroscopeEulerOrientation {
  // Current orientation state (yaw, pitch, roll)
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;

  // Constructor
  GyroscopeEulerOrientation(List<List<double>> initialOrientation);

  // Method to update and return the current heading (yaw)
  // The delta orientation comes from the gyroscope data
  double getHeading(List<double> deltaOrientation) {
    // Assuming deltaOrientation[0] = pitch, deltaOrientation[1] = roll, deltaOrientation[2] = yaw

    // Update the current orientation by applying the delta orientation
    pitch += deltaOrientation[0];
    roll += deltaOrientation[1];
    yaw += deltaOrientation[2];

    // Normalize yaw to be between 0 and 360 degrees
    yaw = yaw % (2 * pi); // Keep the yaw in the range of 0 to 2*pi (radians)

    if (yaw < 0) {
      yaw += 2 * pi;
    }

    // Convert yaw from radians to degrees (if needed)
    return yaw * (180 / pi);
  }
}

class DynamicStepCounter {
  // Step detection threshold
  final double stepThreshold;

  // Constructor with a default threshold
  DynamicStepCounter({this.stepThreshold = 1.0});

  // Method to check if a step has occurred
  bool findStep(double accelerationNorm) {
    // If the acceleration norm exceeds the threshold, we count it as a step
    if (accelerationNorm > stepThreshold) {
      return true;
    }
    return false;
  }
}

class Sensor {
  // Define sensor types as static constants
  static const int TYPE_GRAVITY = 1;
  static const int TYPE_MAGNETIC_FIELD = 2;
  static const int TYPE_MAGNETIC_FIELD_UNCALIBRATED = 3;
  static const int TYPE_GYROSCOPE = 4;
  static const int TYPE_GYROSCOPE_UNCALIBRATED = 5;
  static const int TYPE_LINEAR_ACCELERATION = 6;
  static const int TYPE_STEP_DETECTOR = 7;

  // You can add more sensor types as needed

  // Sensor name (optional, for descriptive purposes)
  final String name;

  // Sensor type (one of the constants defined above)
  final int type;

  // Constructor
  Sensor({required this.type, this.name = ''});
}

class MagneticFieldOrientation {
  // Static method to calculate heading (yaw) based on gravity and magnetic field data
  static double getHeading(
      List<double> gravity, List<double> magneticField, List<double> magBias) {
    // Apply bias correction to the magnetic field data
    List<double> correctedMag = [
      magneticField[0] - magBias[0],
      magneticField[1] - magBias[1],
      magneticField[2] - magBias[2]
    ];

    // Normalize gravity vector
    double gravityNorm = sqrt(gravity[0] * gravity[0] +
        gravity[1] * gravity[1] +
        gravity[2] * gravity[2]);
    List<double> gravityNormVector =
        gravity.map((value) => value / gravityNorm).toList();

    // Calculate cross product of gravity and magnetic field to get the East vector
    List<double> east = [
      gravityNormVector[1] * correctedMag[2] -
          gravityNormVector[2] * correctedMag[1],
      gravityNormVector[2] * correctedMag[0] -
          gravityNormVector[0] * correctedMag[2],
      gravityNormVector[0] * correctedMag[1] -
          gravityNormVector[1] * correctedMag[0]
    ];

    // Normalize the East vector
    double eastNorm =
        sqrt(east[0] * east[0] + east[1] * east[1] + east[2] * east[2]);
    List<double> eastNormVector =
        east.map((value) => value / eastNorm).toList();

    // Calculate the North vector by crossing East with the gravity vector
    List<double> north = [
      gravityNormVector[1] * eastNormVector[2] -
          gravityNormVector[2] * eastNormVector[1],
      gravityNormVector[2] * eastNormVector[0] -
          gravityNormVector[0] * eastNormVector[2],
      gravityNormVector[0] * eastNormVector[1] -
          gravityNormVector[1] * eastNormVector[0]
    ];

    // Heading is the angle between the North vector and the magnetic field in the horizontal plane
    double heading = atan2(eastNormVector[0], north[0]);

    // Convert heading from radians to degrees
    heading = heading * (180 / pi);

    // Normalize heading to 0-360 degrees
    if (heading < 0) {
      heading += 360;
    }

    return heading;
  }
}

const double GYROSCOPE_INTEGRATION_SENSITIVITY = 0.0025;

bool firstRun = true;
bool isRunning = false;
int startTime = 0;
List<double> currGravity = [];
List<double> currMag = [];
List<double> magBias = [];
double magHeading = 0;
List<double> gyroBias = []; // Comes from global settings (intent)
GyroscopeDeltaOrientation gyroscopeDeltaOrientation =
    GyroscopeDeltaOrientation(GYROSCOPE_INTEGRATION_SENSITIVITY, gyroBias);
GyroscopeEulerOrientation gyroscopeEulerOrientation =
    GyroscopeEulerOrientation(ExtraFunctions.IDENTITY_MATRIX);
double initialHeading = MagneticFieldOrientation.getHeading(currGravity, currMag, magBias);
double gyroHeading = 0;
double strideLength = 0; // Comes from global settings (intent)
DynamicStepCounter dynamicStepCounter;
ScatterPlot scatterPlot;
LinearLayout mLinearLayout;
double weeksGPS;
double secondsGPS;

void onSensorChanged(
    {required SensorEvent event,
    required bool firstRun,
    required bool isRunning,
    required int startTime,
    required List<double> currGravity,
    required List<double> currMag,
    required List<double> magBias,
    required double magHeading,
    required GyroscopeDeltaOrientation gyroscopeDeltaOrientation,
    required GyroscopeEulerOrientation gyroscopeEulerOrientation,
    required double initialHeading,
    required List<double> gyroBias,
    required double gyroHeading,
    required double strideLength,
    required DynamicStepCounter dynamicStepCounter,
    required ScatterPlot scatterPlot,
    required LinearLayout mLinearLayout,
    required double weeksGPS,
    required double secondsGPS}) {
  if (firstRun) {
    startTime = event.timestamp;
    firstRun = false;
  }

  if (event.sensorType == Sensor.TYPE_GRAVITY) {
    currGravity = event.values;
  } else if (event.sensorType == Sensor.TYPE_MAGNETIC_FIELD ||
      event.sensorType == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
    currMag = event.values;
  }

  if (isRunning) {
    if (event.sensorType == Sensor.TYPE_GRAVITY) {
      List<double> dataValues = ExtraFunctions.arrayToList(event.values);
      // Prepare data for writing to file
      dataValues.insert(0, (event.timestamp - startTime).toDouble());
      // File writing logic here
    } else if (event.sensorType == Sensor.TYPE_MAGNETIC_FIELD ||
        event.sensorType == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
      magHeading =
          MagneticFieldOrientation.getHeading(currGravity, currMag, magBias);
      List<double> dataValues = [
        event.values[0],
        event.values[1],
        event.values[2],
        magBias[0],
        magBias[1],
        magBias[2]
      ];
      // Prepare data for writing to file
      dataValues.insert(0, (event.timestamp - startTime).toDouble());
      dataValues.add(magHeading);
      // File writing logic here
    } else if (event.sensorType == Sensor.TYPE_GYROSCOPE ||
        event.sensorType == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {
      List<double> deltaOrientation = gyroscopeDeltaOrientation
          .calcDeltaOrientation(event.timestamp, event.values);
      gyroHeading = gyroscopeEulerOrientation.getHeading(deltaOrientation);
      gyroHeading += initialHeading;
      List<double> dataValues = [
        event.values[0],
        event.values[1],
        event.values[2],
        gyroBias[0],
        gyroBias[1],
        gyroBias[2]
      ];
      // Prepare data for writing to file
      dataValues.insert(0, (event.timestamp - startTime).toDouble());
      dataValues.add(gyroHeading);
      // File writing logic here
    } else if (event.sensorType == Sensor.TYPE_LINEAR_ACCELERATION) {
      double norm = ExtraFunctions.calcNorm(
          [event.values[0] + event.values[1] + event.values[2]]);
      bool stepFound = dynamicStepCounter.findStep(norm);
      if (stepFound) {
        List<double> dataValues = ExtraFunctions.arrayToList(event.values);
        dataValues.insert(0, (event.timestamp - startTime).toDouble());
        dataValues.add(1.0);
        // Complimentary filter logic
        double compHeading =
            ExtraFunctions.calcCompHeading(magHeading, gyroHeading);
        double oPointX = scatterPlot.getLastYPoint();
        double oPointY = -scatterPlot.getLastXPoint();
        oPointX += ExtraFunctions.getXFromPolar(strideLength, gyroHeading);
        oPointY += ExtraFunctions.getYFromPolar(strideLength, gyroHeading);
        double rPointX = -oPointY;
        double rPointY = oPointX;
        scatterPlot.addPoint(rPointX, rPointY);
        mLinearLayout.removeAllViews();
        mLinearLayout.addView(scatterPlot.getGraphView());
      } else {
        List<double> dataValues = ExtraFunctions.arrayToList(event.values);
        dataValues.insert(0, (event.timestamp).toDouble());
        dataValues.add(0.0);
      }
    } else if (event.sensorType == Sensor.TYPE_STEP_DETECTOR) {
      bool stepFound = (event.values[0] == 1);
      if (stepFound) {
        double compHeading =
            ExtraFunctions.calcCompHeading(magHeading, gyroHeading);
        double oPointX = scatterPlot.getLastYPoint();
        double oPointY = -scatterPlot.getLastXPoint();
        oPointX += ExtraFunctions.getXFromPolar(strideLength, gyroHeading);
        oPointY += ExtraFunctions.getYFromPolar(strideLength, gyroHeading);
        double rPointX = -oPointY;
        double rPointY = oPointX;
        scatterPlot.addPoint(rPointX, rPointY);
        mLinearLayout.removeAllViews();
        mLinearLayout.addView(scatterPlot.getGraphView());
      }
    }
  }
}
