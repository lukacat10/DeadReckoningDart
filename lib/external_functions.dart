import 'dart:math';

class ExtraFunctions {
  // Private constructor to prevent instantiation
  ExtraFunctions._();

  static const String PREFS_NAME = "Inertial Navigation Preferences";

  static const List<List<double>> IDENTITY_MATRIX = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
  ];

  // Calculate x coordinate from polar coordinates
  static double getXFromPolar(double radius, double angle) {
    return radius * cos(angle);
  }

  // Calculate y coordinate from polar coordinates
  static double getYFromPolar(double radius, double angle) {
    return radius * sin(angle);
  }

  static double nsToSec(double time) {
    return time / 1000000000.0;
  }

  static int factorial(int num) {
    int factorial = 1;
    for (int i = 1; i <= num; i++) {
      factorial *= i;
    }
    return factorial;
  }

  // Multiply two matrices
  static List<List<double>> multiplyMatrices(
      List<List<double>> a, List<List<double>> b) {
    int numRows = a.length;
    int numCols = b[0].length;
    int numElements = b.length;

    List<List<double>> c = List.generate(
        numRows, (_) => List.generate(numCols, (_) => 0.0));

    for (int row = 0; row < numRows; row++) {
      for (int col = 0; col < numCols; col++) {
        for (int element = 0; element < numElements; element++) {
          c[row][col] += a[row][element] * b[element][col];
        }
      }
    }
    return c;
  }

  // Add two matrices
  static List<List<double>> addMatrices(
      List<List<double>> a, List<List<double>> b) {
    int numRows = a.length;
    int numCols = a[0].length;

    List<List<double>> c = List.generate(
        numRows, (_) => List.generate(numCols, (_) => 0.0));

    for (int row = 0; row < numRows; row++) {
      for (int col = 0; col < numCols; col++) {
        c[row][col] = a[row][col] + b[row][col];
      }
    }
    return c;
  }

  // Scale a matrix by a scalar
  static List<List<double>> scaleMatrix(
      List<List<double>> a, double scalar) {
    int numRows = a.length;
    int numCols = a[0].length;

    List<List<double>> b = List.generate(
        numRows, (_) => List.generate(numCols, (_) => 0.0));

    for (int row = 0; row < numRows; row++) {
      for (int col = 0; col < numCols; col++) {
        b[row][col] = a[row][col] * scalar;
      }
    }
    return b;
  }

  // Convert an array to a list of doubles
  static List<double> arrayToList(List<double> array) {
    return List<double>.from(array);
  }

  // Create a list from multiple arguments
  // static List<double> createList(double... args) {
  //   return List<double>.from(args);
  // }

  // Convert radians to degrees
  static double radsToDegrees(double rads) {
    return rads * (180.0 / pi);
  }

  // Add polar angles and normalize
  static double polarAdd(double initHeading, double deltaHeading) {
    double currHeading = initHeading + deltaHeading;
    if (currHeading < -pi) {
      return (currHeading % pi) + pi;
    } else if (currHeading > pi) {
      return (currHeading % pi) - pi;
    } else {
      return currHeading;
    }
  }

  // Calculate a complementary filter for heading
  static double calcCompHeading(double magHeading, double gyroHeading) {
    if (magHeading < 0) magHeading = magHeading % (2.0 * pi);
    if (gyroHeading < 0) gyroHeading = gyroHeading % (2.0 * pi);

    double compHeading = 0.02 * magHeading + 0.98 * gyroHeading;

    if (compHeading > pi) compHeading = (compHeading % pi) - pi;

    return compHeading;
  }

  // Calculate the norm (magnitude) of a vector
  static double calcNorm(List<double> args) {
    double sumSq = 0;
    for (double arg in args) {
      sumSq += arg * arg;
    }
    return sqrt(sumSq);
  }
}
