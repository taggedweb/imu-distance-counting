import 'dart:math';

/// A 2D Kalman Filter implementation for position tracking
/// Used for filtering sensor data and providing position estimates with uncertainty
class KalmanFilter {
  // State vector [x, y, vx, vy] - position and velocity
  late List<double> _state;

  // State covariance matrix (4x4)
  late List<List<double>> _covariance;

  // Process noise covariance matrix (4x4)
  late List<List<double>> _processNoise;

  // Measurement noise covariance matrix (2x2)
  late List<List<double>> _measurementNoise;

  // State transition matrix (4x4)
  late List<List<double>> _stateTransition;

  // Measurement matrix (2x4) - we observe position only
  late List<List<double>> _measurementMatrix;

  // Time step (dt)
  double _dt = 1.0;

  /// Constructor
  /// [processNoise] - Process noise level (affects prediction uncertainty)
  /// [measurementNoise] - Measurement noise level (affects sensor trust)
  /// [initialX] - Initial X position
  /// [initialY] - Initial Y position
  KalmanFilter({
    double processNoise = 0.1,
    double measurementNoise = 1.0,
    double initialX = 0.0,
    double initialY = 0.0,
  }) {
    _initializeMatrices(processNoise, measurementNoise);
    _state = [initialX, initialY, 0.0, 0.0]; // [x, y, vx, vy]
  }

  /// Initialize all matrices used in the Kalman filter
  void _initializeMatrices(double processNoise, double measurementNoise) {
    // Initialize state covariance matrix (high initial uncertainty)
    _covariance = [
      [10.0, 0.0, 0.0, 0.0],
      [0.0, 10.0, 0.0, 0.0],
      [0.0, 0.0, 10.0, 0.0],
      [0.0, 0.0, 0.0, 10.0],
    ];

    // State transition matrix (constant velocity model)
    _stateTransition = [
      [1.0, 0.0, _dt, 0.0],
      [0.0, 1.0, 0.0, _dt],
      [0.0, 0.0, 1.0, 0.0],
      [0.0, 0.0, 0.0, 1.0],
    ];

    // Measurement matrix (we observe position only)
    _measurementMatrix = [
      [1.0, 0.0, 0.0, 0.0],
      [0.0, 1.0, 0.0, 0.0],
    ];

    // Process noise covariance matrix
    double pn = processNoise;
    _processNoise = [
      [pn * _dt * _dt / 4, 0.0, pn * _dt / 2, 0.0],
      [0.0, pn * _dt * _dt / 4, 0.0, pn * _dt / 2],
      [pn * _dt / 2, 0.0, pn, 0.0],
      [0.0, pn * _dt / 2, 0.0, pn],
    ];

    // Measurement noise covariance matrix
    _measurementNoise = [
      [measurementNoise, 0.0],
      [0.0, measurementNoise],
    ];
  }

  /// Predict the next state based on the motion model
  void predict() {
    // Predict state: x_k = F * x_{k-1}
    _state = _multiplyMatrixVector(_stateTransition, _state);

    // Predict covariance: P_k = F * P_{k-1} * F^T + Q
    List<List<double>> temp = _multiplyMatrices(_stateTransition, _covariance);
    List<List<double>> FT = _transposeMatrix(_stateTransition);
    _covariance = _addMatrices(_multiplyMatrices(temp, FT), _processNoise);
  }

  /// Update the filter with a new measurement
  /// [x] - Measured X position
  /// [y] - Measured Y position
  /// [customNoise] - Optional custom measurement noise for this update
  void update(double x, double y, {double? customNoise}) {
    List<double> measurement = [x, y];

    // Use custom noise if provided
    List<List<double>> R =
        customNoise != null
            ? [
              [customNoise, 0.0],
              [0.0, customNoise],
            ]
            : _measurementNoise;

    // Innovation: y = z - H * x
    List<double> predicted = _multiplyMatrixVector(_measurementMatrix, _state);
    List<double> innovation = [
      measurement[0] - predicted[0],
      measurement[1] - predicted[1],
    ];

    // Innovation covariance: S = H * P * H^T + R
    List<List<double>> HP = _multiplyMatrices(_measurementMatrix, _covariance);
    List<List<double>> HT = _transposeMatrix(_measurementMatrix);
    List<List<double>> S = _addMatrices(_multiplyMatrices(HP, HT), R);

    // Kalman gain: K = P * H^T * S^{-1}
    List<List<double>> PHT = _multiplyMatrices(_covariance, HT);
    List<List<double>> SInv = _invertMatrix2x2(S);
    List<List<double>> K = _multiplyMatrices(PHT, SInv);

    // Update state: x = x + K * y
    List<double> Ky = _multiplyMatrixVector(K, innovation);
    _state = [
      _state[0] + Ky[0],
      _state[1] + Ky[1],
      _state[2] + Ky[2],
      _state[3] + Ky[3],
    ];

    // Update covariance: P = (I - K * H) * P
    List<List<double>> KH = _multiplyMatrices(K, _measurementMatrix);
    List<List<double>> I = _identityMatrix(4);
    List<List<double>> IminusKH = _subtractMatrices(I, KH);
    _covariance = _multiplyMatrices(IminusKH, _covariance);
  }

  /// Get the current position estimate
  Map<String, double> getPosition() {
    return {'x': _state[0], 'y': _state[1]};
  }

  /// Get the current velocity estimate
  Map<String, double> getVelocity() {
    return {'vx': _state[2], 'vy': _state[3]};
  }

  /// Get position uncertainty (standard deviation)
  Map<String, double> getPositionUncertainty() {
    return {
      'x_std': sqrt(_covariance[0][0].abs()),
      'y_std': sqrt(_covariance[1][1].abs()),
    };
  }

  /// Get velocity uncertainty (standard deviation)
  Map<String, double> getVelocityUncertainty() {
    return {
      'vx_std': sqrt(_covariance[2][2].abs()),
      'vy_std': sqrt(_covariance[3][3].abs()),
    };
  }

  /// Reset the filter to a new position
  void reset(double x, double y) {
    _state = [x, y, 0.0, 0.0];
    // Reset covariance to high uncertainty
    _covariance = [
      [10.0, 0.0, 0.0, 0.0],
      [0.0, 10.0, 0.0, 0.0],
      [0.0, 0.0, 10.0, 0.0],
      [0.0, 0.0, 0.0, 10.0],
    ];
  }

  /// Update the time step for the motion model
  void setTimeStep(double dt) {
    _dt = dt;
    _updateStateTransitionMatrix();
    _updateProcessNoiseMatrix();
  }

  void _updateStateTransitionMatrix() {
    _stateTransition = [
      [1.0, 0.0, _dt, 0.0],
      [0.0, 1.0, 0.0, _dt],
      [0.0, 0.0, 1.0, 0.0],
      [0.0, 0.0, 0.0, 1.0],
    ];
  }

  void _updateProcessNoiseMatrix() {
    double pn = 0.1; // Default process noise
    _processNoise = [
      [pn * _dt * _dt / 4, 0.0, pn * _dt / 2, 0.0],
      [0.0, pn * _dt * _dt / 4, 0.0, pn * _dt / 2],
      [pn * _dt / 2, 0.0, pn, 0.0],
      [0.0, pn * _dt / 2, 0.0, pn],
    ];
  }

  // Matrix operations
  List<List<double>> _multiplyMatrices(
    List<List<double>> A,
    List<List<double>> B,
  ) {
    int rowsA = A.length;
    int colsA = A[0].length;
    int colsB = B[0].length;

    List<List<double>> result = List.generate(
      rowsA,
      (i) => List.filled(colsB, 0.0),
    );

    for (int i = 0; i < rowsA; i++) {
      for (int j = 0; j < colsB; j++) {
        for (int k = 0; k < colsA; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }

    return result;
  }

  List<double> _multiplyMatrixVector(
    List<List<double>> matrix,
    List<double> vector,
  ) {
    int rows = matrix.length;
    List<double> result = List.filled(rows, 0.0);

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < vector.length; j++) {
        result[i] += matrix[i][j] * vector[j];
      }
    }

    return result;
  }

  List<List<double>> _transposeMatrix(List<List<double>> matrix) {
    int rows = matrix.length;
    int cols = matrix[0].length;
    List<List<double>> result = List.generate(
      cols,
      (i) => List.filled(rows, 0.0),
    );

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[j][i] = matrix[i][j];
      }
    }

    return result;
  }

  List<List<double>> _addMatrices(List<List<double>> A, List<List<double>> B) {
    int rows = A.length;
    int cols = A[0].length;
    List<List<double>> result = List.generate(
      rows,
      (i) => List.filled(cols, 0.0),
    );

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[i][j] = A[i][j] + B[i][j];
      }
    }

    return result;
  }

  List<List<double>> _subtractMatrices(
    List<List<double>> A,
    List<List<double>> B,
  ) {
    int rows = A.length;
    int cols = A[0].length;
    List<List<double>> result = List.generate(
      rows,
      (i) => List.filled(cols, 0.0),
    );

    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        result[i][j] = A[i][j] - B[i][j];
      }
    }

    return result;
  }

  List<List<double>> _identityMatrix(int size) {
    List<List<double>> result = List.generate(
      size,
      (i) => List.filled(size, 0.0),
    );

    for (int i = 0; i < size; i++) {
      result[i][i] = 1.0;
    }

    return result;
  }

  List<List<double>> _invertMatrix2x2(List<List<double>> matrix) {
    double a = matrix[0][0];
    double b = matrix[0][1];
    double c = matrix[1][0];
    double d = matrix[1][1];

    double det = a * d - b * c;

    if (det.abs() < 1e-10) {
      // Matrix is singular, return identity
      return [
        [1.0, 0.0],
        [0.0, 1.0],
      ];
    }

    return [
      [d / det, -b / det],
      [-c / det, a / det],
    ];
  }
}
