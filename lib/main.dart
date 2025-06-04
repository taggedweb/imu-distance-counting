import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:vibration/vibration.dart';
import 'package:pedometer/pedometer.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:flutter_compass/flutter_compass.dart';
import 'kalman_filter.dart';
import 'ble_service.dart';

void main() {
  runApp(const StepCounterApp());
}

class StepCounterApp extends StatelessWidget {
  const StepCounterApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Accurate Indoor Navigation',
      theme: ThemeData(
        primarySwatch: Colors.blue,
        visualDensity: VisualDensity.adaptivePlatformDensity,
        textTheme: const TextTheme(
          headlineLarge: TextStyle(fontSize: 32, fontWeight: FontWeight.bold),
          headlineMedium: TextStyle(fontSize: 24, fontWeight: FontWeight.w600),
          bodyLarge: TextStyle(fontSize: 18),
          bodyMedium: TextStyle(fontSize: 16),
        ),
      ),
      home: const NavigationScreen(),
    );
  }
}

class NavigationScreen extends StatefulWidget {
  const NavigationScreen({super.key});

  @override
  State<NavigationScreen> createState() => _NavigationScreenState();
}

class _NavigationScreenState extends State<NavigationScreen> {
  static const platform = MethodChannel('step_counter/sensors');
  static const eventChannel = EventChannel('step_counter/sensor_events');

  late KalmanFilter _kalmanFilter;
  late BLEService _bleService;

  StreamSubscription? _sensorSubscription;
  StreamSubscription? _bleSubscription;

  // Navigation data
  int _steps = 0;
  double _distance = 0.0;
  double _positionX = 0.0;
  double _positionY = 0.0;
  double _accuracy = 0.0;
  double _compassHeading = 0.0;
  double _imuDistance = 0.0;
  bool _isWalking = false;

  // Raw sensor data for debugging
  List<double> _acceleration = [0.0, 0.0, 0.0];
  List<double> _gyroscope = [0.0, 0.0, 0.0];

  // Advanced tracking variables
  StreamSubscription<StepCount>? _stepCountStream;
  StreamSubscription<PedestrianStatus>? _pedestrianStatusStream;
  StreamSubscription<AccelerometerEvent>? _accelerometerStream;
  StreamSubscription<CompassEvent>? _compassStream;

  int _initialSteps = 0;
  bool _stepCountInitialized = false;

  // IMU tracking properties
  double _velocity = 0.0;
  double _imuX = 0.0;
  double _imuY = 0.0;
  double _smoothedHeading = 0.0;
  int _stillCounter = 0;
  DateTime? _lastUpdateTime;
  bool _imuInitialized = false;

  // Advanced step detection filtering with machine learning-like features
  final List<double> _accelerationBuffer = [];
  final List<DateTime> _stepTimestamps = [];
  final List<double> _stepMagnitudes = [];
  final List<double> _accelerationX = [];
  final List<double> _accelerationY = [];
  final List<double> _accelerationZ = [];
  final List<double> _filteredAcceleration = [];
  final List<double> _stepConfidenceScores = [];
  int _validatedSteps = 0;
  int _filteredSteps = 0;
  double _averageStepCadence = 0.0;
  DateTime? _lastValidStep;
  bool _isInWalkingPattern = false;

  // Advanced signal processing variables
  double _accelerationMean = 9.8;
  double _accelerationStd = 1.0;
  int _consecutiveSteps = 0;
  double _walkingConfidence = 0.0;
  final List<double> _stepIntervals = [];
  double _expectedStepInterval = 0.6;

  // Multi-dimensional analysis
  final List<List<double>> _accelerationWindow = [];
  static const int _analysisWindowSize = 50; // 1-2 seconds of data

  // Advanced pattern recognition
  final List<double> _peakAmplitudes = [];
  final List<double> _valleyAmplitudes = [];
  final List<DateTime> _peakTimestamps = [];
  final List<DateTime> _valleyTimestamps = [];

  // Step quality metrics
  double _stepRegularity = 0.0;
  double _stepSymmetry = 0.0;
  double _gaitStability = 0.0;

  // Calibration system
  bool _isCalibrated = false;
  bool _isCalibrating = false;
  int _calibrationStep = 0;
  final Map<String, dynamic> _calibrationData = {};

  // Adaptive thresholds (will be calibrated)
  double _minStepInterval = 0.3; // Will be calibrated
  double _maxStepInterval = 2.0; // Will be calibrated
  double _minWalkingCadence = 0.5; // Will be calibrated
  double _maxWalkingCadence = 3.0; // Will be calibrated
  double _stepMagnitudeThreshold = 1.5; // Will be calibrated
  double _shakingVarianceThreshold = 5.0; // Will be calibrated
  double _stillThreshold = 0.5; // Will be calibrated
  double _walkingThreshold = 2.0; // Will be calibrated

  // Calibration data storage
  final List<double> _stillReadings = [];
  final List<double> _shakingReadings = [];
  final List<double> _walkingReadings = [];
  final List<double> _walkingCadences = [];

  static const int _bufferSize = 20; // Keep last 20 acceleration readings

  // BLE data
  List<BLEBeacon> _detectedBeacons = [];
  bool _blePositionLock = false;

  // Settings
  double _stepLength = 0.7;
  bool _isTracking = false;
  bool _debugMode = false;

  // UI
  bool _permissionsGranted = false;
  String _statusMessage = 'Initializing...';
  List<String> _unavailableSensors = [];
  bool _sensorsChecked = false;

  @override
  void initState() {
    super.initState();
    _initializeNavigation();
  }

  Future<void> _initializeNavigation() async {
    // First check sensor availability
    await _checkSensorAvailability();

    // Show warning dialog if critical sensors are missing
    _showCriticalSensorWarningIfNeeded();

    // Then request permissions
    await _requestPermissions();
    // Navigation system initialization is now handled in _initializeNavigationSystem()
    // after permissions are granted
  }

  void _showCriticalSensorWarningIfNeeded() {
    if (!_sensorsChecked || !mounted) return;

    // Check if any critical sensors are missing
    bool hasCriticalMissing = _unavailableSensors.any(
      (sensor) => sensor.startsWith('❌'),
    );

    if (hasCriticalMissing) {
      Future.delayed(const Duration(milliseconds: 500), () {
        if (!mounted) return;

        showDialog(
          context: context,
          barrierDismissible: false,
          builder: (BuildContext context) {
            return AlertDialog(
              icon: const Icon(Icons.error, color: Colors.red, size: 48),
              title: const Text(
                'Critical Sensors Missing',
                style: TextStyle(
                  color: Colors.red,
                  fontWeight: FontWeight.bold,
                ),
              ),
              content: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text(
                    'Your device is missing essential sensors required for accurate indoor navigation:',
                    style: TextStyle(fontSize: 16),
                  ),
                  const SizedBox(height: 16),
                  ..._unavailableSensors
                      .where((sensor) => sensor.startsWith('❌'))
                      .map(
                        (sensor) => Padding(
                          padding: const EdgeInsets.only(bottom: 8.0),
                          child: Row(
                            children: [
                              const Text(
                                '•',
                                style: TextStyle(color: Colors.red),
                              ),
                              const SizedBox(width: 8),
                              Expanded(
                                child: Text(
                                  sensor.substring(sensor.indexOf(' ') + 1),
                                  style: const TextStyle(color: Colors.red),
                                ),
                              ),
                            ],
                          ),
                        ),
                      ),
                  const SizedBox(height: 16),
                  Container(
                    padding: const EdgeInsets.all(12),
                    decoration: BoxDecoration(
                      color: Colors.red.shade50,
                      borderRadius: BorderRadius.circular(8),
                      border: Border.all(color: Colors.red.shade200),
                    ),
                    child: const Text(
                      '⚠️ WARNING: The app may not function properly without these sensors. Navigation accuracy will be severely limited.',
                      style: TextStyle(
                        fontSize: 14,
                        color: Colors.red,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ),
                ],
              ),
              actions: [
                TextButton(
                  onPressed: () {
                    Navigator.of(context).pop();
                  },
                  child: const Text(
                    'Continue Anyway',
                    style: TextStyle(color: Colors.grey),
                  ),
                ),
                ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red,
                    foregroundColor: Colors.white,
                  ),
                  onPressed: () {
                    Navigator.of(context).pop();
                    // You could add logic here to exit the app or redirect to device settings
                  },
                  child: const Text('Understood'),
                ),
              ],
            );
          },
        );
      });
    }
  }

  Future<void> _requestPermissions() async {
    setState(() {
      _statusMessage = 'Requesting permissions...';
    });

    // List of required permissions based on Android version
    List<Permission> requiredPermissions = [
      Permission.activityRecognition,
      Permission.locationWhenInUse,
    ];

    // Add Bluetooth permissions based on Android version
    // For Android 12+ (API 31+), we need BLUETOOTH_SCAN and BLUETOOTH_CONNECT
    // For older versions, we need BLUETOOTH and BLUETOOTH_ADMIN
    requiredPermissions.addAll([
      Permission.bluetooth,
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
    ]);

    // Check current status of all permissions
    Map<Permission, PermissionStatus> statuses = {};
    for (Permission permission in requiredPermissions) {
      try {
        statuses[permission] = await permission.status;
      } catch (e) {
        // Some permissions might not be available on certain Android versions
        print('Permission $permission not available: $e');
      }
    }

    // Request permissions that are not granted
    List<Permission> toRequest = [];
    for (Permission permission in requiredPermissions) {
      if (statuses.containsKey(permission) &&
          statuses[permission] != PermissionStatus.granted) {
        toRequest.add(permission);
      }
    }

    if (toRequest.isNotEmpty) {
      try {
        // Request permissions one by one for better reliability
        for (Permission permission in toRequest) {
          PermissionStatus status = await permission.request();
          statuses[permission] = status;

          // Give a small delay between requests
          await Future.delayed(const Duration(milliseconds: 100));
        }
      } catch (e) {
        print('Error requesting permissions: $e');
        setState(() {
          _statusMessage = 'Error requesting permissions: $e';
        });
        return;
      }
    }

    // Check if critical permissions are granted
    bool activityRecognitionGranted =
        statuses[Permission.activityRecognition] == PermissionStatus.granted;
    bool locationGranted =
        statuses[Permission.locationWhenInUse] == PermissionStatus.granted;
    bool bluetoothGranted =
        statuses[Permission.bluetoothScan] == PermissionStatus.granted ||
        statuses[Permission.bluetooth] == PermissionStatus.granted;

    bool allCriticalGranted =
        activityRecognitionGranted && locationGranted && bluetoothGranted;

    // Provide detailed feedback
    List<String> deniedPermissions = [];
    if (!activityRecognitionGranted)
      deniedPermissions.add('Activity Recognition');
    if (!locationGranted) deniedPermissions.add('Location');
    if (!bluetoothGranted) deniedPermissions.add('Bluetooth');

    setState(() {
      _permissionsGranted = allCriticalGranted;
      if (allCriticalGranted) {
        _statusMessage = 'All permissions granted successfully!';
      } else {
        _statusMessage =
            'Missing critical permissions: ${deniedPermissions.join(', ')}. Please grant them for the app to work.';
      }
    });

    // If critical permissions are granted, initialize the navigation system
    if (allCriticalGranted) {
      await _initializeNavigationSystem();
    }
  }

  Future<void> _initializeNavigationSystem() async {
    try {
      _kalmanFilter = KalmanFilter(processNoise: 0.1, measurementNoise: 1.0);
      _bleService = BLEService();
      _setupBLEListening();
      _initAdvancedSensors();

      setState(() {
        _statusMessage = 'Ready to navigate';
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Error initializing navigation: $e';
      });
    }
  }

  /// Initialize advanced sensor tracking (pedometer, accelerometer, compass)
  void _initAdvancedSensors() {
    _initPedometer();
    _initAccelerometer();
    _initCompass();
    _resetAdvancedTracking();
  }

  /// Initialize the pedometer streams
  void _initPedometer() {
    _stepCountStream = Pedometer.stepCountStream.listen(
      _onStepCount,
      onError: (error) => print('Step count error: $error'),
      cancelOnError: false,
    );

    _pedestrianStatusStream = Pedometer.pedestrianStatusStream.listen(
      _onPedestrianStatusChanged,
      onError: (error) => print('Pedestrian status error: $error'),
      cancelOnError: false,
    );
  }

  /// Initialize the accelerometer for IMU-based distance tracking
  void _initAccelerometer() {
    _lastUpdateTime = DateTime.now();
    _accelerometerStream = accelerometerEvents.listen(
      _onAccelerometerEvent,
      onError: (error) => print('Accelerometer error: $error'),
      cancelOnError: false,
    );
  }

  /// Initialize the compass for heading tracking
  void _initCompass() {
    if (FlutterCompass.events != null) {
      _compassStream = FlutterCompass.events!.listen(
        _onCompassEvent,
        onError: (error) => print('Compass error: $error'),
        cancelOnError: false,
      );
    } else {
      print('Compass not available on this device');
    }
  }

  /// Reset advanced tracking variables
  void _resetAdvancedTracking() {
    _initialSteps = 0;
    _stepCountInitialized = false;
    _imuInitialized = false;

    // Reset IMU tracking variables
    _velocity = 0.0;
    _imuX = 0.0;
    _imuY = 0.0;
    _smoothedHeading = 0.0;
    _stillCounter = 0;
    _lastUpdateTime = null;

    // Reset step validation variables
    _accelerationBuffer.clear();
    _stepTimestamps.clear();
    _stepMagnitudes.clear();
    _validatedSteps = 0;
    _filteredSteps = 0;
    _averageStepCadence = 0.0;
    _lastValidStep = null;
    _isInWalkingPattern = false;

    setState(() {
      _steps = 0;
      _distance = 0.0;
      _imuDistance = 0.0;
      _compassHeading = 0.0;
      _isWalking = false;
    });
  }

  /// Start comprehensive calibration process
  Future<void> _startCalibration() async {
    setState(() {
      _isCalibrating = true;
      _calibrationStep = 0;
      _statusMessage = 'Starting calibration...';
    });

    // Clear previous calibration data
    _stillReadings.clear();
    _shakingReadings.clear();
    _walkingReadings.clear();
    _walkingCadences.clear();
    _calibrationData.clear();

    // Show calibration dialog
    _showCalibrationDialog();
  }

  /// Show step-by-step calibration dialog
  void _showCalibrationDialog() {
    final calibrationSteps = [
      {
        'title': 'Step 1: Still Position',
        'instruction':
            'Place your phone on a flat surface and keep it completely still for 10 seconds.',
        'icon': Icons.smartphone,
        'duration': 10,
      },
      {
        'title': 'Step 2: Vertical Position',
        'instruction':
            'Hold your phone vertically in your hand and keep it still for 5 seconds.',
        'icon': Icons.stay_current_portrait,
        'duration': 5,
      },
      {
        'title': 'Step 3: Horizontal Position',
        'instruction':
            'Hold your phone horizontally and keep it still for 5 seconds.',
        'icon': Icons.stay_current_landscape,
        'duration': 5,
      },
      {
        'title': 'Step 4: Shaking Detection',
        'instruction':
            'Shake your phone gently for 5 seconds to help distinguish shaking from walking.',
        'icon': Icons.vibration,
        'duration': 5,
      },
      {
        'title': 'Step 5: Walking Pattern',
        'instruction': 'Walk normally for 15 seconds at your usual pace.',
        'icon': Icons.directions_walk,
        'duration': 15,
      },
    ];

    if (_calibrationStep < calibrationSteps.length) {
      final step = calibrationSteps[_calibrationStep];

      showDialog(
        context: context,
        barrierDismissible: false,
        builder: (BuildContext context) {
          return StatefulBuilder(
            builder: (context, setState) {
              return AlertDialog(
                title: Row(
                  children: [
                    Icon(step['icon'] as IconData, color: Colors.blue),
                    const SizedBox(width: 8),
                    Text(step['title'] as String),
                  ],
                ),
                content: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Text(
                      step['instruction'] as String,
                      style: const TextStyle(fontSize: 16),
                    ),
                    const SizedBox(height: 20),
                    LinearProgressIndicator(
                      value: _calibrationStep / calibrationSteps.length,
                      backgroundColor: Colors.grey[300],
                      valueColor: AlwaysStoppedAnimation<Color>(Colors.blue),
                    ),
                    const SizedBox(height: 10),
                    Text(
                      'Step ${_calibrationStep + 1} of ${calibrationSteps.length}',
                      style: const TextStyle(color: Colors.grey),
                    ),
                  ],
                ),
                actions: [
                  ElevatedButton(
                    onPressed: () {
                      Navigator.of(context).pop();
                      _performCalibrationStep(
                        _calibrationStep,
                        step['duration'] as int,
                      );
                    },
                    child: const Text('Start'),
                  ),
                ],
              );
            },
          );
        },
      );
    } else {
      _finishCalibration();
    }
  }

  /// Perform individual calibration step
  Future<void> _performCalibrationStep(int step, int duration) async {
    setState(() {
      _statusMessage = 'Calibrating... Step ${step + 1}';
    });

    // Start collecting calibration data
    _startCalibrationDataCollection(step);

    // Wait for the specified duration
    await Future.delayed(Duration(seconds: duration));

    // Stop collecting data and process it
    _stopCalibrationDataCollection(step);

    // Move to next step
    setState(() {
      _calibrationStep++;
    });

    // Show next calibration step
    _showCalibrationDialog();
  }

  /// Start collecting calibration data for specific step
  void _startCalibrationDataCollection(int step) {
    // Implementation will collect sensor data based on step type
    _calibrationData['step_$step'] = {
      'start_time': DateTime.now(),
      'readings': <double>[],
      'type': step,
    };
  }

  /// Stop collecting calibration data and process it
  void _stopCalibrationDataCollection(int step) {
    final data = _calibrationData['step_$step'];
    if (data != null) {
      data['end_time'] = DateTime.now();
      _processCalibrationData(step, data);
    }
  }

  /// Process calibration data for each step
  void _processCalibrationData(int step, Map<String, dynamic> data) {
    switch (step) {
      case 0: // Still position (flat surface)
        _processStillCalibration(data);
        break;
      case 1: // Vertical position
        _processVerticalCalibration(data);
        break;
      case 2: // Horizontal position
        _processHorizontalCalibration(data);
        break;
      case 3: // Shaking detection
        _processShakingCalibration(data);
        break;
      case 4: // Walking pattern
        _processWalkingCalibration(data);
        break;
    }
  }

  /// Process still position calibration
  void _processStillCalibration(Map<String, dynamic> data) {
    List<double> readings = List<double>.from(data['readings'] ?? []);
    if (readings.isNotEmpty) {
      double avgMagnitude = readings.reduce((a, b) => a + b) / readings.length;
      double variance =
          readings
              .map((r) => pow(r - avgMagnitude, 2))
              .reduce((a, b) => a + b) /
          readings.length;

      _stillThreshold =
          avgMagnitude + (sqrt(variance) * 2); // 2 standard deviations
      _stillReadings.addAll(readings);

      print(
        'Still calibration: avg=$avgMagnitude, variance=$variance, threshold=$_stillThreshold',
      );
    }
  }

  /// Process vertical position calibration
  void _processVerticalCalibration(Map<String, dynamic> data) {
    // Store vertical position baseline
    List<double> readings = List<double>.from(data['readings'] ?? []);
    _calibrationData['vertical_baseline'] = readings;
    print('Vertical calibration: ${readings.length} readings collected');
  }

  /// Process horizontal position calibration
  void _processHorizontalCalibration(Map<String, dynamic> data) {
    // Store horizontal position baseline
    List<double> readings = List<double>.from(data['readings'] ?? []);
    _calibrationData['horizontal_baseline'] = readings;
    print('Horizontal calibration: ${readings.length} readings collected');
  }

  /// Process shaking calibration
  void _processShakingCalibration(Map<String, dynamic> data) {
    List<double> readings = List<double>.from(data['readings'] ?? []);
    if (readings.isNotEmpty) {
      double avgMagnitude = readings.reduce((a, b) => a + b) / readings.length;
      double variance =
          readings
              .map((r) => pow(r - avgMagnitude, 2))
              .reduce((a, b) => a + b) /
          readings.length;

      _shakingVarianceThreshold =
          variance * 0.5; // Use 50% of shaking variance as threshold
      _shakingReadings.addAll(readings);

      print(
        'Shaking calibration: avg=$avgMagnitude, variance=$variance, threshold=$_shakingVarianceThreshold',
      );
    }
  }

  /// Process walking calibration
  void _processWalkingCalibration(Map<String, dynamic> data) {
    List<double> readings = List<double>.from(data['readings'] ?? []);
    if (readings.isNotEmpty) {
      double avgMagnitude = readings.reduce((a, b) => a + b) / readings.length;
      double maxMagnitude = readings.reduce((a, b) => a > b ? a : b);
      double minMagnitude = readings.reduce((a, b) => a < b ? a : b);

      // Set walking thresholds based on actual walking data
      _stepMagnitudeThreshold =
          avgMagnitude * 0.7; // 70% of average walking magnitude
      _walkingThreshold = maxMagnitude * 0.8; // 80% of peak walking magnitude

      // Calculate cadence from walking pattern
      _calculateWalkingCadence(readings, data);

      _walkingReadings.addAll(readings);

      print(
        'Walking calibration: avg=$avgMagnitude, min=$minMagnitude, max=$maxMagnitude',
      );
      print(
        'Step threshold=$_stepMagnitudeThreshold, walking threshold=$_walkingThreshold',
      );
    }
  }

  /// Calculate walking cadence from calibration data
  void _calculateWalkingCadence(
    List<double> readings,
    Map<String, dynamic> data,
  ) {
    DateTime startTime = data['start_time'];
    DateTime endTime = data['end_time'];
    double duration = endTime.difference(startTime).inMilliseconds / 1000.0;

    // Count peaks in acceleration data to estimate step frequency
    int peakCount = 0;
    double threshold = readings.reduce((a, b) => a + b) / readings.length;

    for (int i = 1; i < readings.length - 1; i++) {
      if (readings[i] > threshold &&
          readings[i] > readings[i - 1] &&
          readings[i] > readings[i + 1]) {
        peakCount++;
      }
    }

    double estimatedCadence = peakCount / duration;

    // Set adaptive cadence thresholds
    _minWalkingCadence = estimatedCadence * 0.5; // 50% of estimated cadence
    _maxWalkingCadence = estimatedCadence * 2.0; // 200% of estimated cadence

    // Set step interval thresholds
    if (estimatedCadence > 0) {
      _minStepInterval =
          (1.0 / _maxWalkingCadence) * 0.8; // 80% of max frequency
      _maxStepInterval =
          (1.0 / _minWalkingCadence) * 1.2; // 120% of min frequency
    }

    _walkingCadences.add(estimatedCadence);

    print(
      'Cadence calibration: estimated=$estimatedCadence Hz, range=$_minWalkingCadence-$_maxWalkingCadence Hz',
    );
    print('Step interval range: $_minStepInterval-$_maxStepInterval seconds');
  }

  /// Finish calibration and save results
  void _finishCalibration() {
    setState(() {
      _isCalibrating = false;
      _isCalibrated = true;
      _statusMessage = 'Calibration completed successfully!';
    });

    // Show calibration results
    _showCalibrationResults();
  }

  /// Show calibration results to user
  void _showCalibrationResults() {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Row(
            children: [
              Icon(Icons.check_circle, color: Colors.green),
              SizedBox(width: 8),
              Text('Calibration Complete'),
            ],
          ),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text(
                'Your device has been calibrated successfully!',
                style: TextStyle(fontSize: 16),
              ),
              const SizedBox(height: 16),
              Text('Still threshold: ${_stillThreshold.toStringAsFixed(2)}'),
              Text(
                'Shaking threshold: ${_shakingVarianceThreshold.toStringAsFixed(2)}',
              ),
              Text(
                'Step magnitude: ${_stepMagnitudeThreshold.toStringAsFixed(2)}',
              ),
              Text(
                'Walking cadence: ${_minWalkingCadence.toStringAsFixed(1)}-${_maxWalkingCadence.toStringAsFixed(1)} Hz',
              ),
              Text(
                'Step intervals: ${_minStepInterval.toStringAsFixed(2)}-${_maxStepInterval.toStringAsFixed(2)}s',
              ),
              const SizedBox(height: 16),
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color: Colors.green.shade50,
                  borderRadius: BorderRadius.circular(8),
                  border: Border.all(color: Colors.green.shade200),
                ),
                child: const Text(
                  '✅ The app will now provide much more accurate step detection and distance tracking.',
                  style: TextStyle(
                    fontSize: 14,
                    color: Colors.green,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ),
            ],
          ),
          actions: [
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
              ),
              onPressed: () {
                Navigator.of(context).pop();
              },
              child: const Text('Start Navigation'),
            ),
          ],
        );
      },
    );
  }

  void _setupBLEListening() {
    _bleSubscription = _bleService.positionStream.listen((position) {
      if (position != null && _isTracking) {
        // Update Kalman filter with BLE position (high accuracy)
        _kalmanFilter.update(
          position['x']!,
          position['y']!,
          customNoise: position['accuracy'],
        );

        final filteredPos = _kalmanFilter.getPosition();
        final uncertainty = _kalmanFilter.getPositionUncertainty();

        setState(() {
          _positionX = filteredPos['x']!;
          _positionY = filteredPos['y']!;
          _accuracy = sqrt(
            uncertainty['x_std']! * uncertainty['x_std']! +
                uncertainty['y_std']! * uncertainty['y_std']!,
          );
          _blePositionLock = true;
        });

        // Provide haptic feedback for position lock
        Vibration.vibrate(duration: 100);
      }
    });
  }

  /// Handle step count updates with sophisticated filtering
  void _onStepCount(StepCount event) {
    if (!_isTracking) return;

    print("Raw step count: ${event.steps}");

    // Initialize step counter on first step count received during navigation
    if (!_stepCountInitialized) {
      _initialSteps = event.steps;
      _stepCountInitialized = true;
      _validatedSteps = 0;
      _filteredSteps = 0;
      print('Step counter initialized with ${event.steps} initial steps');
      setState(() {
        _steps = 0;
      });
      return;
    }

    // Calculate raw steps since we started counting
    int rawSteps = event.steps - _initialSteps;
    if (rawSteps < 0) return; // Invalid step count

    // Apply sophisticated step validation
    int newSteps = rawSteps - _validatedSteps - _filteredSteps;
    if (newSteps > 0) {
      int validSteps = _validateSteps(newSteps);
      _validatedSteps += validSteps;
      _filteredSteps += (newSteps - validSteps);

      print(
        'Raw steps: $rawSteps, New: $newSteps, Valid: $validSteps, Filtered: ${newSteps - validSteps}',
      );
      print(
        'Total validated: $_validatedSteps, Total filtered: $_filteredSteps',
      );
    }

    setState(() {
      _steps = _validatedSteps;
    });

    // Calculate distance based on validated step count
    double stepBasedDistance = _steps * _stepLength;
    double totalDistance = max(stepBasedDistance, _imuDistance);

    setState(() {
      _distance = totalDistance;
    });
  }

  /// Sophisticated step validation to filter out false positives
  int _validateSteps(int newSteps) {
    final now = DateTime.now();
    int validatedCount = 0;

    for (int i = 0; i < newSteps; i++) {
      bool isValid = _isValidStep(now);
      if (isValid) {
        validatedCount++;
        _recordValidStep(now);
      }
    }

    return validatedCount;
  }

  /// Determine if a step is valid using advanced multi-criteria analysis

  /// Advanced shaking detection using multiple signal analysis techniques
  bool _isAdvancedShakingDetected() {
    if (_accelerationBuffer.length < 15) return false;

    // Calculate variance over different time windows
    double shortTermVariance = _calculateVarianceWindow(
      _accelerationBuffer,
      5,
    ); // 0.2 seconds
    double mediumTermVariance = _calculateVarianceWindow(
      _accelerationBuffer,
      10,
    ); // 0.4 seconds

    // Shaking typically shows high variance in short and medium terms
    bool highShortTermVariance =
        shortTermVariance > _shakingVarianceThreshold * 1.2;
    bool highMediumTermVariance =
        mediumTermVariance > _shakingVarianceThreshold * 0.8;

    // Check for rapid direction changes (characteristic of shaking)
    int directionChanges = _countRecentDirectionChanges();
    bool tooManyDirectionChanges =
        directionChanges > 8; // More than 8 changes in recent data

    // Check for excessive acceleration spikes
    double recentMaxAcceleration = _accelerationBuffer
        .skip(_accelerationBuffer.length - 10)
        .reduce((a, b) => a > b ? a : b);
    bool excessiveSpikes =
        recentMaxAcceleration > _shakingVarianceThreshold * 2;

    return (highShortTermVariance && highMediumTermVariance) ||
        tooManyDirectionChanges ||
        excessiveSpikes;
  }

  /// Calculate variance for a specific window size
  double _calculateVarianceWindow(List<double> data, int windowSize) {
    if (data.length < windowSize) return 0.0;

    List<double> window = data.skip(data.length - windowSize).toList();
    double mean = window.reduce((a, b) => a + b) / window.length;
    double variance =
        window.map((value) => pow(value - mean, 2)).reduce((a, b) => a + b) /
        window.length;

    return variance;
  }

  /// Count direction changes in recent acceleration data
  int _countRecentDirectionChanges() {
    if (_accelerationBuffer.length < 6) return 0;

    int changes = 0;
    List<double> recent =
        _accelerationBuffer.skip(_accelerationBuffer.length - 6).toList();

    for (int i = 2; i < recent.length; i++) {
      bool wasIncreasing = recent[i - 1] > recent[i - 2];
      bool isIncreasing = recent[i] > recent[i - 1];

      if (wasIncreasing != isIncreasing) changes++;
    }

    return changes;
  }

  /// Advanced walking cadence validation with pattern recognition
  bool _isValidWalkingCadenceAdvanced(DateTime stepTime) {
    if (_stepTimestamps.length < 3) return true; // Not enough data, allow step

    // Get recent steps within last 8 seconds for pattern analysis
    List<DateTime> recentSteps =
        _stepTimestamps
            .where((timestamp) => stepTime.difference(timestamp).inSeconds < 8)
            .toList();

    if (recentSteps.length < 2) return true;

    // Calculate instantaneous cadence
    double totalTime =
        recentSteps.last.difference(recentSteps.first).inMilliseconds / 1000.0;
    double instantaneousCadence = (recentSteps.length - 1) / totalTime;

    // Check if cadence is within calibrated range
    if (instantaneousCadence < _minWalkingCadence ||
        instantaneousCadence > _maxWalkingCadence) {
      return false;
    }

    // Check for cadence consistency (not too erratic)
    if (recentSteps.length >= 4) {
      List<double> intervals = [];
      for (int i = 1; i < recentSteps.length; i++) {
        intervals.add(
          recentSteps[i].difference(recentSteps[i - 1]).inMilliseconds / 1000.0,
        );
      }

      double meanInterval =
          intervals.reduce((a, b) => a + b) / intervals.length;
      double intervalVariance =
          intervals
              .map((interval) => pow(interval - meanInterval, 2))
              .reduce((a, b) => a + b) /
          intervals.length;
      double coefficientOfVariation = sqrt(intervalVariance) / meanInterval;

      // Reject if steps are too erratic (CV > 0.4 means 40% variation)
      if (coefficientOfVariation > 0.4) {
        return false;
      }
    }

    return true;
  }

  /// Advanced step magnitude validation with dynamic thresholds
  bool _hasValidStepMagnitudeAdvanced() {
    if (_stepMagnitudes.length < 5) return true; // Not enough data

    // Get recent magnitude data
    List<double> recentMagnitudes =
        _stepMagnitudes.skip(_stepMagnitudes.length - 5).toList();
    double averageRecentMagnitude =
        recentMagnitudes.reduce((a, b) => a + b) / recentMagnitudes.length;

    // Dynamic threshold based on recent activity level
    double dynamicThreshold = _stepMagnitudeThreshold;

    // Lower threshold if we're in a consistent walking pattern
    if (_isInWalkingPattern && _stepTimestamps.length > 5) {
      dynamicThreshold *= 0.85; // 15% more lenient when walking consistently
    }

    // Higher threshold if recent variance suggests irregular motion
    if (_accelerationBuffer.length >= 10) {
      double recentVariance = _calculateVarianceWindow(_accelerationBuffer, 10);
      if (recentVariance > _shakingVarianceThreshold * 0.6) {
        dynamicThreshold *= 1.3; // 30% more strict when motion is irregular
      }
    }

    return averageRecentMagnitude >= dynamicThreshold;
  }

  /// Check for consistent gait pattern
  bool _isConsistentGaitPattern() {
    if (_stepTimestamps.length < 4)
      return true; // Not enough data for pattern analysis

    // Analyze step interval consistency
    List<double> intervals = [];
    for (int i = 1; i < _stepTimestamps.length; i++) {
      intervals.add(
        _stepTimestamps[i].difference(_stepTimestamps[i - 1]).inMilliseconds /
            1000.0,
      );
    }

    if (intervals.length < 3) return true;

    // Calculate consistency metrics
    double meanInterval = intervals.reduce((a, b) => a + b) / intervals.length;
    double variance =
        intervals
            .map((interval) => pow(interval - meanInterval, 2))
            .reduce((a, b) => a + b) /
        intervals.length;
    double standardDeviation = sqrt(variance);

    // Good gait should have consistent step intervals
    double consistencyRatio = standardDeviation / meanInterval;

    // Accept if consistency ratio is reasonable (less than 35% variation)
    return consistencyRatio < 0.35;
  }

  /// Validate motion state for step validity
  bool _isValidMotionState() {
    // If we detect we're walking, be more lenient
    if (_isWalking) return true;

    // If we have established a walking pattern recently, allow steps
    if (_isInWalkingPattern && _stepTimestamps.isNotEmpty) {
      DateTime lastStep = _stepTimestamps.last;
      double timeSinceLastStep =
          DateTime.now().difference(lastStep).inMilliseconds / 1000.0;

      // Allow if last step was recent (within 3 seconds)
      if (timeSinceLastStep < 3.0) return true;
    }

    // For first few steps when walking state isn't detected yet, be lenient
    if (_stepTimestamps.length < 3) return true;

    return false;
  }

  /// Determine if a step is valid based on multiple criteria
  bool _isValidStep(DateTime stepTime) {
    // If not calibrated, use relaxed validation
    if (!_isCalibrated) {
      return _isValidStepRelaxed(stepTime);
    }

    // Check if user is in walking state (relaxed check)
    if (!_isWalking && _stepTimestamps.isEmpty) {
      // Allow first few steps even if not marked as walking
      print('Step allowed: First steps or walking state not detected yet');
    }

    // Check minimum time interval since last valid step
    if (_lastValidStep != null) {
      double timeSinceLastStep =
          stepTime.difference(_lastValidStep!).inMilliseconds / 1000.0;

      if (timeSinceLastStep < _minStepInterval) {
        print(
          'Step rejected: Too frequent (${timeSinceLastStep.toStringAsFixed(2)}s < ${_minStepInterval.toStringAsFixed(2)}s)',
        );
        return false;
      }

      if (timeSinceLastStep > _maxStepInterval) {
        print(
          'Step rejected: Too infrequent (${timeSinceLastStep.toStringAsFixed(2)}s > ${_maxStepInterval.toStringAsFixed(2)}s)',
        );
        // Reset walking pattern if steps are too far apart
        _isInWalkingPattern = false;
        return false;
      }
    }

    // Check for shaking pattern in acceleration buffer (if calibrated)
    if (_isShakingDetected()) {
      print(
        'Step rejected: Shaking pattern detected (variance > ${_shakingVarianceThreshold.toStringAsFixed(2)})',
      );
      return false;
    }

    // Check walking cadence pattern (relaxed)
    if (!_isValidWalkingCadence()) {
      print('Step rejected: Invalid walking cadence');
      return false;
    }

    // Check if acceleration magnitude supports a step (relaxed)
    if (!_hasValidStepMagnitude()) {
      print('Step rejected: Insufficient acceleration magnitude');
      return false;
    }

    return true;
  }

  /// Relaxed step validation when not calibrated
  bool _isValidStepRelaxed(DateTime stepTime) {
    // Very basic validation - mainly time-based
    if (_lastValidStep != null) {
      double timeSinceLastStep =
          stepTime.difference(_lastValidStep!).inMilliseconds / 1000.0;

      // Very relaxed time constraints
      if (timeSinceLastStep < 0.1) {
        // 100ms minimum
        print(
          'Step rejected: Too frequent (${timeSinceLastStep.toStringAsFixed(2)}s)',
        );
        return false;
      }

      if (timeSinceLastStep > 5.0) {
        // 5 seconds maximum
        print(
          'Step rejected: Too infrequent (${timeSinceLastStep.toStringAsFixed(2)}s)',
        );
        return false;
      }
    }

    // Basic shaking detection
    if (_accelerationBuffer.length >= 5) {
      double mean =
          _accelerationBuffer.reduce((a, b) => a + b) /
          _accelerationBuffer.length;
      double variance =
          _accelerationBuffer
              .map((mag) => pow(mag - mean, 2))
              .reduce((a, b) => a + b) /
          _accelerationBuffer.length;

      if (variance > 10.0) {
        // Very high threshold for uncalibrated
        print(
          'Step rejected: High variance detected (${variance.toStringAsFixed(2)})',
        );
        return false;
      }
    }

    return true;
  }

  /// Record a valid step and update walking pattern
  void _recordValidStep(DateTime stepTime) {
    _lastValidStep = stepTime;
    _stepTimestamps.add(stepTime);

    // Keep only recent step timestamps for pattern analysis
    final cutoffTime = stepTime.subtract(Duration(seconds: 10));
    _stepTimestamps.removeWhere((timestamp) => timestamp.isBefore(cutoffTime));

    // Update walking pattern status
    _isInWalkingPattern = _stepTimestamps.length >= 3;

    // Calculate average cadence
    if (_stepTimestamps.length >= 2) {
      double totalTime =
          _stepTimestamps.last
              .difference(_stepTimestamps.first)
              .inMilliseconds /
          1000.0;
      _averageStepCadence = (_stepTimestamps.length - 1) / totalTime;
    }
  }

  /// Detect if the device is being shaken based on acceleration variance
  bool _isShakingDetected() {
    if (_accelerationBuffer.length < 10) return false;

    // Calculate variance of recent acceleration magnitudes
    double mean =
        _accelerationBuffer.reduce((a, b) => a + b) /
        _accelerationBuffer.length;
    double variance =
        _accelerationBuffer
            .map((mag) => pow(mag - mean, 2))
            .reduce((a, b) => a + b) /
        _accelerationBuffer.length;

    // High variance indicates shaking/irregular movement
    return variance > _shakingVarianceThreshold;
  }

  /// Check if current cadence matches normal walking patterns
  bool _isValidWalkingCadence() {
    if (_stepTimestamps.length < 3) return true; // Not enough data, allow step

    // Calculate recent cadence (steps per second)
    final recentSteps =
        _stepTimestamps
            .where(
              (timestamp) =>
                  DateTime.now().difference(timestamp).inSeconds <= 5,
            )
            .toList();

    if (recentSteps.length < 2) return true;

    double recentTime =
        recentSteps.last.difference(recentSteps.first).inMilliseconds / 1000.0;
    double recentCadence = (recentSteps.length - 1) / recentTime;

    return recentCadence >= _minWalkingCadence &&
        recentCadence <= _maxWalkingCadence;
  }

  /// Check if acceleration magnitude supports a valid step
  bool _hasValidStepMagnitude() {
    if (_stepMagnitudes.isEmpty) return true; // No data available, allow step

    // Check if recent acceleration peaks indicate actual steps
    double averageMagnitude =
        _stepMagnitudes.reduce((a, b) => a + b) / _stepMagnitudes.length;
    return averageMagnitude >= _stepMagnitudeThreshold;
  }

  /// Handle accelerometer events with advanced signal processing
  void _onAccelerometerEvent(AccelerometerEvent event) {
    if (!_isTracking && !_isCalibrating) return;

    final now = DateTime.now();

    // Store raw acceleration data for multi-dimensional analysis
    _accelerationX.add(event.x);
    _accelerationY.add(event.y);
    _accelerationZ.add(event.z);

    // Maintain buffer size
    if (_accelerationX.length > _analysisWindowSize) {
      _accelerationX.removeAt(0);
      _accelerationY.removeAt(0);
      _accelerationZ.removeAt(0);
    }

    // Calculate total acceleration magnitude
    double accelerationMagnitude = sqrt(
      event.x * event.x + event.y * event.y + event.z * event.z,
    );

    // Apply advanced low-pass filtering to reduce noise
    double filteredMagnitude = _applyLowPassFilter(accelerationMagnitude);
    _filteredAcceleration.add(filteredMagnitude);
    if (_filteredAcceleration.length > _analysisWindowSize) {
      _filteredAcceleration.removeAt(0);
    }

    // Store acceleration window for pattern analysis
    _accelerationWindow.add([
      event.x,
      event.y,
      event.z,
      accelerationMagnitude,
      filteredMagnitude,
    ]);
    if (_accelerationWindow.length > _analysisWindowSize) {
      _accelerationWindow.removeAt(0);
    }

    // Update statistical measures
    _updateAccelerationStatistics();

    // Collect calibration data if calibrating
    if (_isCalibrating && _calibrationStep >= 0 && _calibrationStep < 5) {
      final data = _calibrationData['step_$_calibrationStep'];
      if (data != null) {
        (data['readings'] as List<double>).add(filteredMagnitude);
      }
    }

    // Return early if not tracking (only calibrating)
    if (!_isTracking) return;

    // Update acceleration buffer for step validation
    _accelerationBuffer.add(filteredMagnitude);
    if (_accelerationBuffer.length > _bufferSize) {
      _accelerationBuffer.removeAt(0);
    }

    // Perform advanced step detection
    _performAdvancedStepDetection(now, filteredMagnitude);

    // Continue with IMU distance calculation
    _processIMUData(event, now);
  }

  /// Apply low-pass filter to reduce noise in acceleration data
  double _applyLowPassFilter(double currentValue) {
    if (_filteredAcceleration.isEmpty) return currentValue;

    double alpha = 0.2; // Low-pass filter coefficient
    double previousFiltered = _filteredAcceleration.last;
    return alpha * currentValue + (1 - alpha) * previousFiltered;
  }

  /// Update statistical measures for acceleration data
  void _updateAccelerationStatistics() {
    if (_filteredAcceleration.length < 10) return;

    // Calculate running mean and standard deviation
    double sum = _filteredAcceleration.reduce((a, b) => a + b);
    _accelerationMean = sum / _filteredAcceleration.length;

    double variance = 0.0;
    for (double value in _filteredAcceleration) {
      variance += pow(value - _accelerationMean, 2);
    }
    _accelerationStd = sqrt(variance / _filteredAcceleration.length);
  }

  /// Perform advanced step detection using multiple algorithms
  void _performAdvancedStepDetection(DateTime timestamp, double magnitude) {
    if (_filteredAcceleration.length < 20) return; // Need enough data

    // Detect peaks and valleys in the signal
    _detectPeaksAndValleys(timestamp, magnitude);

    // Calculate step confidence using multiple criteria
    double stepConfidence = _calculateStepConfidence(timestamp, magnitude);
    _stepConfidenceScores.add(stepConfidence);
    if (_stepConfidenceScores.length > 20) {
      _stepConfidenceScores.removeAt(0);
    }

    // Check if this is a valid step using advanced criteria
    if (_isAdvancedValidStep(timestamp, magnitude, stepConfidence)) {
      _recordAdvancedValidStep(timestamp, magnitude, stepConfidence);
    }

    // Update walking pattern analysis
    _updateWalkingPatternAnalysis();
  }

  /// Detect peaks and valleys in acceleration signal
  void _detectPeaksAndValleys(DateTime timestamp, double magnitude) {
    if (_filteredAcceleration.length < 5) return;

    int currentIndex = _filteredAcceleration.length - 1;
    if (currentIndex < 2) return;

    double current = _filteredAcceleration[currentIndex];
    double previous = _filteredAcceleration[currentIndex - 1];
    double beforePrevious = _filteredAcceleration[currentIndex - 2];

    // Peak detection: current value is higher than neighbors
    if (previous > beforePrevious &&
        previous > current &&
        previous > _accelerationMean + _accelerationStd * 0.5) {
      _peakAmplitudes.add(previous);
      _peakTimestamps.add(timestamp);

      // Maintain buffer size
      if (_peakAmplitudes.length > 20) {
        _peakAmplitudes.removeAt(0);
        _peakTimestamps.removeAt(0);
      }
    }

    // Valley detection: current value is lower than neighbors
    if (previous < beforePrevious &&
        previous < current &&
        previous < _accelerationMean - _accelerationStd * 0.5) {
      _valleyAmplitudes.add(previous);
      _valleyTimestamps.add(timestamp);

      // Maintain buffer size
      if (_valleyAmplitudes.length > 20) {
        _valleyAmplitudes.removeAt(0);
        _valleyTimestamps.removeAt(0);
      }
    }
  }

  /// Calculate step confidence score using multiple criteria
  double _calculateStepConfidence(DateTime timestamp, double magnitude) {
    double confidence = 0.0;

    // Criterion 1: Magnitude relative to mean (25% weight)
    double magnitudeScore =
        (magnitude - _accelerationMean).abs() / (_accelerationStd + 0.1);
    confidence += magnitudeScore.clamp(0.0, 1.0) * 0.25;

    // Criterion 2: Peak-valley pattern (25% weight)
    double patternScore = _calculatePatternScore();
    confidence += patternScore * 0.25;

    // Criterion 3: Temporal consistency (25% weight)
    double temporalScore = _calculateTemporalScore(timestamp);
    confidence += temporalScore * 0.25;

    // Criterion 4: Walking state consistency (25% weight)
    double walkingScore = _calculateWalkingStateScore();
    confidence += walkingScore * 0.25;

    return confidence.clamp(0.0, 1.0);
  }

  /// Calculate pattern score based on peak-valley analysis
  double _calculatePatternScore() {
    if (_peakAmplitudes.length < 2 || _valleyAmplitudes.length < 2) return 0.0;

    // Check amplitude consistency
    double avgPeakAmplitude =
        _peakAmplitudes.reduce((a, b) => a + b) / _peakAmplitudes.length;
    double avgValleyAmplitude =
        _valleyAmplitudes.reduce((a, b) => a + b) / _valleyAmplitudes.length;
    double amplitudeDifference = avgPeakAmplitude - avgValleyAmplitude;

    // Good step pattern should have clear distinction between peaks and valleys
    if (amplitudeDifference < 0.5) return 0.0;
    if (amplitudeDifference > 5.0) return 0.3; // Too high might be shaking

    return (amplitudeDifference / 3.0).clamp(0.0, 1.0);
  }

  /// Calculate temporal consistency score
  double _calculateTemporalScore(DateTime currentTime) {
    if (_stepTimestamps.length < 2) return 0.5; // Neutral score for first steps

    double avgInterval = _expectedStepInterval;
    if (_stepIntervals.isNotEmpty) {
      avgInterval =
          _stepIntervals.reduce((a, b) => a + b) / _stepIntervals.length;
    }

    DateTime lastStep = _stepTimestamps.last;
    double currentInterval =
        currentTime.difference(lastStep).inMilliseconds / 1000.0;

    // Score based on how close the interval is to expected
    double intervalDifference = (currentInterval - avgInterval).abs();
    if (intervalDifference > 1.0) return 0.0; // Too far from expected

    return (1.0 - intervalDifference).clamp(0.0, 1.0);
  }

  /// Calculate walking state consistency score
  double _calculateWalkingStateScore() {
    // Base score on walking state
    double baseScore = _isWalking ? 0.8 : 0.3;

    // Adjust based on consecutive steps
    double consecutiveBonus = (_consecutiveSteps / 10.0).clamp(0.0, 0.2);

    // Adjust based on overall walking confidence
    return (baseScore + consecutiveBonus + _walkingConfidence * 0.1).clamp(
      0.0,
      1.0,
    );
  }

  /// Advanced step validation using multiple criteria and confidence score
  bool _isAdvancedValidStep(
    DateTime stepTime,
    double magnitude,
    double confidence,
  ) {
    // Primary confidence threshold
    if (confidence < 0.4) return false;

    // If not calibrated, use relaxed validation
    if (!_isCalibrated) {
      return _isValidStepRelaxedAdvanced(stepTime, confidence);
    }

    // Time interval validation
    if (_lastValidStep != null) {
      double timeSinceLastStep =
          stepTime.difference(_lastValidStep!).inMilliseconds / 1000.0;

      if (timeSinceLastStep < _minStepInterval) return false;
      if (timeSinceLastStep > _maxStepInterval) {
        _resetWalkingPattern();
        if (timeSinceLastStep > 3.0) return false; // Too long gap
      }
    }

    // Advanced shaking detection
    if (_isAdvancedShakingDetected()) return false;

    // Walking cadence validation
    if (!_isValidAdvancedWalkingCadence(stepTime)) return false;

    // Magnitude validation with adaptive threshold
    if (!_hasValidAdvancedStepMagnitude(magnitude)) return false;

    // Gait pattern validation
    if (!_isValidGaitPattern()) return false;

    return true;
  }

  /// Relaxed validation for uncalibrated devices with confidence scoring
  bool _isValidStepRelaxedAdvanced(DateTime stepTime, double confidence) {
    if (confidence < 0.3) return false;

    if (_lastValidStep != null) {
      double timeSinceLastStep =
          stepTime.difference(_lastValidStep!).inMilliseconds / 1000.0;
      if (timeSinceLastStep < 0.2) return false;
      if (timeSinceLastStep > 3.0) return false;
    }

    // Very basic shaking detection for uncalibrated
    if (_accelerationBuffer.length >= 10) {
      double variance = _calculateVariance(_accelerationBuffer);
      if (variance > 15.0) return false; // Obvious shaking
    }

    return true;
  }

  /// Advanced shaking detection using multiple signals

  /// Calculate variance for a specific axis
  double _calculateAxisVariance(int axisIndex) {
    if (_accelerationWindow.length < 5) return 0.0;

    List<double> axisData =
        _accelerationWindow.map((window) => window[axisIndex]).toList();
    double mean = axisData.reduce((a, b) => a + b) / axisData.length;
    double variance =
        axisData.map((value) => pow(value - mean, 2)).reduce((a, b) => a + b) /
        axisData.length;

    return variance;
  }

  /// Count direction changes in acceleration
  int _countDirectionChanges() {
    if (_filteredAcceleration.length < 3) return 0;

    int changes = 0;
    for (int i = 2; i < _filteredAcceleration.length; i++) {
      double prev2 = _filteredAcceleration[i - 2];
      double prev1 = _filteredAcceleration[i - 1];
      double current = _filteredAcceleration[i];

      bool wasIncreasing = prev1 > prev2;
      bool isIncreasing = current > prev1;

      if (wasIncreasing != isIncreasing) changes++;
    }

    return changes;
  }

  /// Advanced walking cadence validation
  bool _isValidAdvancedWalkingCadence(DateTime stepTime) {
    if (_stepTimestamps.length < 3) return true;

    // Calculate recent cadence over multiple steps
    List<DateTime> recentSteps =
        _stepTimestamps
            .where((timestamp) => stepTime.difference(timestamp).inSeconds < 10)
            .toList();

    if (recentSteps.length < 2) return true;

    double totalTime =
        recentSteps.last.difference(recentSteps.first).inMilliseconds / 1000.0;
    double recentCadence = (recentSteps.length - 1) / totalTime;

    // Update expected step interval
    if (recentCadence > 0) {
      _expectedStepInterval = 1.0 / recentCadence;
    }

    return recentCadence >= _minWalkingCadence &&
        recentCadence <= _maxWalkingCadence;
  }

  /// Advanced step magnitude validation with adaptive threshold
  bool _hasValidAdvancedStepMagnitude(double magnitude) {
    if (_filteredAcceleration.length < 10) return true;

    // Adaptive threshold based on recent data
    double recentMean = _accelerationMean;
    double dynamicThreshold = _stepMagnitudeThreshold;

    // Adjust threshold based on walking confidence
    if (_walkingConfidence > 0.7) {
      dynamicThreshold *= 0.8; // Lower threshold when confident we're walking
    }

    return magnitude >= dynamicThreshold ||
        magnitude >= recentMean + _accelerationStd;
  }

  /// Validate gait pattern for step regularity and stability
  bool _isValidGaitPattern() {
    if (_stepIntervals.length < 3) return true; // Not enough data

    // Calculate step regularity (consistency of intervals)
    _stepRegularity = _calculateStepRegularity();

    // Calculate step symmetry (balance between left and right steps)
    _stepSymmetry = _calculateStepSymmetry();

    // Calculate gait stability
    _gaitStability = _calculateGaitStability();

    // Accept step if gait metrics are reasonable
    return _stepRegularity > 0.3 && _stepSymmetry > 0.2 && _gaitStability > 0.3;
  }

  /// Calculate step regularity based on interval consistency
  double _calculateStepRegularity() {
    if (_stepIntervals.length < 3) return 1.0;

    double mean =
        _stepIntervals.reduce((a, b) => a + b) / _stepIntervals.length;
    double variance =
        _stepIntervals
            .map((interval) => pow(interval - mean, 2))
            .reduce((a, b) => a + b) /
        _stepIntervals.length;
    double coefficient = sqrt(variance) / mean;

    return (1.0 - coefficient).clamp(0.0, 1.0);
  }

  /// Calculate step symmetry (placeholder for now)
  double _calculateStepSymmetry() {
    // This would typically require more sophisticated analysis of acceleration patterns
    // For now, return a reasonable default
    return 0.8;
  }

  /// Calculate gait stability based on acceleration variance
  double _calculateGaitStability() {
    if (_filteredAcceleration.length < 20) return 1.0;

    double variance = _calculateVariance(_filteredAcceleration);
    double stability = 1.0 / (1.0 + variance / 5.0);

    return stability.clamp(0.0, 1.0);
  }

  /// Record a valid step with advanced metrics
  void _recordAdvancedValidStep(
    DateTime stepTime,
    double magnitude,
    double confidence,
  ) {
    _lastValidStep = stepTime;
    _stepTimestamps.add(stepTime);
    _stepMagnitudes.add(magnitude);
    _consecutiveSteps++;

    // Record step interval
    if (_stepTimestamps.length >= 2) {
      double interval =
          stepTime
              .difference(_stepTimestamps[_stepTimestamps.length - 2])
              .inMilliseconds /
          1000.0;
      _stepIntervals.add(interval);
      if (_stepIntervals.length > 20) {
        _stepIntervals.removeAt(0);
      }
    }

    // Update walking confidence
    _walkingConfidence = (_walkingConfidence * 0.9 + confidence * 0.1).clamp(
      0.0,
      1.0,
    );

    // Keep only recent data
    final cutoffTime = stepTime.subtract(Duration(seconds: 30));
    _stepTimestamps.removeWhere((timestamp) => timestamp.isBefore(cutoffTime));

    // Remove corresponding step magnitudes
    while (_stepMagnitudes.length > _stepTimestamps.length) {
      _stepMagnitudes.removeAt(0);
    }

    // Update walking pattern status
    _isInWalkingPattern =
        _stepTimestamps.length >= 3 && _walkingConfidence > 0.5;

    // Calculate average cadence
    if (_stepTimestamps.length >= 2) {
      double totalTime =
          _stepTimestamps.last
              .difference(_stepTimestamps.first)
              .inMilliseconds /
          1000.0;
      _averageStepCadence = (_stepTimestamps.length - 1) / totalTime;
    }
  }

  /// Update walking pattern analysis
  void _updateWalkingPatternAnalysis() {
    // Reset consecutive steps if too much time has passed
    if (_lastValidStep != null) {
      double timeSinceLastStep =
          DateTime.now().difference(_lastValidStep!).inMilliseconds / 1000.0;
      if (timeSinceLastStep > 2.0) {
        _consecutiveSteps = 0;
        _walkingConfidence *= 0.8; // Reduce confidence
      }
    }
  }

  /// Reset walking pattern when long gap detected
  void _resetWalkingPattern() {
    _consecutiveSteps = 0;
    _isInWalkingPattern = false;
    _walkingConfidence *= 0.5;
  }

  /// Calculate variance of a list of values
  double _calculateVariance(List<double> values) {
    if (values.length < 2) return 0.0;

    double mean = values.reduce((a, b) => a + b) / values.length;
    double variance =
        values.map((value) => pow(value - mean, 2)).reduce((a, b) => a + b) /
        values.length;

    return variance;
  }

  /// Process IMU data for distance calculation (separated from step detection)
  void _processIMUData(AccelerometerEvent event, DateTime now) {
    // Initialize timing on first accelerometer event during navigation
    if (!_imuInitialized || _lastUpdateTime == null) {
      _lastUpdateTime = now;
      _imuInitialized = true;
      return;
    }

    final elapsedTime =
        now.difference(_lastUpdateTime!).inMilliseconds / 1000.0;
    _lastUpdateTime = now;

    // Skip if time difference is too small or too large
    if (elapsedTime <= 0.01 || elapsedTime > 1.0) return;

    // Apply filtering to acceleration for better accuracy
    double accX = event.x * 0.8; // Reduced sensitivity

    // Calculate total acceleration magnitude for still detection
    double aTotal = sqrt(
      event.x * event.x + event.y * event.y + event.z * event.z,
    );

    // Use calibrated still detection threshold
    bool isStill =
        _isCalibrated
            ? (aTotal < _stillThreshold)
            : (aTotal - 9.8).abs() < 0.2; // Fallback to default

    if (isStill) {
      _stillCounter++;
      // Reset velocity when still for longer period
      if (_stillCounter > 10) {
        _velocity = 0.0;
      }
    } else {
      _stillCounter = 0;
      _velocity += accX * elapsedTime;
    }

    // Limit velocity to reasonable walking values
    _velocity = _velocity.clamp(-2.0, 2.0);

    // Update position based on velocity and smoothed heading
    if (_velocity.abs() > 0.01) {
      double dx = _velocity * cos(_smoothedHeading) * elapsedTime;
      double dy = _velocity * sin(_smoothedHeading) * elapsedTime;
      _imuX += dx;
      _imuY += dy;
    }

    // Update IMU distance
    _imuDistance = sqrt(_imuX * _imuX + _imuY * _imuY);

    // Update raw sensor data for debugging
    setState(() {
      _acceleration = [event.x, event.y, event.z];
    });
  }

  /// Handle compass events for heading
  void _onCompassEvent(CompassEvent event) {
    if (event.heading != null) {
      // Store the original heading in degrees for display
      double heading = event.heading!;

      setState(() {
        _compassHeading = heading;
      });

      // Convert to radians for calculations and apply smoothing
      double rawHeadingRad = heading * (pi / 180);

      // Apply exponential smoothing with higher alpha for more smoothing
      const double alpha = 0.9; // Increased smoothing factor
      if (_smoothedHeading == 0.0) {
        _smoothedHeading = rawHeadingRad;
      } else {
        // Handle angular wrapping for proper smoothing
        double diff = rawHeadingRad - _smoothedHeading;
        if (diff > pi) {
          diff -= 2 * pi;
        } else if (diff < -pi) {
          diff += 2 * pi;
        }
        _smoothedHeading =
            alpha * _smoothedHeading + (1 - alpha) * rawHeadingRad;

        // Normalize to [0, 2π) range
        if (_smoothedHeading < 0) {
          _smoothedHeading += 2 * pi;
        } else if (_smoothedHeading >= 2 * pi) {
          _smoothedHeading -= 2 * pi;
        }
      }
    }
  }

  /// Handle pedestrian status changes (walking/stopped)
  void _onPedestrianStatusChanged(PedestrianStatus event) {
    bool isWalking = event.status == 'walking';
    setState(() {
      _isWalking = isWalking;
    });
  }

  Future<void> _startTracking() async {
    try {
      await platform.invokeMethod('startSensors');
      await _bleService.startScanning();

      _sensorSubscription = eventChannel.receiveBroadcastStream().listen(
        (data) => _handleSensorData(data),
        onError: (error) {
          setState(() {
            _statusMessage = 'Sensor error: $error';
          });
        },
      );

      // Reset and start advanced tracking
      _resetAdvancedTracking();

      setState(() {
        _isTracking = true;
        _statusMessage = 'Tracking active';
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Failed to start tracking: $e';
      });
    }
  }

  Future<void> _stopTracking() async {
    try {
      await platform.invokeMethod('stopSensors');
      await _bleService.stopScanning();
      await _sensorSubscription?.cancel();

      // Cancel advanced sensor streams
      await _stepCountStream?.cancel();
      await _pedestrianStatusStream?.cancel();
      await _accelerometerStream?.cancel();
      await _compassStream?.cancel();

      setState(() {
        _isTracking = false;
        _statusMessage = 'Tracking stopped';
        _blePositionLock = false;
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Failed to stop tracking: $e';
      });
    }
  }

  void _handleSensorData(dynamic data) {
    if (data is Map) {
      // Predict next position using Kalman filter
      _kalmanFilter.predict();

      // Get raw sensor data
      final steps = data['steps'] ?? 0;
      final sensorX = data['positionX'] ?? 0.0;
      final sensorY = data['positionY'] ?? 0.0;
      final acceleration = List<double>.from(
        data['acceleration'] ?? [0.0, 0.0, 0.0],
      );
      final gyroscope = List<double>.from(data['gyroscope'] ?? [0.0, 0.0, 0.0]);

      // Update Kalman filter with IMU-derived position (lower accuracy)
      if (!_blePositionLock) {
        _kalmanFilter.update(sensorX, sensorY, customNoise: 2.0);
      }

      // Get filtered position
      final filteredPos = _kalmanFilter.getPosition();
      final uncertainty = _kalmanFilter.getPositionUncertainty();

      setState(() {
        _steps = steps;
        _distance = steps * _stepLength;
        _positionX = filteredPos['x']!;
        _positionY = filteredPos['y']!;
        _acceleration = acceleration;
        _gyroscope = gyroscope;
        _accuracy = sqrt(
          uncertainty['x_std']! * uncertainty['x_std']! +
              uncertainty['y_std']! * uncertainty['y_std']!,
        );
        _detectedBeacons = _bleService.getDetectedBeacons();
      });

      // Reset BLE position lock after some time
      if (_blePositionLock) {
        Timer(const Duration(seconds: 5), () {
          setState(() {
            _blePositionLock = false;
          });
        });
      }
    }
  }

  Future<void> _resetPosition() async {
    try {
      await platform.invokeMethod('resetPosition', {'x': 0.0, 'y': 0.0});

      _kalmanFilter.reset(0.0, 0.0);

      setState(() {
        _positionX = 0.0;
        _positionY = 0.0;
        _distance = 0.0;
        _steps = 0;
        _statusMessage = 'Position reset to origin';
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Failed to reset position: $e';
      });
    }
  }

  Future<void> _updateStepLength(double newLength) async {
    try {
      await platform.invokeMethod('setStepLength', {'stepLength': newLength});

      setState(() {
        _stepLength = newLength;
        _distance = _steps * _stepLength;
      });
    } catch (e) {
      setState(() {
        _statusMessage = 'Failed to update step length: $e';
      });
    }
  }

  Future<void> _checkSensorAvailability() async {
    try {
      setState(() {
        _statusMessage = 'Checking sensor availability...';
      });

      // Check sensor availability through platform channel
      final Map<dynamic, dynamic> result = await platform.invokeMethod(
        'checkSensorAvailability',
      );

      final Map<dynamic, dynamic> sensorStatus = result['sensorStatus'] ?? {};
      final List<dynamic> missingCritical = result['missingCritical'] ?? [];
      final List<dynamic> missingOptional = result['missingOptional'] ?? [];
      final bool allCriticalAvailable = result['allCriticalAvailable'] ?? false;
      final bool canFunction = result['canFunction'] ?? false;

      List<String> unavailableSensors = [];

      // Add critical missing sensors with impact description
      for (String sensor in missingCritical.cast<String>()) {
        switch (sensor) {
          case 'Accelerometer':
            unavailableSensors.add(
              '❌ Accelerometer (CRITICAL: No motion detection possible)',
            );
            break;
          case 'Gyroscope':
            unavailableSensors.add(
              '❌ Gyroscope (CRITICAL: No rotation tracking possible)',
            );
            break;
          default:
            unavailableSensors.add('❌ $sensor (CRITICAL)');
        }
      }

      // Add optional missing sensors with workarounds
      for (String sensor in missingOptional.cast<String>()) {
        switch (sensor) {
          case 'Step Counter':
            unavailableSensors.add(
              '⚠️ Step Counter (Using accelerometer-based detection instead)',
            );
            break;
          case 'Magnetometer/Compass':
            unavailableSensors.add(
              '⚠️ Magnetometer/Compass (Direction tracking limited to gyroscope integration)',
            );
            break;
          case 'Bluetooth LE':
            unavailableSensors.add(
              '⚠️ Bluetooth LE (BLE beacon positioning unavailable)',
            );
            break;
          case 'Bluetooth (disabled)':
            unavailableSensors.add(
              '⚠️ Bluetooth disabled (Enable in settings for BLE beacons)',
            );
            break;
          default:
            unavailableSensors.add('⚠️ $sensor (Optional)');
        }
      }

      _unavailableSensors = unavailableSensors;
      _sensorsChecked = true;

      setState(() {
        if (!canFunction) {
          _statusMessage =
              'CRITICAL: Essential sensors missing. Navigation functionality severely limited.';
        } else if (!allCriticalAvailable) {
          _statusMessage =
              'Warning: Some critical sensors missing. Reduced accuracy expected.';
        } else if (missingOptional.isNotEmpty) {
          _statusMessage =
              'Ready! Some optional sensors missing but core functionality available.';
        } else {
          _statusMessage =
              'Excellent! All sensors available for maximum accuracy.';
        }
      });

      // Show detailed sensor status in debug mode
      if (_debugMode) {
        print('Sensor availability check results:');
        print('Can function: $canFunction');
        print('All critical available: $allCriticalAvailable');
        sensorStatus.forEach((sensor, available) {
          print('$sensor: $available');
        });
        print('Missing critical: $missingCritical');
        print('Missing optional: $missingOptional');
      }
    } catch (e) {
      print('Error checking sensor availability: $e');
      setState(() {
        _statusMessage =
            'Could not check sensor availability. Some features may not work.';
        _unavailableSensors = [
          '❌ Sensor check failed - Unknown device capabilities',
        ];
        _sensorsChecked = true;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Indoor Navigation'),
        backgroundColor: Colors.blue.shade800,
        foregroundColor: Colors.white,
        actions: [
          IconButton(
            icon: Icon(
              _debugMode ? Icons.bug_report : Icons.bug_report_outlined,
            ),
            onPressed: () {
              setState(() {
                _debugMode = !_debugMode;
              });
            },
          ),
        ],
      ),
      body:
          !_permissionsGranted
              ? _buildPermissionScreen()
              : SingleChildScrollView(
                padding: const EdgeInsets.all(16.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    _buildStatusCard(),
                    const SizedBox(height: 16),
                    _buildSensorStatusCard(),
                    const SizedBox(height: 16),
                    _buildNavigationCard(),
                    const SizedBox(height: 16),
                    _buildControlsCard(),
                    if (_debugMode) ...[
                      const SizedBox(height: 16),
                      _buildDebugCard(),
                    ],
                  ],
                ),
              ),
    );
  }

  Widget _buildPermissionScreen() {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(24.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.security, size: 64, color: Colors.orange),
            const SizedBox(height: 24),
            const Text(
              'Permissions Required',
              style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 16),
            const Text(
              'This app requires the following permissions for accurate indoor navigation:',
              style: TextStyle(fontSize: 16),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 16),
            const Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text('• Activity Recognition - For step counting'),
                Text('• Bluetooth - For BLE beacon detection'),
                Text('• Location - For positioning accuracy'),
              ],
            ),
            const SizedBox(height: 24),
            Text(
              _statusMessage,
              style: const TextStyle(fontSize: 14, color: Colors.grey),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 24),
            ElevatedButton(
              onPressed: _requestPermissions,
              child: const Text('Grant Permissions'),
            ),
            const SizedBox(height: 12),
            TextButton(
              onPressed: () async {
                await openAppSettings();
              },
              child: const Text('Open App Settings'),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  _isTracking ? Icons.gps_fixed : Icons.gps_off,
                  color: _isTracking ? Colors.green : Colors.grey,
                ),
                const SizedBox(width: 8),
                Text(
                  _isTracking ? 'TRACKING ACTIVE' : 'TRACKING INACTIVE',
                  style: TextStyle(
                    fontWeight: FontWeight.bold,
                    color: _isTracking ? Colors.green : Colors.grey,
                  ),
                ),
                const Spacer(),
                if (_blePositionLock)
                  const Icon(Icons.bluetooth_connected, color: Colors.blue),
              ],
            ),
            const SizedBox(height: 8),
            Text(_statusMessage, style: const TextStyle(fontSize: 14)),
          ],
        ),
      ),
    );
  }

  Widget _buildSensorStatusCard() {
    if (!_sensorsChecked) {
      return Card(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Row(
                children: [
                  const Icon(Icons.sensors, color: Colors.grey),
                  const SizedBox(width: 8),
                  const Text(
                    'Sensor Status',
                    style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              const SizedBox(height: 12),
              const Row(
                children: [
                  SizedBox(
                    width: 20,
                    height: 20,
                    child: CircularProgressIndicator(strokeWidth: 2),
                  ),
                  SizedBox(width: 12),
                  Text('Checking sensor availability...'),
                ],
              ),
            ],
          ),
        ),
      );
    }

    // Only show sensor status card if there are issues or in debug mode
    if (_unavailableSensors.isEmpty && !_debugMode) {
      return const SizedBox.shrink();
    }

    return Card(
      color:
          _unavailableSensors.any((s) => s.startsWith('❌'))
              ? Colors.red.shade50
              : _unavailableSensors.isNotEmpty
              ? Colors.orange.shade50
              : Colors.green.shade50,
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(
                  _unavailableSensors.any((s) => s.startsWith('❌'))
                      ? Icons.error
                      : _unavailableSensors.isNotEmpty
                      ? Icons.warning
                      : Icons.check_circle,
                  color:
                      _unavailableSensors.any((s) => s.startsWith('❌'))
                          ? Colors.red
                          : _unavailableSensors.isNotEmpty
                          ? Colors.orange
                          : Colors.green,
                ),
                const SizedBox(width: 8),
                const Text(
                  'Sensor Status',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const SizedBox(height: 12),
            if (_unavailableSensors.isEmpty) ...[
              const Row(
                children: [
                  Icon(Icons.check_circle, color: Colors.green, size: 20),
                  SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      'All sensors available - Maximum accuracy expected!',
                      style: TextStyle(
                        color: Colors.green,
                        fontWeight: FontWeight.w500,
                      ),
                    ),
                  ),
                ],
              ),
            ] else ...[
              ..._unavailableSensors.map(
                (sensor) => Padding(
                  padding: const EdgeInsets.only(bottom: 8.0),
                  child: Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        sensor.split(' ')[0], // Get the emoji
                        style: const TextStyle(fontSize: 16),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(
                          sensor.substring(
                            sensor.indexOf(' ') + 1,
                          ), // Get text after emoji
                          style: TextStyle(
                            color:
                                sensor.startsWith('❌')
                                    ? Colors.red.shade700
                                    : Colors.orange.shade700,
                          ),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              const SizedBox(height: 8),
              Container(
                padding: const EdgeInsets.all(12),
                decoration: BoxDecoration(
                  color:
                      _unavailableSensors.any((s) => s.startsWith('❌'))
                          ? Colors.red.shade100
                          : Colors.orange.shade100,
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Row(
                  children: [
                    Icon(
                      Icons.info_outline,
                      size: 16,
                      color:
                          _unavailableSensors.any((s) => s.startsWith('❌'))
                              ? Colors.red.shade700
                              : Colors.orange.shade700,
                    ),
                    const SizedBox(width: 8),
                    Expanded(
                      child: Text(
                        _unavailableSensors.any((s) => s.startsWith('❌'))
                            ? 'Critical sensors missing may severely impact navigation accuracy'
                            : 'Missing sensors have workarounds - reduced accuracy expected',
                        style: TextStyle(
                          fontSize: 12,
                          color:
                              _unavailableSensors.any((s) => s.startsWith('❌'))
                                  ? Colors.red.shade700
                                  : Colors.orange.shade700,
                          fontStyle: FontStyle.italic,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 12),
              ElevatedButton.icon(
                onPressed: _showSensorWorkarounds,
                icon: const Icon(Icons.lightbulb_outline, size: 18),
                label: const Text('Show Workarounds & Tips'),
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.blue.shade100,
                  foregroundColor: Colors.blue.shade700,
                  elevation: 0,
                ),
              ),
            ],
          ],
        ),
      ),
    );
  }

  void _showSensorWorkarounds() {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Row(
            children: [
              Icon(Icons.lightbulb, color: Colors.blue),
              SizedBox(width: 8),
              Text('Sensor Workarounds & Tips'),
            ],
          ),
          content: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                if (_unavailableSensors.any(
                  (s) => s.contains('Accelerometer'),
                )) ...[
                  _buildWorkaroundTile(
                    '❌ Accelerometer Missing',
                    'Critical Impact: No motion detection possible',
                    [
                      'Consider using a different device with accelerometer support',
                      'Some older or very basic devices may lack this sensor',
                      'Without accelerometer, step counting and movement detection are impossible',
                    ],
                    Colors.red,
                  ),
                  const SizedBox(height: 16),
                ],
                if (_unavailableSensors.any(
                  (s) => s.contains('Gyroscope'),
                )) ...[
                  _buildWorkaroundTile(
                    '❌ Gyroscope Missing',
                    'Critical Impact: No rotation tracking possible',
                    [
                      'Consider using a device with gyroscope support',
                      'Direction changes will not be detected accurately',
                      'Navigation will rely solely on step counting in straight lines',
                    ],
                    Colors.red,
                  ),
                  const SizedBox(height: 16),
                ],
                if (_unavailableSensors.any(
                  (s) => s.contains('Step Counter'),
                )) ...[
                  _buildWorkaroundTile(
                    '⚠️ Step Counter Missing',
                    'Workaround: Using accelerometer-based detection',
                    [
                      'The app will detect steps using accelerometer data',
                      'May be slightly less accurate than hardware step counter',
                      'Calibrate step length carefully for better accuracy',
                      'Works well for most walking patterns',
                    ],
                    Colors.orange,
                  ),
                  const SizedBox(height: 16),
                ],
                if (_unavailableSensors.any(
                  (s) => s.contains('Magnetometer'),
                )) ...[
                  _buildWorkaroundTile(
                    '⚠️ Magnetometer/Compass Missing',
                    'Workaround: Gyroscope-only direction tracking',
                    [
                      'Direction tracking limited to gyroscope integration',
                      'No magnetic north reference - all directions are relative',
                      'Direction drift may occur over time',
                      'Reset position frequently to maintain accuracy',
                      'Consider marking known reference directions manually',
                    ],
                    Colors.orange,
                  ),
                  const SizedBox(height: 16),
                ],
                if (_unavailableSensors.any(
                  (s) => s.contains('Bluetooth'),
                )) ...[
                  _buildWorkaroundTile(
                    '⚠️ Bluetooth Issues',
                    'Impact: BLE beacon positioning unavailable',
                    [
                      if (_unavailableSensors.any(
                        (s) => s.contains('disabled'),
                      ))
                        'Enable Bluetooth in device settings'
                      else
                        'Device lacks Bluetooth LE support',
                      'Navigation will rely entirely on IMU sensors',
                      'No absolute position correction from beacons',
                      'Position drift will accumulate over time',
                      'Reset position at known landmarks frequently',
                    ],
                    _unavailableSensors.any((s) => s.contains('disabled'))
                        ? Colors.orange
                        : Colors.red,
                  ),
                  const SizedBox(height: 16),
                ],
                Container(
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.blue.shade50,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.blue.shade200),
                  ),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      const Row(
                        children: [
                          Icon(
                            Icons.tips_and_updates,
                            color: Colors.blue,
                            size: 20,
                          ),
                          SizedBox(width: 8),
                          Text(
                            'General Tips for Better Accuracy',
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                              color: Colors.blue,
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(height: 8),
                      const Text('• Keep device in consistent pocket/position'),
                      const Text('• Walk at steady, normal pace'),
                      const Text('• Reset position at known landmarks'),
                      const Text(
                        '• Calibrate step length using known distances',
                      ),
                      const Text(
                        '• Use BLE beacons when available for best accuracy',
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: const Text('Close'),
            ),
          ],
        );
      },
    );
  }

  Widget _buildWorkaroundTile(
    String title,
    String subtitle,
    List<String> tips,
    Color color,
  ) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            title,
            style: TextStyle(
              fontWeight: FontWeight.bold,
              color: color,
              fontSize: 16,
            ),
          ),
          const SizedBox(height: 4),
          Text(
            subtitle,
            style: TextStyle(color: color, fontStyle: FontStyle.italic),
          ),
          const SizedBox(height: 8),
          ...tips.map(
            (tip) => Padding(
              padding: const EdgeInsets.only(bottom: 4),
              child: Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('• ', style: TextStyle(color: color)),
                  Expanded(
                    child: Text(
                      tip,
                      style: TextStyle(color: color.withOpacity(0.8)),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildNavigationCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Navigation Data',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Steps',
                    _steps.toString(),
                    Icons.directions_walk,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Step Distance',
                    '${(_steps * _stepLength).toStringAsFixed(1)} m',
                    Icons.straighten,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'IMU Distance',
                    '${_imuDistance.toStringAsFixed(1)} m',
                    Icons.timeline,
                    color: Colors.purple,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Total Distance',
                    '${_distance.toStringAsFixed(1)} m',
                    Icons.straighten,
                    color: Colors.green,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Position X',
                    '${_positionX.toStringAsFixed(1)} m',
                    Icons.place,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Position Y',
                    '${_positionY.toStringAsFixed(1)} m',
                    Icons.place,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Compass',
                    '${_compassHeading.toStringAsFixed(0)}°',
                    Icons.explore,
                    color: Colors.blue,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'IMU Heading',
                    '${(_smoothedHeading * 180 / pi).toStringAsFixed(0)}°',
                    Icons.compass_calibration,
                    color: Colors.orange,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Walking',
                    _isWalking ? 'Yes' : 'No',
                    _isWalking ? Icons.directions_walk : Icons.accessibility,
                    color: _isWalking ? Colors.green : Colors.grey,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Accuracy',
                    '±${_accuracy.toStringAsFixed(1)} m',
                    Icons.track_changes,
                    color:
                        _accuracy < 1.0
                            ? Colors.green
                            : _accuracy < 2.0
                            ? Colors.orange
                            : Colors.red,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Step filtering information
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Valid Steps',
                    _validatedSteps.toString(),
                    Icons.check_circle,
                    color: Colors.green,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Filtered',
                    _filteredSteps.toString(),
                    Icons.block,
                    color: _filteredSteps > 0 ? Colors.red : Colors.grey,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Cadence',
                    '${_averageStepCadence.toStringAsFixed(1)} Hz',
                    Icons.speed,
                    color:
                        _averageStepCadence > 0.5 && _averageStepCadence < 3.0
                            ? Colors.green
                            : Colors.orange,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Pattern',
                    _isInWalkingPattern ? 'Walking' : 'Irregular',
                    _isInWalkingPattern ? Icons.trending_up : Icons.warning,
                    color: _isInWalkingPattern ? Colors.green : Colors.orange,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Calibration status
            Row(
              children: [
                Expanded(
                  child: _buildMetricTile(
                    'Calibration',
                    _isCalibrating
                        ? 'In Progress'
                        : _isCalibrated
                        ? 'Complete'
                        : 'Not Started',
                    _isCalibrating
                        ? Icons.tune
                        : _isCalibrated
                        ? Icons.check_circle
                        : Icons.warning,
                    color:
                        _isCalibrating
                            ? Colors.blue
                            : _isCalibrated
                            ? Colors.green
                            : Colors.orange,
                  ),
                ),
                Expanded(
                  child: _buildMetricTile(
                    'Detection Mode',
                    _isCalibrated ? 'Adaptive' : 'Basic',
                    _isCalibrated ? Icons.psychology : Icons.sensors,
                    color: _isCalibrated ? Colors.green : Colors.grey,
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildMetricTile(
    String label,
    String value,
    IconData icon, {
    Color? color,
  }) {
    return Container(
      padding: const EdgeInsets.all(12),
      margin: const EdgeInsets.symmetric(horizontal: 4),
      decoration: BoxDecoration(
        color: Colors.grey.shade100,
        borderRadius: BorderRadius.circular(8),
      ),
      child: Column(
        children: [
          Icon(icon, color: color ?? Colors.blue.shade700),
          const SizedBox(height: 4),
          Text(
            label,
            style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
          ),
          Text(
            value,
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.bold,
              color: color ?? Colors.black87,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildControlsCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const Text(
              'Controls',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _isTracking ? _stopTracking : _startTracking,
                    icon: Icon(_isTracking ? Icons.stop : Icons.play_arrow),
                    label: Text(_isTracking ? 'Stop' : 'Start'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: _isTracking ? Colors.red : Colors.green,
                      foregroundColor: Colors.white,
                    ),
                  ),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _resetPosition,
                    icon: const Icon(Icons.refresh),
                    label: const Text('Reset'),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Calibration button
            ElevatedButton.icon(
              onPressed: _isCalibrating ? null : _startCalibration,
              icon: Icon(
                _isCalibrated ? Icons.check_circle : Icons.tune,
                color: _isCalibrated ? Colors.green : null,
              ),
              label: Text(
                _isCalibrating
                    ? 'Calibrating...'
                    : _isCalibrated
                    ? 'Recalibrate'
                    : 'Calibrate Device',
              ),
              style: ElevatedButton.styleFrom(
                backgroundColor: _isCalibrated ? Colors.green.shade100 : null,
                foregroundColor: _isCalibrated ? Colors.green.shade800 : null,
              ),
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                const Text(
                  'Step Length:',
                  style: TextStyle(fontWeight: FontWeight.w500),
                ),
                const SizedBox(width: 16),
                Expanded(
                  child: Slider(
                    value: _stepLength,
                    min: 0.5,
                    max: 1.0,
                    divisions: 50,
                    label: '${_stepLength.toStringAsFixed(2)} m',
                    onChanged: (value) {
                      _updateStepLength(value);
                    },
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildDebugCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Debug Information',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 16),
            Text(
              'Acceleration: ${_acceleration.map((a) => a.toStringAsFixed(2)).join(', ')}',
            ),
            Text(
              'Gyroscope: ${_gyroscope.map((g) => g.toStringAsFixed(2)).join(', ')}',
            ),
            Text('BLE Beacons: ${_detectedBeacons.length}'),
            if (_detectedBeacons.isNotEmpty) ...[
              const SizedBox(height: 8),
              ..._detectedBeacons.map(
                (beacon) => Text(
                  '${beacon.id}: ${beacon.rssi.toStringAsFixed(1)} dBm, ${beacon.distance.toStringAsFixed(1)} m',
                ),
              ),
            ],
            const SizedBox(height: 16),
            const Text(
              'Sensor Status:',
              style: TextStyle(fontWeight: FontWeight.bold),
            ),
            if (_sensorsChecked) ...[
              if (_unavailableSensors.isEmpty)
                const Text(
                  '✅ All sensors available - Maximum accuracy!',
                  style: TextStyle(
                    color: Colors.green,
                    fontWeight: FontWeight.w500,
                  ),
                )
              else ...[
                const SizedBox(height: 4),
                ..._unavailableSensors.map(
                  (sensor) => Padding(
                    padding: const EdgeInsets.only(left: 8.0, bottom: 2.0),
                    child: Text(
                      sensor,
                      style: TextStyle(
                        fontSize: 13,
                        color:
                            sensor.startsWith('❌') ? Colors.red : Colors.orange,
                      ),
                    ),
                  ),
                ),
                const SizedBox(height: 8),
                if (_unavailableSensors.any((s) => s.startsWith('❌')))
                  const Text(
                    '⚠️ Critical sensors missing may severely impact navigation accuracy',
                    style: TextStyle(
                      fontSize: 12,
                      color: Colors.red,
                      fontStyle: FontStyle.italic,
                    ),
                  )
                else
                  const Text(
                    'ℹ️ Missing sensors have workarounds - reduced accuracy expected',
                    style: TextStyle(
                      fontSize: 12,
                      color: Colors.orange,
                      fontStyle: FontStyle.italic,
                    ),
                  ),
              ],
            ] else
              const Text(
                '🔍 Checking sensor availability...',
                style: TextStyle(color: Colors.grey),
              ),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    _sensorSubscription?.cancel();
    _bleSubscription?.cancel();
    _stepCountStream?.cancel();
    _pedestrianStatusStream?.cancel();
    _accelerometerStream?.cancel();
    _compassStream?.cancel();
    _bleService.dispose();
    super.dispose();
  }
}
