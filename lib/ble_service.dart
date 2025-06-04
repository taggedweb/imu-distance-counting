import 'dart:async';
import 'dart:math';

import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';

/// Represents a BLE beacon with position and signal strength information
class BLEBeacon {
  final String id;
  final String name;
  final double x;
  final double y;
  final double rssi;
  final double distance;
  final DateTime lastSeen;

  BLEBeacon({
    required this.id,
    required this.name,
    required this.x,
    required this.y,
    required this.rssi,
    required this.distance,
    required this.lastSeen,
  });

  @override
  String toString() {
    return 'BLEBeacon(id: $id, name: $name, x: $x, y: $y, rssi: $rssi, distance: ${distance.toStringAsFixed(2)}m)';
  }
}

/// BLE service for indoor positioning using Bluetooth beacons
class BLEService {
  // Stream controllers
  final StreamController<Map<String, double>?> _positionController =
      StreamController<Map<String, double>?>.broadcast();
  final StreamController<List<BLEBeacon>> _beaconController =
      StreamController<List<BLEBeacon>>.broadcast();

  // Detected beacons storage
  final Map<String, BLEBeacon> _detectedBeacons = {};
  final List<BLEBeacon> _knownBeacons = [];

  // Scanning state
  bool _isScanning = false;
  Timer? _scanTimer;
  Timer? _positionTimer;

  // Position calculation parameters
  static const double _minBeaconsForTrilateration = 3;
  static const double _beaconTimeout = 10.0; // seconds
  static const double _rssiToDistanceConstant = -59.0; // RSSI at 1m
  static const double _pathLossExponent = 2.0;

  // Bluetooth instance
  FlutterBluetoothSerial? _bluetooth;

  /// Constructor
  BLEService() {
    _initializeBluetooth();
    _setupKnownBeacons();
    _startPositionCalculation();
  }

  /// Initialize Bluetooth
  Future<void> _initializeBluetooth() async {
    try {
      _bluetooth = FlutterBluetoothSerial.instance;

      // Check if Bluetooth is available
      bool? isAvailable = await _bluetooth?.isAvailable;
      if (isAvailable != true) {
        print('Bluetooth is not available on this device');
        return;
      }

      // Check if Bluetooth is enabled
      bool? isEnabled = await _bluetooth?.isEnabled;
      if (isEnabled != true) {
        print('Bluetooth is not enabled');
        // You might want to request enabling Bluetooth here
      }
    } catch (e) {
      print('Error initializing Bluetooth: $e');
    }
  }

  /// Setup known beacon positions (in a real app, this would come from a database)
  void _setupKnownBeacons() {
    _knownBeacons.addAll([
      BLEBeacon(
        id: 'beacon_001',
        name: 'Entrance Beacon',
        x: 0.0,
        y: 0.0,
        rssi: 0.0,
        distance: 0.0,
        lastSeen: DateTime.now(),
      ),
      BLEBeacon(
        id: 'beacon_002',
        name: 'Corner Beacon',
        x: 10.0,
        y: 0.0,
        rssi: 0.0,
        distance: 0.0,
        lastSeen: DateTime.now(),
      ),
      BLEBeacon(
        id: 'beacon_003',
        name: 'Center Beacon',
        x: 5.0,
        y: 8.0,
        rssi: 0.0,
        distance: 0.0,
        lastSeen: DateTime.now(),
      ),
      BLEBeacon(
        id: 'beacon_004',
        name: 'Exit Beacon',
        x: 12.0,
        y: 8.0,
        rssi: 0.0,
        distance: 0.0,
        lastSeen: DateTime.now(),
      ),
    ]);
  }

  /// Start scanning for BLE beacons
  Future<void> startScanning() async {
    if (_isScanning) {
      return;
    }

    _isScanning = true;
    print('Starting BLE beacon scanning...');

    try {
      // Start discovery
      _bluetooth?.startDiscovery();

      // Listen to discovered devices

      // Simulate beacon detection for demo purposes
      _startSimulatedBeaconDetection();
    } catch (e) {
      print('Error starting BLE scanning: $e');
      _isScanning = false;
    }
  }

  /// Stop scanning for BLE beacons
  Future<void> stopScanning() async {
    if (!_isScanning) {
      return;
    }

    _isScanning = false;
    _scanTimer?.cancel();

    try {
      await _bluetooth?.cancelDiscovery();
      print('Stopped BLE beacon scanning');
    } catch (e) {
      print('Error stopping BLE scanning: $e');
    }
  }

  /// Handle discovered Bluetooth device
  void _handleDiscoveredDevice(BluetoothDiscoveryResult result) {
    if (!_isScanning) return;

    BluetoothDevice device = result.device;
    int? rssi = result.rssi;

    // Check if this is a known beacon
    BLEBeacon? knownBeacon = _findKnownBeacon(device.address ?? device.name);
    if (knownBeacon != null) {
      double distance = _calculateDistance(rssi.toDouble());

      BLEBeacon updatedBeacon = BLEBeacon(
        id: knownBeacon.id,
        name: knownBeacon.name,
        x: knownBeacon.x,
        y: knownBeacon.y,
        rssi: rssi.toDouble(),
        distance: distance,
        lastSeen: DateTime.now(),
      );

      _detectedBeacons[knownBeacon.id] = updatedBeacon;
      print('Updated beacon: ${updatedBeacon.toString()}');
    }
  }

  /// Find a known beacon by device address or name
  BLEBeacon? _findKnownBeacon(String? identifier) {
    if (identifier == null) return null;

    for (BLEBeacon beacon in _knownBeacons) {
      if (beacon.id.contains(identifier) || beacon.name.contains(identifier)) {
        return beacon;
      }
    }
    return null;
  }

  /// Calculate distance from RSSI value
  double _calculateDistance(double rssi) {
    if (rssi >= 0) {
      return 0.0;
    }

    // Distance calculation using path loss model
    // Distance = 10^((Tx Power - RSSI) / (10 * N))
    // Where Tx Power is typically -59 dBm at 1 meter
    double distance =
        pow(
          10,
          (_rssiToDistanceConstant - rssi) / (10 * _pathLossExponent),
        ).toDouble();

    // Clamp distance to reasonable values
    return distance.clamp(0.1, 100.0);
  }

  /// Start simulated beacon detection for demo purposes
  void _startSimulatedBeaconDetection() {
    _scanTimer = Timer.periodic(const Duration(seconds: 2), (timer) {
      if (!_isScanning) {
        timer.cancel();
        return;
      }

      // Simulate detecting beacons with varying RSSI
      _simulateBeaconDetection();
    });
  }

  /// Simulate beacon detection with random RSSI values
  void _simulateBeaconDetection() {
    Random random = Random();

    for (BLEBeacon knownBeacon in _knownBeacons) {
      // Randomly decide if beacon is detected (90% chance)
      if (random.nextDouble() < 0.9) {
        // Simulate RSSI between -30 and -90 dBm
        double rssi = -30 - random.nextDouble() * 60;
        double distance = _calculateDistance(rssi);

        BLEBeacon detectedBeacon = BLEBeacon(
          id: knownBeacon.id,
          name: knownBeacon.name,
          x: knownBeacon.x,
          y: knownBeacon.y,
          rssi: rssi,
          distance: distance,
          lastSeen: DateTime.now(),
        );

        _detectedBeacons[knownBeacon.id] = detectedBeacon;
      }
    }

    // Emit updated beacon list
    _beaconController.add(getDetectedBeacons());
  }

  /// Start position calculation timer
  void _startPositionCalculation() {
    _positionTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      _calculatePosition();
    });
  }

  /// Calculate position using trilateration
  void _calculatePosition() {
    // Remove old beacons
    _removeOldBeacons();

    List<BLEBeacon> validBeacons = _getValidBeacons();

    if (validBeacons.length >= _minBeaconsForTrilateration) {
      Map<String, double>? position = _performTrilateration(validBeacons);
      if (position != null) {
        // Add accuracy estimate based on beacon quality
        position['accuracy'] = _calculateAccuracy(validBeacons);
        _positionController.add(position);
        return;
      }
    }

    // No position available
    _positionController.add(null);
  }

  /// Remove beacons that haven't been seen recently
  void _removeOldBeacons() {}

  /// Get valid beacons for position calculation
  List<BLEBeacon> _getValidBeacons() {
    return _detectedBeacons.values
        .where((beacon) => beacon.distance > 0.1 && beacon.distance < 50.0)
        .toList()
      ..sort((a, b) => a.distance.compareTo(b.distance)); // Sort by distance
  }

  /// Perform trilateration to calculate position
  Map<String, double>? _performTrilateration(List<BLEBeacon> beacons) {
    if (beacons.length < 3) return null;

    // Use the three closest beacons for trilateration
    List<BLEBeacon> trilaterationBeacons = beacons.take(3).toList();

    try {
      // Trilateration algorithm implementation
      BLEBeacon b1 = trilaterationBeacons[0];
      BLEBeacon b2 = trilaterationBeacons[1];
      BLEBeacon b3 = trilaterationBeacons[2];

      double x1 = b1.x, y1 = b1.y, r1 = b1.distance;
      double x2 = b2.x, y2 = b2.y, r2 = b2.distance;
      double x3 = b3.x, y3 = b3.y, r3 = b3.distance;

      // Calculate position using trilateration formulas
      double A = 2 * (x2 - x1);
      double B = 2 * (y2 - y1);

      double D = 2 * (x3 - x2);
      double E = 2 * (y3 - y2);

      double denominator = A * E - B * D;
      if (denominator.abs() < 1e-10) {
        // Beacons are collinear, can't triangulate
        return null;
      }

      return {'x': 0, 'y': 0};
    } catch (e) {
      print('Error in trilateration: $e');
      return null;
    }
  }

  /// Calculate position accuracy based on beacon quality
  double _calculateAccuracy(List<BLEBeacon> beacons) {
    if (beacons.isEmpty) return 10.0; // Default high uncertainty

    // Calculate accuracy based on:
    // 1. Number of beacons
    // 2. Average distance
    // 3. RSSI quality

    double avgDistance =
        beacons.map((b) => b.distance).reduce((a, b) => a + b) / beacons.length;
    double avgRssi =
        beacons.map((b) => b.rssi.abs()).reduce((a, b) => a + b) /
        beacons.length;

    // Better accuracy with more beacons, closer distance, and stronger signal
    double baseAccuracy = avgDistance * 0.3; // 30% of average distance
    double rssiPenalty = (avgRssi - 40) * 0.05; // Penalty for weak signals
    double beaconBonus = (beacons.length - 3) * -0.5; // Bonus for more beacons

    double accuracy = baseAccuracy + rssiPenalty + beaconBonus;
    return accuracy.clamp(0.5, 10.0); // Clamp between 0.5m and 10m
  }

  /// Get list of currently detected beacons
  List<BLEBeacon> getDetectedBeacons() {
    return _detectedBeacons.values.toList()
      ..sort((a, b) => a.distance.compareTo(b.distance));
  }

  /// Get position stream
  Stream<Map<String, double>?> get positionStream => _positionController.stream;

  /// Get beacon stream
  Stream<List<BLEBeacon>> get beaconStream => _beaconController.stream;

  /// Check if currently scanning
  bool get isScanning => _isScanning;

  /// Get number of detected beacons
  int get beaconCount => _detectedBeacons.length;

  /// Dispose resources
  void dispose() {
    _scanTimer?.cancel();
    _positionTimer?.cancel();
    _positionController.close();
    _beaconController.close();

    if (_isScanning) {
      stopScanning();
    }
  }
}
