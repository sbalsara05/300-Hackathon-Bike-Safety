import Foundation

public enum WarningLevel: Int, Comparable, Codable, Sendable {
    case clear = 0, approaching, caution, danger
    public static func < (a: WarningLevel, b: WarningLevel) -> Bool { a.rawValue < b.rawValue }
    public var name: String { ["CLEAR", "APPROACHING", "CAUTION", "DANGER"][rawValue] }
}

public enum Direction: String, Codable, Sendable {
    case left, right, front, rear
    case none = ""
}

/// One camera detection as a YOLO model reports it. `box` = x, y, width, height in pixels.
public struct Detection: Codable, Sendable {
    public var cls: String
    public var conf: Double
    public var box: [Double]
    public init(cls: String, conf: Double, box: [Double]) { self.cls = cls; self.conf = conf; self.box = box }
}

public struct PhoneState: Codable, Sendable {
    public var speed: Double      // m/s
    public var heading: Double    // degrees
    public var lean: Double       // degrees, positive = leaning right
    public var lat: Double
    public var lon: Double
    public init(speed: Double = 0, heading: Double = 0, lean: Double = 0, lat: Double = 0, lon: Double = 0) {
        self.speed = speed; self.heading = heading; self.lean = lean; self.lat = lat; self.lon = lon
    }
}

public struct CameraSpec: Sendable {
    public var width: Double
    public var height: Double
    public var hfovDeg: Double
    public init(width: Double, height: Double, hfovDeg: Double) { self.width = width; self.height = height; self.hfovDeg = hfovDeg }
    public var focalPx: Double { (width / 2) / tan(hfovDeg * .pi / 360) }
}

/// A point in the rider frame: rider at (0, 0), x = right, y = forward, metres.
public struct Point: Sendable {
    public var x: Double
    public var y: Double
    public init(_ x: Double, _ y: Double) { self.x = x; self.y = y }
    public var range: Double { (x * x + y * y).squareRoot() }
}

/// Every threshold in one place. Values marked "design" come from the design doc; the rest are
/// choices made for the synthetic rides and should be re-measured on the real bike.
public struct Config: Sendable {
    public init() {}
    // LiDAR mounting
    public var lidarZeroOffsetDeg = 0.0        // added to every bin angle so 0 deg = bike forward
    public var lidarHeightM = 1.0              // design (guess, to measure)
    // Scan cleaning
    public var minRangeM = 0.5                 // design
    public var bodyMaskHalfAngleDeg = 30.0     // design guess is 20; widened to cover the simulated legs
    public var bodyMaskRangeM = 1.3            // design (guess, to measure)
    public var leanStartDeg = 3.0              // below this the scan plane clears the road inside 12 m
    public var leanGroundFactor = 0.75         // drop points beyond this fraction of the predicted road hit
    // Grouping and tracking
    public var clusterGapM = 0.5               // design
    public var clusterMaxBinGap = 3
    public var clusterMinPoints = 2
    public var trackGateM = 1.5
    public var trackMaxMisses = 3
    public var confirmHits = 3
    // Warning rules
    public var riderSpaceM = 0.4               // contact distance
    public var corridorHalfWidthM = 0.6        // an object closer than this to the rider's line is "in the path"
    public var pathHalfWidthM = 0.5
    public var dangerS = 1.5                   // design
    public var cautionS = 3.0                  // design
    public var closePassM = 1.5                // design
    public var doorZoneM = 1.5                 // design
    public var approachingSpeed = 2.0          // design, m/s
    public var approachingRangeM = 30.0        // design
    public var cameraDangerMinS = 2.0          // design hard minimum
    public var cameraCautionMinS = 3.5         // design hard minimum
    public var heavyVehicleFactor = 1.3        // design
    public var holdS = 0.5                     // design
    public var debounceFrames = 2              // design: 2-3 frames
    public var movingSpeed = 0.7               // m/s over the ground to count as moving
    // Camera
    public var rearCamera = CameraSpec(width: 320, height: 240, hfovDeg: 62)
    public var frontCamera = CameraSpec(width: 640, height: 480, hfovDeg: 68)
    public var ttcWindow = 14                  // design (Halo)
    public var ttcMinGrowth = 0.06             // design (Halo)
    public var ttcMinGrowFraction = 0.6        // design (Halo)
    // Sensor loss
    public var lidarLostS = 0.5
    public var cameraLostS = 1.0
}
