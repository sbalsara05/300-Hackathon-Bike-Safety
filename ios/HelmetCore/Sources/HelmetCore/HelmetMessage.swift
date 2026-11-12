import Foundation

/// What the phone writes to the helmet 10-20 times a second (firmware/helmet/feather_hud_ble).
/// {"lvl":3,"dir":"left","ttc":1.2,"tgt":[[x,y,width,length,"c"], ...]}. Every write is also the heartbeat.
public struct HelmetMessage: Sendable {
    public static let serviceUUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
    public static let writeUUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
    public static let deviceNamePrefix = "HelmetHU"
    public static let maxBytes = 244
    public static let maxTargets = 8

    public struct Target: Sendable {
        public var x, y, width, length: Double
        public var type: Character
        public init(x: Double, y: Double, width: Double, length: Double, type: Character) {
            self.x = x; self.y = y; self.width = width; self.length = length; self.type = type
        }
    }

    public var level: WarningLevel
    public var dir: Direction
    public var ttc: Double?
    public var targets: [Target]

    public init(level: WarningLevel, dir: Direction = .none, ttc: Double? = nil, targets: [Target] = []) {
        self.level = level; self.dir = dir; self.ttc = ttc; self.targets = targets
    }

    /// JSON under 244 bytes; drops the farthest targets if it would not fit.
    public func encoded() -> Data {
        func f(_ v: Double) -> String { String(format: "%.1f", v) }
        var list = Array(targets.sorted { hypot($0.x, $0.y) < hypot($1.x, $1.y) }.prefix(Self.maxTargets))
        while true {
            let tgt = list.map { "[\(f($0.x)),\(f($0.y)),\(f($0.width)),\(f($0.length)),\"\($0.type)\"]" }.joined(separator: ",")
            let ttcPart = ttc.map { ",\"ttc\":\(f(max(0, min($0, 99))))" } ?? ""
            let s = "{\"lvl\":\(level.rawValue),\"dir\":\"\(dir.rawValue)\"\(ttcPart),\"tgt\":[\(tgt)]}"
            if s.utf8.count <= Self.maxBytes || list.isEmpty { return Data(s.utf8) }
            list.removeLast()
        }
    }
}
