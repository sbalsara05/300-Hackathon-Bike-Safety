import Foundation

/// A ride file (ride-log v1, see sim/README.md): one header line, then one frame per line.
public struct RideLog: Sendable {
    public struct Expect: Codable, Sendable {
        public var peak_min: Int?
        public var peak_max: Int?
        public var dir: String?
        public var danger_lead_s_min: Double?
        public var caution_lead_s_min: Double?
        public var pass_gap_m: Double?
        public var sensor_flag: String?
        public var basis: String?
    }
    public struct TruthSummary: Codable, Sendable { public var contact_t: Double? }
    public struct Meta: Codable, Sendable {
        public var name: String
        public var description: String
        public var dt: Double
        public var rider_speed_mps: Double
        public var expect: Expect
        public var truth_summary: TruthSummary
    }
    public struct TruthObject: Codable, Sendable {
        public var id: Int
        public var cls: String
        public var x, y, w, l, yaw: Double
    }
    public struct Frame: Codable, Sendable {
        public var t: Double
        public var lidar: String?
        public var rear: [Detection]?
        public var front: [Detection]?
        public var phone: PhoneState
        public var truth: [TruthObject]?

        public var input: FrameInput {
            let scan = lidar.flatMap { Data(base64Encoded: $0) }.flatMap { LidarScan(packet: $0) }
            return FrameInput(t: t, lidar: scan, rear: rear, front: front, phone: phone)
        }
    }

    public var meta: Meta
    public var frames: [Frame]

    public init(contentsOf url: URL) throws {
        let text = try String(contentsOf: url, encoding: .utf8)
        let lines = text.split(separator: "\n", omittingEmptySubsequences: true)
        guard let first = lines.first else { throw CocoaError(.fileReadCorruptFile) }
        let dec = JSONDecoder()
        meta = try dec.decode(Meta.self, from: Data(first.utf8))
        frames = try lines.dropFirst().map { try dec.decode(Frame.self, from: Data($0.utf8)) }
    }
}
