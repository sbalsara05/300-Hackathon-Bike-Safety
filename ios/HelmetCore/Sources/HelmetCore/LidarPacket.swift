import Foundation

/// One LiDAR revolution as sent by firmware/bike/qtpy_lidar_udp:
/// "HL" | version (1) | flags (1) | seq (uint16 LE) | t_ms (uint32 LE) | 360 x uint16 LE range in mm.
public struct LidarScan: Sendable {
    public static let packetSize = 730
    public var seq: UInt16
    public var tMs: UInt32
    public var rangesMm: [UInt16]     // index = degrees clockwise from the LiDAR's 0 mark, 0 = no return

    public init(seq: UInt16, tMs: UInt32, rangesMm: [UInt16]) { self.seq = seq; self.tMs = tMs; self.rangesMm = rangesMm }

    public init?(packet: Data) {
        guard packet.count == Self.packetSize else { return nil }
        let b = [UInt8](packet)
        guard b[0] == 0x48, b[1] == 0x4C, b[2] == 1 else { return nil }
        seq = UInt16(b[4]) | UInt16(b[5]) << 8
        tMs = UInt32(b[6]) | UInt32(b[7]) << 8 | UInt32(b[8]) << 16 | UInt32(b[9]) << 24
        var r = [UInt16](repeating: 0, count: 360)
        for i in 0..<360 { r[i] = UInt16(b[10 + 2 * i]) | UInt16(b[11 + 2 * i]) << 8 }
        rangesMm = r
    }
}
