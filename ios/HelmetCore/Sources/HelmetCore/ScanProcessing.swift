import Foundation

public struct ScanPoint: Sendable {
    public var index: Int
    public var range: Double
    public var x: Double
    public var y: Double
}

/// A group of neighbouring LiDAR points: one object, or the visible part of one.
public struct Cluster: Sendable {
    public var points: [ScanPoint]
    public var xMin = Double.infinity, xMax = -Double.infinity
    public var yMin = Double.infinity, yMax = -Double.infinity
    public var nearest: ScanPoint
    public var centroid: Point

    init(points: [ScanPoint]) {
        self.points = points
        nearest = points[0]
        var sx = 0.0, sy = 0.0
        for p in points {
            xMin = min(xMin, p.x); xMax = max(xMax, p.x)
            yMin = min(yMin, p.y); yMax = max(yMax, p.y)
            if p.range < nearest.range { nearest = p }
            sx += p.x; sy += p.y
        }
        centroid = Point(sx / Double(points.count), sy / Double(points.count))
    }

    /// Sideways distance from the rider's line to the object (0 if it straddles the line).
    public var latGap: Double { xMin <= 0 && xMax >= 0 ? 0 : min(abs(xMin), abs(xMax)) }
    /// True while some part of the object is level with the rider.
    public var alongside: Bool { yMin <= 0.3 && yMax >= -0.3 }
    public var ahead: Bool { yMin > 0.3 }
    /// Distance along the road to the object (0 while alongside).
    public var yGap: Double { alongside ? 0 : (ahead ? yMin : -yMax) }
}

public enum ScanProcessing {
    /// Drop points on the rider and bike, and points where the leaning scan plane hits the road.
    public static func clean(_ scan: LidarScan, leanDeg: Double, cfg: Config) -> [ScanPoint] {
        var out: [ScanPoint] = []
        out.reserveCapacity(360)
        let leanSin = sin(leanDeg * .pi / 180)
        for i in 0..<360 where scan.rangesMm[i] != 0 {
            let r = Double(scan.rangesMm[i]) / 1000
            var deg = (Double(i) + 0.5 + cfg.lidarZeroOffsetDeg).truncatingRemainder(dividingBy: 360)
            if deg < 0 { deg += 360 }
            let th = deg * .pi / 180
            if r < cfg.minRangeM { continue }
            let fromForward = deg > 180 ? 360 - deg : deg
            if fromForward <= cfg.bodyMaskHalfAngleDeg && r < cfg.bodyMaskRangeM { continue }
            if abs(leanDeg) > cfg.leanStartDeg {
                let s = sin(th) * leanSin                      // > 0 on the side the bike leans toward
                if s > 0 && r >= cfg.leanGroundFactor * cfg.lidarHeightM / s { continue }
            }
            out.append(ScanPoint(index: i, range: r, x: r * sin(th), y: r * cos(th)))
        }
        return out
    }

    /// Neighbouring points closer than `clusterGapM` form one object.
    public static func cluster(_ points: [ScanPoint], cfg: Config) -> [Cluster] {
        guard !points.isEmpty else { return [] }
        func linked(_ a: ScanPoint, _ b: ScanPoint, wrap: Bool = false) -> Bool {
            let binGap = wrap ? b.index + 360 - a.index : b.index - a.index
            return binGap <= cfg.clusterMaxBinGap && hypot(a.x - b.x, a.y - b.y) <= cfg.clusterGapM
        }
        var groups: [[ScanPoint]] = [[points[0]]]
        for p in points.dropFirst() {
            if linked(groups[groups.count - 1].last!, p) { groups[groups.count - 1].append(p) } else { groups.append([p]) }
        }
        if groups.count > 1, linked(groups[groups.count - 1].last!, groups[0][0], wrap: true) {
            groups[0] = groups.removeLast() + groups[0]
        }
        return groups.filter { $0.count >= cfg.clusterMinPoints }.map(Cluster.init)
    }
}
