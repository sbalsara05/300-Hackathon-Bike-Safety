import Foundation

/// An object followed from scan to scan by its edge nearest the rider.
public final class LidarTrack {
    public internal(set) var id: Int
    public internal(set) var label: String?
    public internal(set) var cluster: Cluster
    public internal(set) var hits = 1
    var misses = 0
    var lastT: Double
    var vNear = Point(0, 0)
    /// Speed at which the gap along the road is shrinking (m/s); meaningful when not alongside.
    public internal(set) var closingY = 0.0
    /// Speed at which the sideways gap is shrinking (m/s).
    public internal(set) var closingX = 0.0
    var worldVy: Double?
    /// true = moving over the ground, false = parked/static, nil = not known yet.
    public internal(set) var moving: Bool?
    public internal(set) var width = 0.0, length = 0.0       // remembered size
    var minGapAlongside: Double?
    var areaHist: [(t: Double, lnArea: Double)] = []
    var camTTC: Double?
    var camT = -Double.infinity
    var adoptedCameraID = false
    var recentRaw: [WarningLevel] = []
    var heldLevel = WarningLevel.clear, heldT = 0.0
    private var rawY: [Double] = [], rawX: [Double] = []

    static func median(_ v: [Double]) -> Double {
        let s = v.sorted()
        return s.isEmpty ? 0 : (s.count % 2 == 1 ? s[s.count / 2] : (s[s.count / 2 - 1] + s[s.count / 2]) / 2)
    }

    init(id: Int, cluster: Cluster, t: Double) {
        self.id = id; self.cluster = cluster; lastT = t
        width = cluster.xMax - cluster.xMin; length = cluster.yMax - cluster.yMin
    }

    func update(_ c: Cluster, t: Double, speed: Double, cfg: Config) {
        let dt = max(t - lastT, 0.02)
        let prev = cluster
        let raw = Point((c.nearest.x - prev.nearest.x) / dt, (c.nearest.y - prev.nearest.y) / dt)
        vNear = hits == 1 ? raw : Point(0.6 * vNear.x + 0.4 * raw.x, 0.6 * vNear.y + 0.4 * raw.y)
        // Median of the last few frame-to-frame changes: the rider's legs hide parts of an object for a
        // frame at a time, which makes single-frame jumps that must not read as motion.
        if !c.alongside && !prev.alongside && c.ahead == prev.ahead {
            rawY.append((prev.yGap - c.yGap) / dt)
            if rawY.count > 5 { rawY.removeFirst() }
            closingY = Self.median(rawY)
        } else {
            rawY.removeAll()
        }
        rawX.append((prev.latGap - c.latGap) / dt)
        if rawX.count > 5 { rawX.removeFirst() }
        closingX = rawX.count >= 3 ? Self.median(rawX) : 0
        hits += 1; misses = 0; lastT = t; cluster = c
        width = max(width, c.xMax - c.xMin); length = max(length, c.yMax - c.yMin)
        if !c.alongside && hits >= cfg.confirmHits {
            let w = c.ahead ? speed - closingY : speed + closingY
            worldVy = worldVy.map { 0.7 * $0 + 0.3 * w } ?? w
            moving = abs(worldVy!) > cfg.movingSpeed
        }
    }
}

/// A vehicle only a camera can see so far.
public final class CameraTrack {
    public internal(set) var id: Int
    public internal(set) var cls: String
    public let facing: Double            // +1 front camera, -1 rear camera
    public internal(set) var x = 0.0, y = 0.0, dist = 0.0, gap = 0.0
    public internal(set) var ttc: Double?
    var hist: [(t: Double, lnArea: Double)] = []
    var lastT: Double
    var hits = 1
    var recentRaw: [WarningLevel] = []
    var heldLevel = WarningLevel.clear, heldT = 0.0

    init(id: Int, cls: String, facing: Double, t: Double) { self.id = id; self.cls = cls; self.facing = facing; lastT = t }
}

enum VehicleSize {
    /// Typical width and height in metres, used to turn a box size into a distance.
    static func of(_ cls: String) -> (w: Double, h: Double)? {
        switch cls {
        case "car": return (1.8, 1.5)
        case "truck": return (2.5, 3.2)
        case "bus": return (2.55, 3.1)
        case "bicycle", "motorcycle": return (0.6, 1.7)
        case "person": return (0.5, 1.7)
        default: return nil
        }
    }
}

/// A detection turned into rider-frame geometry.
struct CameraFix {
    var cls: String
    var x: Double, y: Double, dist: Double, gap: Double
    var bearingDeg: Double
    var lnArea: Double
    var clipped: Bool

    init?(_ d: Detection, spec: CameraSpec, facing: Double) {
        guard d.box.count == 4, d.conf >= 0.3, let size = VehicleSize.of(d.cls) else { return nil }
        let (bx, by, bw, bh) = (d.box[0], d.box[1], d.box[2], d.box[3])
        guard bw > 2, bh > 2 else { return nil }
        let f = spec.focalPx
        clipped = bx <= 2 || bx + bw >= spec.width - 2
        let vClipped = by <= 2 || by + bh >= spec.height - 2
        let zw = f * size.w / bw, zh = f * size.h / bh
        // Seen at an angle the box includes the vehicle's side, so width alone reads too close.
        dist = vClipped ? zw : max(zw, zh)
        let uL = (bx - spec.width / 2) / f, uR = (bx + bw - spec.width / 2) / f
        let straddles = uL < 0 && uR > 0
        let outer = abs(uL) > abs(uR) ? uL : uR                 // the vehicle's near outer corner
        gap = straddles ? 0 : max(0, abs(outer) * dist - size.w)
        let xCam = straddles ? (uL + uR) / 2 * dist : (outer > 0 ? 1 : -1) * (gap + size.w / 2)
        x = facing > 0 ? xCam : -xCam                           // the rear image is not mirrored
        y = facing * dist
        bearingDeg = atan2(x, y) * 180 / .pi
        lnArea = log(bw * bh)
        cls = d.cls
    }
}

enum TTC {
    /// Halo's method: time to contact = 2 / growth rate of log(box area), fitted over the last frames.
    static func estimate(_ hist: [(t: Double, lnArea: Double)], cfg: Config) -> Double? {
        let h = Array(hist.suffix(cfg.ttcWindow))
        guard h.count >= 6, let first = h.first, let last = h.last, last.t - first.t >= 0.4 else { return nil }
        let n = Double(h.count)
        let mt = h.reduce(0) { $0 + $1.t } / n, ma = h.reduce(0) { $0 + $1.lnArea } / n
        var num = 0.0, den = 0.0, ups = 0
        for (i, s) in h.enumerated() {
            num += (s.t - mt) * (s.lnArea - ma); den += (s.t - mt) * (s.t - mt)
            if i > 0 && s.lnArea > h[i - 1].lnArea { ups += 1 }
        }
        guard den > 0 else { return nil }
        let slope = num / den
        let growth = exp(slope * (last.t - first.t)) - 1
        guard slope > 0, growth >= cfg.ttcMinGrowth, Double(ups) / (n - 1) >= cfg.ttcMinGrowFraction else { return nil }
        // The fit describes the middle of the window; move it forward to now.
        return max(0, 2 / slope - (last.t - first.t) / 2)
    }
}
