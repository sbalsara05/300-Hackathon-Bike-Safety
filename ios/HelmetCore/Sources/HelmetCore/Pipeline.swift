import Foundation

/// Everything that arrived in one 0.1 s tick. nil = that sensor sent nothing this tick.
public struct FrameInput: Sendable {
    public var t: Double
    public var lidar: LidarScan?
    public var rear: [Detection]?
    public var front: [Detection]?
    public var phone: PhoneState
    public init(t: Double, lidar: LidarScan?, rear: [Detection]?, front: [Detection]?, phone: PhoneState) {
        self.t = t; self.lidar = lidar; self.rear = rear; self.front = front; self.phone = phone
    }
}

/// One tracked object, for drawing.
public struct TrackView: Sendable, Identifiable {
    public var id: Int
    public var label: String?
    public var x, y, width, length: Double
    public var level: WarningLevel
    public var cameraOnly: Bool
}

public struct PassEvent: Sendable, Identifiable {
    public var id: Int
    public var t: Double
    public var gapM: Double
    public var side: Direction
    public var label: String?
}

public struct PipelineOutput: Sendable {
    public var t: Double
    public var level: WarningLevel
    public var dir: Direction
    public var ttc: Double?
    public var message: HelmetMessage
    public var points: [ScanPoint]
    public var tracks: [TrackView]
    public var lidarLost: Bool
    public var rearCameraLost: Bool
    public var newPasses: [PassEvent]
}

/// Sensors in, warning out. The same code runs on live data and on ride files.
public final class Pipeline {
    public var cfg: Config
    public private(set) var passes: [PassEvent] = []

    private var lidarTracks: [LidarTrack] = []
    private var cameraTracks: [CameraTrack] = []
    private var nextID = 1
    private var startT: Double?
    private var lastLidarT: Double?
    private var lastRearT: Double?
    private var shown = WarningLevel.clear
    private var shownDir = Direction.none
    private var shownTTC: Double?
    private var lastAtShown = 0.0
    private var lastPoints: [ScanPoint] = []

    public init(cfg: Config = Config()) { self.cfg = cfg }

    public func reset() {
        lidarTracks = []; cameraTracks = []; passes = []; nextID = 1
        startT = nil; lastLidarT = nil; lastRearT = nil
        shown = .clear; shownDir = .none; shownTTC = nil; lastAtShown = 0; lastPoints = []
    }

    public func step(_ input: FrameInput) -> PipelineOutput {
        let t = input.t, speed = input.phone.speed
        if startT == nil { startT = t }
        var newPasses: [PassEvent] = []

        // 1. LiDAR: clean, group, track
        if let scan = input.lidar {
            lastLidarT = t
            lastPoints = ScanProcessing.clean(scan, leanDeg: input.phone.lean, cfg: cfg)
            updateLidarTracks(ScanProcessing.cluster(lastPoints, cfg: cfg), t: t, speed: speed, passes: &newPasses)
        }
        for tr in lidarTracks where t - tr.lastT > 0.35 { finishPass(tr, t: t, into: &newPasses) }
        lidarTracks.removeAll { t - $0.lastT > 0.35 }
        if lastLidarT.map({ t - $0 > 0.35 }) ?? false { lastPoints = [] }

        // 2. Cameras: label LiDAR objects, track far vehicles on their own
        if let rear = input.rear { lastRearT = t; fuse(rear, spec: cfg.rearCamera, facing: -1, t: t) }
        if let front = input.front { fuse(front, spec: cfg.frontCamera, facing: 1, t: t) }
        cameraTracks.removeAll { t - $0.lastT > 1.0 }

        let lidarLost = t - (lastLidarT ?? startT!) > cfg.lidarLostS
        let rearLost = t - (lastRearT ?? startT!) > cfg.cameraLostS

        // 3. Decide: a level per object, the worst one wins
        struct Candidate { var level: WarningLevel; var ttc: Double?; var dir: Direction; var range: Double }
        var best: Candidate?
        func offer(_ c: Candidate) {
            guard let b = best else { best = c; return }
            if c.level != b.level { if c.level > b.level { best = c }; return }
            let (ct, bt) = (c.ttc ?? .infinity, b.ttc ?? .infinity)
            if ct < bt || (ct == bt && c.range < b.range) { best = c }
        }
        var views: [TrackView] = []
        for tr in lidarTracks {
            let (raw, ttc) = evaluate(tr, speed: speed, t: t)
            let level = debounce(&tr.recentRaw, raw)
            let c = tr.cluster
            if level > .clear { offer(Candidate(level: level, ttc: ttc, dir: direction(c.centroid), range: c.nearest.range)) }
            if tr.hits >= 2 {
                views.append(view(of: tr, level: level))
            }
        }
        for ct in cameraTracks {
            let (raw, ttc) = evaluate(ct, speed: speed, lidarLost: lidarLost)
            let level = debounce(&ct.recentRaw, raw)
            let size = VehicleSize.of(ct.cls) ?? (1.8, 1.5)
            let dir: Direction = ct.gap >= cfg.corridorHalfWidthM ? (ct.x < 0 ? .left : .right) : (ct.facing > 0 ? .front : .rear)
            if level > .clear { offer(Candidate(level: level, ttc: ttc, dir: dir, range: ct.dist)) }
            if ct.hits >= cfg.confirmHits {
                views.append(TrackView(id: ct.id, label: ct.cls, x: ct.x, y: ct.y, width: size.0,
                                       length: ct.cls == "car" ? 4.5 : max(size.0, 1.0), level: level, cameraOnly: true))
            }
        }

        // One vehicle can show up twice (two groups of LiDAR points, or LiDAR plus a camera estimate).
        // Keep one box per overlap: the higher warning level first, LiDAR before camera-only.
        var kept: [TrackView] = []
        for v in views.sorted(by: { ($0.level.rawValue, $0.cameraOnly ? 0 : 1) > ($1.level.rawValue, $1.cameraOnly ? 0 : 1) }) {
            let overlaps = kept.contains { k in
                let ox = min(v.x + v.width / 2, k.x + k.width / 2) - max(v.x - v.width / 2, k.x - k.width / 2)
                let oy = min(v.y + v.length / 2, k.y + k.length / 2) - max(v.y - v.length / 2, k.y - k.length / 2)
                return ox > 0 && oy > 0 && ox * oy > 0.3 * min(v.width * v.length, k.width * k.length)
            }
            if !overlaps { kept.append(v) }
        }
        views = kept

        // 4. Hold: rise at once, drop only after holdS of calm
        let raw = best?.level ?? .clear
        if raw >= shown {
            shown = raw; lastAtShown = t; shownDir = best?.dir ?? .none; shownTTC = best?.ttc
        } else if t - lastAtShown >= cfg.holdS {
            shown = raw; lastAtShown = t; shownDir = best?.dir ?? .none; shownTTC = best?.ttc
        }

        passes.append(contentsOf: newPasses)
        let targets = views.map {
            HelmetMessage.Target(x: $0.x, y: $0.y, width: $0.width, length: $0.length, type: $0.label?.first ?? "?")
        }
        let msg = HelmetMessage(level: shown, dir: shown >= .caution ? shownDir : .none,
                                ttc: shown >= .approaching ? shownTTC : nil, targets: targets)
        return PipelineOutput(t: t, level: shown, dir: shownDir, ttc: shownTTC, message: msg, points: lastPoints,
                              tracks: views, lidarLost: lidarLost, rearCameraLost: rearLost, newPasses: newPasses)
    }

    /// The LiDAR only sees the faces turned toward it, so draw the object at its remembered (or typical)
    /// size, grown away from the rider from the edges that are actually visible.
    private func view(of tr: LidarTrack, level: WarningLevel) -> TrackView {
        let c = tr.cluster
        let typical: (w: Double, l: Double) = ["car": (1.8, 4.5), "truck": (2.5, 8.0), "bus": (2.55, 12.0),
                                                "bicycle": (0.6, 1.8), "person": (0.5, 0.5)][tr.label ?? ""] ?? (0.3, 0.3)
        let w = max(tr.width, typical.w, 0.3), l = max(tr.length, typical.l, 0.3)
        var x = (c.xMin + c.xMax) / 2, y = (c.yMin + c.yMax) / 2
        if c.xMin > 0 { x = c.xMin + w / 2 } else if c.xMax < 0 { x = c.xMax - w / 2 }
        if c.ahead { y = c.yMin + l / 2 } else if !c.alongside { y = c.yMax - l / 2 }
        return TrackView(id: tr.id, label: tr.label, x: x, y: y, width: w, length: l, level: level, cameraOnly: false)
    }

    // MARK: LiDAR tracking

    private func updateLidarTracks(_ clusters: [Cluster], t: Double, speed: Double, passes: inout [PassEvent]) {
        var pairs: [(d: Double, ti: Int, ci: Int)] = []
        for (ti, tr) in lidarTracks.enumerated() {
            let dt = t - tr.lastT
            let px = tr.cluster.nearest.x + tr.vNear.x * dt, py = tr.cluster.nearest.y + tr.vNear.y * dt
            let gate = tr.hits == 1 ? cfg.trackGateM + 0.7 : cfg.trackGateM
            for (ci, c) in clusters.enumerated() {
                let d = hypot(c.nearest.x - px, c.nearest.y - py)
                if d <= gate { pairs.append((d, ti, ci)) }
            }
        }
        pairs.sort { $0.d < $1.d }
        var usedT = Set<Int>(), usedC = Set<Int>()
        for p in pairs where !usedT.contains(p.ti) && !usedC.contains(p.ci) {
            usedT.insert(p.ti); usedC.insert(p.ci)
            let tr = lidarTracks[p.ti]
            tr.update(clusters[p.ci], t: t, speed: speed, cfg: cfg)
            if tr.moving == true {
                if tr.cluster.alongside {
                    tr.minGapAlongside = min(tr.minGapAlongside ?? .infinity, tr.cluster.nearest.range)
                } else {
                    finishPass(tr, t: t, into: &passes)
                }
            }
        }
        for (ti, tr) in lidarTracks.enumerated() where !usedT.contains(ti) { tr.misses += 1 }
        for tr in lidarTracks where tr.misses > cfg.trackMaxMisses { finishPass(tr, t: t, into: &passes) }
        lidarTracks.removeAll { $0.misses > cfg.trackMaxMisses }
        for (ci, c) in clusters.enumerated() where !usedC.contains(ci) {
            lidarTracks.append(LidarTrack(id: nextID, cluster: c, t: t)); nextID += 1
        }
    }

    /// Log "car passed at 0.8 m" once an overtaking object is no longer alongside.
    private func finishPass(_ tr: LidarTrack, t: Double, into out: inout [PassEvent]) {
        guard let gap = tr.minGapAlongside else { return }
        tr.minGapAlongside = nil
        let side: Direction = tr.cluster.centroid.x < 0 ? .left : .right
        // one vehicle can break into two groups of points; keep a single pass with the smaller gap
        if let i = out.lastIndex(where: { $0.side == side && t - $0.t < 2 }) {
            out[i].gapM = min(out[i].gapM, gap); return
        }
        if let i = passes.lastIndex(where: { $0.side == side && t - $0.t < 2 }) {
            passes[i].gapM = min(passes[i].gapM, gap); return
        }
        out.append(PassEvent(id: passes.count + out.count + 1, t: t, gapM: gap, side: side, label: tr.label))
    }

    // MARK: Camera fusion

    private func fuse(_ dets: [Detection], spec: CameraSpec, facing: Double, t: Double) {
        func angleDiff(_ a: Double, _ b: Double) -> Double {
            var d = abs(a - b).truncatingRemainder(dividingBy: 360)
            if d > 180 { d = 360 - d }
            return d
        }
        for det in dets {
            guard let fix = CameraFix(det, spec: spec, facing: facing) else { continue }
            // an existing camera-only track for this detection, if any
            let camMatch = cameraTracks.filter {
                $0.facing == facing && angleDiff(atan2($0.x, $0.y) * 180 / .pi, fix.bearingDeg) < 8
                    && abs($0.dist - fix.dist) < 0.3 * fix.dist + 1.5
            }.min { abs($0.dist - fix.dist) < abs($1.dist - fix.dist) }

            // a LiDAR object in the same direction and at a similar range
            let lidarMatch = lidarTracks.filter { tr in
                let c = tr.cluster
                guard tr.hits >= 2, facing > 0 ? c.yMax > 0.5 : c.yMin < -0.5 else { return false }
                let diff = angleDiff(atan2(c.centroid.x, c.centroid.y) * 180 / .pi, fix.bearingDeg)
                if fix.clipped { return diff < 25 && c.nearest.range < 6 }
                return diff < 15 && abs(c.nearest.range - fix.dist) < max(2.5, 0.4 * fix.dist)
            }.min {
                angleDiff(atan2($0.cluster.centroid.x, $0.cluster.centroid.y) * 180 / .pi, fix.bearingDeg)
                    < angleDiff(atan2($1.cluster.centroid.x, $1.cluster.centroid.y) * 180 / .pi, fix.bearingDeg)
            }

            if let tr = lidarMatch {
                tr.label = fix.cls
                tr.camT = t
                if let cam = camMatch {                       // hand over: keep the camera track's ID and history
                    if !tr.adoptedCameraID { tr.id = cam.id; tr.adoptedCameraID = true; tr.areaHist = cam.hist }
                    cameraTracks.removeAll { $0 === cam }
                }
                if !fix.clipped {
                    tr.areaHist.append((t, fix.lnArea))
                    if tr.areaHist.count > 30 { tr.areaHist.removeFirst() }
                    tr.camTTC = TTC.estimate(tr.areaHist, cfg: cfg)
                }
            } else if let cam = camMatch {
                cam.lastT = t
                guard !fix.clipped else { continue }
                cam.hits += 1; cam.cls = fix.cls
                cam.dist = 0.5 * cam.dist + 0.5 * fix.dist
                cam.x = 0.5 * cam.x + 0.5 * fix.x; cam.y = facing * cam.dist
                cam.gap = 0.5 * cam.gap + 0.5 * fix.gap
                cam.hist.append((t, fix.lnArea))
                if cam.hist.count > 30 { cam.hist.removeFirst() }
                cam.ttc = TTC.estimate(cam.hist, cfg: cfg)
            } else if !fix.clipped {
                let cam = CameraTrack(id: nextID, cls: fix.cls, facing: facing, t: t); nextID += 1
                cam.x = fix.x; cam.y = fix.y; cam.dist = fix.dist; cam.gap = fix.gap
                cam.hist = [(t, fix.lnArea)]
                cameraTracks.append(cam)
            }
        }
    }

    // MARK: Warning rules

    private func debounce(_ recent: inout [WarningLevel], _ raw: WarningLevel) -> WarningLevel {
        recent.append(raw)
        if recent.count > cfg.debounceFrames { recent.removeFirst() }
        return recent.count < cfg.debounceFrames ? .clear : recent.min()!
    }

    private func direction(_ p: Point) -> Direction {
        if abs(p.x) > cfg.corridorHalfWidthM { return p.x < 0 ? .left : .right }
        return p.y >= 0 ? .front : .rear
    }

    private func evaluate(_ tr: LidarTrack, speed: Double, t: Double) -> (WarningLevel, Double?) {
        guard tr.hits >= cfg.confirmHits, tr.misses == 0 else { return (.clear, nil) }
        let c = tr.cluster
        let k = (tr.label == "truck" || tr.label == "bus") ? cfg.heavyVehicleFactor : 1.0
        var level = WarningLevel.clear
        var ttc: Double?
        func raise(_ l: WarningLevel, _ time: Double?) {
            if l > level { level = l; ttc = time } else if l == level, let time, time < (ttc ?? .infinity) { ttc = time }
        }
        let inCorridor = c.latGap < cfg.corridorHalfWidthM

        // Collision course along the road: it is in the rider's line and the gap is closing.
        if inCorridor && !c.alongside && tr.closingY > 0.5 {
            let ty = max(0, c.yGap - cfg.riderSpaceM) / tr.closingY
            if ty <= cfg.dangerS * k { raise(.danger, ty) } else if ty <= cfg.cautionS * k { raise(.caution, ty) }
        }
        // Same, timed by the camera (earlier than the LiDAR can see).
        if inCorridor && !c.alongside, let ct = tr.camTTC, t - tr.camT < 0.5 {
            if ct <= max(cfg.cameraDangerMinS, cfg.dangerS * k) { raise(.danger, ct) }
            else if ct <= max(cfg.cameraCautionMinS, cfg.cautionS * k) { raise(.caution, ct) }
        }
        // Anything solid in the path the rider covers in the next 1.5 s (an opening door).
        if speed > 0.5 {
            let reach = speed * cfg.dangerS
            let hit = c.points.filter { abs($0.x) <= cfg.pathHalfWidthM && $0.y > 0.3 && $0.y <= reach }.map(\.y).min()
            if let hit { raise(.danger, hit / speed) }
        }
        // Alongside and moving in on the rider.
        if c.alongside && !inCorridor && tr.closingX > 0.5 {
            let tx = max(0, c.latGap - cfg.riderSpaceM) / tr.closingX
            if tx <= cfg.dangerS { raise(.danger, tx) } else if tx <= cfg.cautionS { raise(.caution, tx) }
        }
        // Close pass: a moving object passes, or is about to pass, closer than 1.5 m.
        if tr.moving == true && !inCorridor && c.latGap < cfg.closePassM {
            if c.alongside { raise(.caution, nil) }
            else if tr.closingY > 0.5 && c.yGap / tr.closingY <= cfg.cautionS * k { raise(.caution, c.yGap / tr.closingY) }
        }
        // Door zone: riding within 1.5 m of a parked car.
        if tr.moving == false && speed > 1.5 && !inCorridor && c.latGap < cfg.doorZoneM
            && (c.alongside || (c.ahead && c.yGap <= speed * cfg.cautionS))
            && (tr.label == "car" || tr.label == "truck" || tr.label == "bus" || max(tr.length, tr.width) >= 1.0) {
            raise(.caution, nil)
        }
        // Approaching: a moving object closing faster than 2 m/s.
        if tr.moving == true && !c.alongside && tr.closingY > cfg.approachingSpeed && c.nearest.range < cfg.approachingRangeM {
            raise(.approaching, c.yGap / tr.closingY)
        }
        return (level, ttc)
    }

    private func evaluate(_ ct: CameraTrack, speed: Double, lidarLost: Bool) -> (WarningLevel, Double?) {
        guard ct.hits >= cfg.confirmHits, let ttc = ct.ttc else { return (.clear, nil) }
        let k = (ct.cls == "truck" || ct.cls == "bus") ? cfg.heavyVehicleFactor : 1.0
        if ct.gap < cfg.corridorHalfWidthM {
            if ttc <= max(cfg.cameraDangerMinS, cfg.dangerS * k) { return (.danger, ttc) }
            if ttc <= max(cfg.cameraCautionMinS, cfg.cautionS * k) { return (.caution, ttc) }
        } else if lidarLost && ct.gap < cfg.closePassM && ttc <= cfg.cautionS * k {
            return (.caution, ttc)                    // with no LiDAR, the camera has to call close passes
        }
        // Only from behind: ahead, everything parked "approaches" at the rider's own speed.
        let closing = ct.dist / max(ttc, 0.1)
        if ct.facing < 0 && closing > cfg.approachingSpeed && ct.dist < cfg.approachingRangeM {
            return (.approaching, ttc)
        }
        return (.clear, nil)
    }
}
