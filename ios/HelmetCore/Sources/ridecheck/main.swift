// Runs ride files through the pipeline and checks each against its expectations.
//   swift run ridecheck [rides-folder] [-v ride-name]
import Foundation
import HelmetCore

var args = Array(CommandLine.arguments.dropFirst())
var verbose: String?
if let i = args.firstIndex(of: "-v"), i + 1 < args.count { verbose = args[i + 1]; args.removeSubrange(i...i + 1) }
let dir = URL(fileURLWithPath: args.first ?? "../../sim/rides")
let files = ((try? FileManager.default.contentsOfDirectory(at: dir, includingPropertiesForKeys: nil)) ?? [])
    .filter { $0.pathExtension == "jsonl" }.sorted { $0.lastPathComponent < $1.lastPathComponent }
guard !files.isEmpty else { print("no ride files in \(dir.path)"); exit(2) }

var failures = 0
for url in files {
    let ride = try RideLog(contentsOf: url)
    let pipe = Pipeline()
    var peak = WarningLevel.clear
    var dirsAtPeak = Set<String>()
    var firstCaution: Double?, firstDanger: Double?
    var sawLidarLost = false, sawRearLost = false, maxBytes = 0
    for f in ride.frames {
        let out = pipe.step(f.input)
        if out.level > peak { peak = out.level; dirsAtPeak = [] }
        if out.level == peak && peak >= .caution { dirsAtPeak.insert(out.dir.rawValue) }
        if out.level >= .caution && firstCaution == nil { firstCaution = f.t }
        if out.level >= .danger && firstDanger == nil { firstDanger = f.t }
        sawLidarLost = sawLidarLost || out.lidarLost
        sawRearLost = sawRearLost || out.rearCameraLost
        maxBytes = max(maxBytes, out.message.encoded().count)
        if verbose == ride.meta.name {
            let tr = out.tracks.map { "#\($0.id)\($0.cameraOnly ? "c" : "")\($0.label?.prefix(1) ?? "?")(\(String(format: "%.1f,%.1f", $0.x, $0.y)))L\($0.level.rawValue)" }
            print(String(format: "%5.1f  %@ %@ ttc=%@  %@", f.t, out.level.name.padding(toLength: 11, withPad: " ", startingAt: 0),
                         out.dir.rawValue.padding(toLength: 5, withPad: " ", startingAt: 0),
                         out.ttc.map { String(format: "%.1f", $0) } ?? "-", tr.joined(separator: " ")))
        }
    }
    let e = ride.meta.expect
    var problems: [String] = []
    if let m = e.peak_min, peak.rawValue < m { problems.append("peak \(peak.rawValue) < \(m)") }
    if let m = e.peak_max, peak.rawValue > m { problems.append("peak \(peak.rawValue) > \(m)") }
    if let d = e.dir, peak >= .caution, !dirsAtPeak.contains(d) { problems.append("dir \(dirsAtPeak.sorted()) lacks \(d)") }
    if let c = ride.meta.truth_summary.contact_t {
        if let need = e.danger_lead_s_min {
            let lead = firstDanger.map { c - $0 } ?? -1
            if lead < need { problems.append(String(format: "DANGER lead %.1f s < %.1f", lead, need)) }
        }
        if let need = e.caution_lead_s_min {
            let lead = firstCaution.map { c - $0 } ?? -1
            if lead < need { problems.append(String(format: "CAUTION lead %.1f s < %.1f", lead, need)) }
        }
    }
    if let g = e.pass_gap_m {
        if let p = pipe.passes.min(by: { abs($0.gapM - g) < abs($1.gapM - g) }) {
            if abs(p.gapM - g) > 0.15 { problems.append(String(format: "pass gap %.2f m, expected %.2f", p.gapM, g)) }
        } else { problems.append("no pass logged") }
    }
    if e.sensor_flag == "lidar" && !sawLidarLost { problems.append("LiDAR loss not flagged") }
    if e.sensor_flag == "rear_camera" && !sawRearLost { problems.append("rear camera loss not flagged") }
    if e.sensor_flag != "lidar" && sawLidarLost { problems.append("false LiDAR loss") }
    if e.sensor_flag != "rear_camera" && sawRearLost { problems.append("false rear camera loss") }
    if maxBytes > HelmetMessage.maxBytes { problems.append("message \(maxBytes) bytes") }

    let lead = ride.meta.truth_summary.contact_t.map { c in
        String(format: " lead C=%.1f D=%.1f", firstCaution.map { c - $0 } ?? -1, firstDanger.map { c - $0 } ?? -1) } ?? ""
    let passText = pipe.passes.map { String(format: "%.2f%@", $0.gapM, $0.side == .left ? "L" : "R") }.joined(separator: ",")
    let status = problems.isEmpty ? "PASS" : "FAIL"
    print("\(status)  \(ride.meta.name.padding(toLength: 30, withPad: " ", startingAt: 0)) peak=\(peak.name.padding(toLength: 11, withPad: " ", startingAt: 0)) dir=\(dirsAtPeak.sorted().joined(separator: "/"))\(lead) passes=[\(passText)] \(problems.joined(separator: "; "))")
    if !problems.isEmpty { failures += 1 }
}
print("\(files.count - failures)/\(files.count) rides pass")
exit(failures == 0 ? 0 : 1)
