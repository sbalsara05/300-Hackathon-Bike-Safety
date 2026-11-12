// Listens for LiDAR packets on this Mac and prints the live warning level, the same way the app does.
// Use it with sim/play_ride_udp.py (fake bike unit) or with the real QT Py pointed at this Mac.
//   swift run livecheck [seconds]
import Foundation
import HelmetApp
import HelmetCore

let seconds = Double(CommandLine.arguments.dropFirst().first ?? "") ?? 15
let receiver = LidarReceiver()
receiver.start()
let pipeline = Pipeline()
let start = Date()
var peak = WarningLevel.clear
var scans = 0
let timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { _ in
    let t = Date().timeIntervalSince(start)
    let scan = receiver.takeLatest()
    if scan != nil { scans += 1 }
    let out = pipeline.step(FrameInput(t: t, lidar: scan, rear: nil, front: nil, phone: PhoneState()))
    peak = max(peak, out.level)
    if Int(t * 10) % 5 == 0 {
        print(String(format: "%5.1f s  %@  %@  objects=%d  lidar=%@", t, out.level.name, out.dir.rawValue,
                     out.tracks.count, out.lidarLost ? "LOST" : "ok"))
    }
    if t >= seconds {
        print("scans received: \(scans), peak level: \(peak.name), passes: \(pipeline.passes.map { String(format: "%.2f m", $0.gapM) })")
        exit(0)
    }
}
RunLoop.main.run()
