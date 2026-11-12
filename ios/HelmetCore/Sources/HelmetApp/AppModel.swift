import CoreLocation
import Foundation
import HelmetCore
#if os(iOS)
import UIKit
#endif

/// Where a ride's data comes from.
public enum RideSource: Hashable {
    case live
    case replay(String)      // name of a bundled ride file

    public var title: String {
        switch self {
        case .live: return "Live sensors"
        case .replay(let n): return n.replacingOccurrences(of: "_", with: " ")
        }
    }
}

/// Runs a ride: feeds the pipeline ten times a second and forwards every result to the helmet.
@MainActor
public final class AppModel: NSObject, ObservableObject, CLLocationManagerDelegate {
    @Published public var source = RideSource.replay("table_demo_walkers")
    @Published public private(set) var riding = false
    @Published public private(set) var output: PipelineOutput?
    @Published public private(set) var passes: [PassEvent] = []
    @Published public private(set) var rideSeconds = 0.0
    @Published public private(set) var speed = 0.0
    @Published public var testMessage: HelmetMessage?      // set by the test screen; sent while no ride runs

    public let helmet = HelmetLink()
    public let lidar = LidarReceiver()
    public let replayNames: [String]

    private let pipeline = Pipeline()
    private var timer: Timer?
    private var frames: [RideLog.Frame] = []
    private var frameIndex = 0
    private var startDate = Date()
    private let location = CLLocationManager()

    public override init() {
        let urls = Bundle.module.urls(forResourcesWithExtension: "jsonl", subdirectory: "Rides") ?? []
        replayNames = urls.map { $0.deletingPathExtension().lastPathComponent }.sorted()
        super.init()
        location.delegate = self
        // The heartbeat: something is written to the helmet every 0.1 s, ride or no ride.
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.tick() }
        }
    }

    public func start() {
        pipeline.reset()
        passes = []; output = nil; rideSeconds = 0; frameIndex = 0; frames = []
        switch source {
        case .replay(let name):
            if let url = Bundle.module.url(forResource: name, withExtension: "jsonl", subdirectory: "Rides"),
               let ride = try? RideLog(contentsOf: url) {
                frames = ride.frames
            }
        case .live:
            lidar.start()
            location.requestWhenInUseAuthorization()
            location.startUpdatingLocation()
        }
        startDate = Date()
        riding = true
        setKeepAwake(true)
    }

    public func stop() {
        riding = false
        lidar.stop()
        location.stopUpdatingLocation()
        setKeepAwake(false)
    }

    private func tick() {
        guard riding else {
            // idle: keep the link alive with CLEAR, or with the test screen's message
            helmet.send(testMessage ?? HelmetMessage(level: .clear))
            return
        }
        let input: FrameInput
        switch source {
        case .replay:
            guard frameIndex < frames.count else { stop(); return }
            input = frames[frameIndex].input
            frameIndex += 1
            speed = input.phone.speed
        case .live:
            // Cameras and lean are not wired in yet: LiDAR and GPS speed only.
            input = FrameInput(t: Date().timeIntervalSince(startDate), lidar: lidar.takeLatest(), rear: nil, front: nil,
                               phone: PhoneState(speed: speed))
        }
        let out = pipeline.step(input)
        output = out
        rideSeconds = input.t
        if !out.newPasses.isEmpty || passes.count != pipeline.passes.count { passes = pipeline.passes }
        helmet.send(out.message)
    }

    /// Hold the screen awake only between Start and Stop; the rider's auto-lock setting is untouched.
    private func setKeepAwake(_ on: Bool) {
        #if os(iOS)
        UIApplication.shared.isIdleTimerDisabled = on
        #endif
    }

    nonisolated public func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let s = locations.last?.speed else { return }
        Task { @MainActor in if self.riding, case .live = self.source { self.speed = max(0, s) } }
    }

    nonisolated public func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {}
}
