import Foundation
import HelmetCore
import Network

/// Listens for the bike unit's LiDAR packets (UDP port 5005, 730 bytes, about 10 a second).
public final class LidarReceiver: ObservableObject {
    @Published public private(set) var packetsPerSecond = 0
    @Published public private(set) var error: String?

    private var listener: NWListener?
    private let queue = DispatchQueue(label: "lidar.udp")
    private let lock = NSLock()
    private var latest: LidarScan?
    private var count = 0
    private var meter: Timer?

    public init() {}

    public func start(port: UInt16 = 5005) {
        guard listener == nil else { return }
        do {
            let params = NWParameters.udp
            params.allowLocalEndpointReuse = true
            let l = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)
            l.newConnectionHandler = { [weak self] conn in
                conn.start(queue: self?.queue ?? .main)
                self?.receive(on: conn)
            }
            l.stateUpdateHandler = { [weak self] state in
                if case .failed(let e) = state { DispatchQueue.main.async { self?.error = e.localizedDescription } }
            }
            l.start(queue: queue)
            listener = l
            error = nil
        } catch {
            self.error = error.localizedDescription
        }
        meter = Timer.scheduledTimer(withTimeInterval: 1, repeats: true) { [weak self] _ in
            guard let self else { return }
            self.lock.lock(); let n = self.count; self.count = 0; self.lock.unlock()
            self.packetsPerSecond = n
        }
    }

    public func stop() {
        listener?.cancel(); listener = nil
        meter?.invalidate(); meter = nil
        packetsPerSecond = 0
    }

    /// The newest scan since the last call, or nil if none arrived.
    public func takeLatest() -> LidarScan? {
        lock.lock(); defer { lock.unlock() }
        let s = latest; latest = nil
        return s
    }

    private func receive(on conn: NWConnection) {
        conn.receiveMessage { [weak self] data, _, _, error in
            guard let self else { return }
            if let data, let scan = LidarScan(packet: data) {
                self.lock.lock(); self.latest = scan; self.count += 1; self.lock.unlock()
            }
            if error == nil { self.receive(on: conn) } else { conn.cancel() }
        }
    }
}
