import CoreBluetooth
import Foundation
import HelmetCore

/// Bluetooth link to the helmet (firmware/helmet/feather_hud_ble). The phone finds the helmet by its
/// service ID, connects, and writes one small JSON message 10-20 times a second. Reconnects on its own.
public final class HelmetLink: NSObject, ObservableObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    public enum State: String { case off = "Bluetooth off", unauthorized = "No permission", scanning = "Searching", connecting = "Connecting", connected = "Connected" }

    @Published public private(set) var state = State.off
    @Published public private(set) var sentCount = 0

    private var central: CBCentralManager?
    private var peripheral: CBPeripheral?
    private var rx: CBCharacteristic?
    private let service = CBUUID(string: HelmetMessage.serviceUUID)
    private let write = CBUUID(string: HelmetMessage.writeUUID)

    /// Call once; the first call triggers the Bluetooth permission prompt.
    public func start() {
        if central == nil { central = CBCentralManager(delegate: self, queue: .main) }
    }

    public func send(_ message: HelmetMessage) {
        guard let p = peripheral, let rx, state == .connected, p.canSendWriteWithoutResponse else { return }
        p.writeValue(message.encoded(), for: rx, type: .withoutResponse)
        sentCount += 1
    }

    private func scan() {
        peripheral = nil; rx = nil
        guard let central, central.state == .poweredOn else { return }
        state = .scanning
        central.scanForPeripherals(withServices: [service])
    }

    public func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn: scan()
        case .unauthorized: state = .unauthorized
        default: state = .off
        }
    }

    public func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                               advertisementData: [String: Any], rssi RSSI: NSNumber) {
        central.stopScan()
        self.peripheral = peripheral
        peripheral.delegate = self
        state = .connecting
        central.connect(peripheral)
    }

    public func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([service])
    }

    public func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) { scan() }

    public func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) { scan() }

    public func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let s = peripheral.services?.first(where: { $0.uuid == service }) else { return }
        peripheral.discoverCharacteristics([write], for: s)
    }

    public func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let c = service.characteristics?.first(where: { $0.uuid == write }) else { return }
        rx = c
        state = .connected
    }
}
