import HelmetCore
import SwiftUI

extension WarningLevel {
    var color: Color {
        switch self {
        case .clear: return .green
        case .approaching: return .blue
        case .caution: return .orange
        case .danger: return .red
        }
    }
}

public struct RootView: View {
    @StateObject private var model = AppModel()

    public init() {}

    public var body: some View {
        TabView {
            RideView(model: model).tabItem { Label("Ride", systemImage: "bicycle") }
            HelmetTestView(model: model, helmet: model.helmet).tabItem { Label("Helmet", systemImage: "dot.radiowaves.left.and.right") }
            SummaryView(model: model).tabItem { Label("Summary", systemImage: "list.bullet") }
        }
        .onAppear { model.helmet.start() }
    }
}

struct RideView: View {
    @ObservedObject var model: AppModel

    var body: some View {
        let out = model.output
        let level = out?.level ?? .clear
        VStack(spacing: 12) {
            VStack(spacing: 2) {
                Text(model.riding ? level.name : "READY").font(.system(size: 34, weight: .heavy))
                Text(detail(out)).font(.subheadline)
            }
            .frame(maxWidth: .infinity).padding(.vertical, 10)
            .background(model.riding ? level.color : Color.gray).foregroundStyle(.white)
            .clipShape(RoundedRectangle(cornerRadius: 12))

            TopDownView(output: out).frame(maxWidth: .infinity, maxHeight: .infinity)
                .background(Color.black).clipShape(RoundedRectangle(cornerRadius: 12))

            StatusRow(model: model, helmet: model.helmet, lidar: model.lidar)

            HStack {
                Picker("Source", selection: $model.source) {
                    Text("Live sensors").tag(RideSource.live)
                    ForEach(model.replayNames, id: \.self) { Text("Replay: " + RideSource.replay($0).title).tag(RideSource.replay($0)) }
                }
                .disabled(model.riding)
                Spacer()
                Button(model.riding ? "Stop ride" : "Start ride") { model.riding ? model.stop() : model.start() }
                    .buttonStyle(.borderedProminent).tint(model.riding ? .red : .green)
            }
        }
        .padding()
    }

    private func detail(_ out: PipelineOutput?) -> String {
        guard model.riding, let out else { return "Pick a source and start a ride" }
        var parts = [String(format: "%.0f km/h", model.speed * 3.6)]
        if out.level >= .caution && out.dir != .none { parts.append(out.dir.rawValue) }
        if out.level >= .approaching, let t = out.ttc { parts.append(String(format: "%.1f s", t)) }
        return parts.joined(separator: " · ")
    }
}

struct StatusRow: View {
    @ObservedObject var model: AppModel
    @ObservedObject var helmet: HelmetLink
    @ObservedObject var lidar: LidarReceiver

    var body: some View {
        HStack(spacing: 8) {
            chip("Helmet", helmet.state == .connected, helmet.state.rawValue)
            if model.riding, let out = model.output {
                chip("LiDAR", !out.lidarLost, out.lidarLost ? "No data" : (model.source == .live ? "\(lidar.packetsPerSecond)/s" : "Replay"))
                chip("Rear cam", !out.rearCameraLost, out.rearCameraLost ? "No data" : "OK")
            } else {
                chip("LiDAR", false, "Idle")
                chip("Rear cam", false, "Idle")
            }
        }
    }

    private func chip(_ name: String, _ ok: Bool, _ text: String) -> some View {
        VStack(spacing: 1) {
            Text(name).font(.caption2).foregroundStyle(.secondary)
            Text(text).font(.caption.weight(.semibold)).foregroundStyle(ok ? Color.green : Color.red)
        }
        .frame(maxWidth: .infinity).padding(.vertical, 6)
        .background(Color.gray.opacity(0.15)).clipShape(RoundedRectangle(cornerRadius: 8))
    }
}

/// The view from above: rider at the triangle, forward is up. Dots are LiDAR points, boxes are objects.
struct TopDownView: View {
    var output: PipelineOutput?

    var body: some View {
        Canvas { ctx, size in
            let scale = min(size.width / 11, size.height / 20)             // pixels per metre
            let origin = CGPoint(x: size.width / 2, y: size.height * 0.6)
            func pt(_ x: Double, _ y: Double) -> CGPoint { CGPoint(x: origin.x + x * scale, y: origin.y - y * scale) }
            for r in [1.5, 5.0, 10.0] {
                let rect = CGRect(x: origin.x - r * scale, y: origin.y - r * scale, width: 2 * r * scale, height: 2 * r * scale)
                ctx.stroke(Path(ellipseIn: rect), with: .color(.white.opacity(0.15)), lineWidth: 1)
            }
            if let out = output {
                for p in out.points {
                    let c = pt(p.x, p.y)
                    ctx.fill(Path(ellipseIn: CGRect(x: c.x - 1.5, y: c.y - 1.5, width: 3, height: 3)), with: .color(.white.opacity(0.7)))
                }
                for tr in out.tracks {
                    let c = pt(tr.x, tr.y)
                    let rect = CGRect(x: c.x - tr.width * scale / 2, y: c.y - tr.length * scale / 2,
                                      width: tr.width * scale, height: tr.length * scale)
                    let color = tr.level == .clear ? Color.gray : tr.level.color
                    ctx.stroke(Path(roundedRect: rect, cornerRadius: 2), with: .color(color),
                               style: StrokeStyle(lineWidth: 2, dash: tr.cameraOnly ? [4, 3] : []))
                    if let label = tr.label {
                        ctx.draw(Text(label).font(.caption2).foregroundColor(color), at: CGPoint(x: rect.midX, y: rect.minY - 7))
                    }
                }
            }
            var rider = Path()
            rider.move(to: CGPoint(x: origin.x, y: origin.y - 9))
            rider.addLine(to: CGPoint(x: origin.x - 6, y: origin.y + 7))
            rider.addLine(to: CGPoint(x: origin.x + 6, y: origin.y + 7))
            rider.closeSubpath()
            ctx.fill(rider, with: .color(.cyan))
        }
    }
}

/// Sends hand-picked warnings to the helmet, to bring it up before any sensor works.
struct HelmetTestView: View {
    @ObservedObject var model: AppModel
    @ObservedObject var helmet: HelmetLink
    @State private var level = WarningLevel.clear
    @State private var dir = Direction.left

    var body: some View {
        Form {
            Section("Link") {
                LabeledContent("Helmet", value: helmet.state.rawValue)
                LabeledContent("Messages sent", value: "\(helmet.sentCount)")
            }
            Section("Test warning") {
                Picker("Level", selection: $level) {
                    Text("Clear").tag(WarningLevel.clear)
                    Text("Approaching").tag(WarningLevel.approaching)
                    Text("Caution").tag(WarningLevel.caution)
                    Text("Danger").tag(WarningLevel.danger)
                }
                Picker("Direction", selection: $dir) {
                    Text("Left").tag(Direction.left)
                    Text("Right").tag(Direction.right)
                    Text("Front").tag(Direction.front)
                    Text("Rear").tag(Direction.rear)
                }
                .pickerStyle(.segmented)
                Text("Sent ten times a second while no ride is running. A car is drawn on the warning's side.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
        }
        .onAppear(perform: apply)
        .onChange(of: level) { apply() }
        .onChange(of: dir) { apply() }
        .onDisappear { model.testMessage = nil }
    }

    private func apply() {
        let spot: (Double, Double) = [.left: (-1.6, -4.2), .right: (1.6, -4.2), .front: (0, 8), .rear: (0, -8)][dir] ?? (0, 0)
        let car = HelmetMessage.Target(x: spot.0, y: spot.1, width: 1.8, length: 4.5, type: "c")
        model.testMessage = HelmetMessage(level: level, dir: level >= .caution ? dir : .none,
                                          ttc: level >= .approaching ? 1.2 : nil, targets: level == .clear ? [] : [car])
    }
}

struct SummaryView: View {
    @ObservedObject var model: AppModel

    var body: some View {
        List {
            Section("This ride") {
                LabeledContent("Time", value: String(format: "%.0f s", model.rideSeconds))
                LabeledContent("Passes", value: "\(model.passes.count)")
                if let closest = model.passes.min(by: { $0.gapM < $1.gapM }) {
                    LabeledContent("Closest pass", value: String(format: "%.1f m", closest.gapM))
                }
            }
            Section("Passes") {
                if model.passes.isEmpty { Text("No passes logged yet").foregroundStyle(.secondary) }
                ForEach(model.passes) { p in
                    HStack {
                        Text("\(p.label ?? "object") passed on the \(p.side.rawValue)")
                        Spacer()
                        Text(String(format: "%.1f m", p.gapM)).monospacedDigit()
                            .foregroundStyle(p.gapM < 1.5 ? Color.orange : Color.primary)
                    }
                }
            }
        }
    }
}
