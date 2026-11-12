// swift-tools-version:5.9
import PackageDescription

let package = Package(
    name: "HelmetCore",
    platforms: [.iOS(.v17), .macOS(.v14)],
    products: [
        .library(name: "HelmetCore", targets: ["HelmetCore"]),
        .library(name: "HelmetApp", targets: ["HelmetApp"]),
    ],
    targets: [
        // Rules and data formats; no UI, no radios.
        .target(name: "HelmetCore"),
        // Screens, Bluetooth, Wi-Fi and replay. Lives in the package so it can be compiled without Xcode.
        .target(name: "HelmetApp", dependencies: ["HelmetCore"], resources: [.copy("Rides")]),
        // Runs every ride in sim/rides through the pipeline and checks it against the ride's expectations.
        .executableTarget(name: "ridecheck", dependencies: ["HelmetCore"]),
        // Prints the live warning level from LiDAR packets arriving on this Mac.
        .executableTarget(name: "livecheck", dependencies: ["HelmetApp"]),
    ]
)
