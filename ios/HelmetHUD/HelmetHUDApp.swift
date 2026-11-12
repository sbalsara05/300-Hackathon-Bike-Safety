import HelmetApp
import SwiftUI

// The app itself is a thin shell; screens and logic live in the HelmetCore package next to this folder.
@main
struct HelmetHUDApp: App {
    var body: some Scene {
        WindowGroup { RootView() }
    }
}
