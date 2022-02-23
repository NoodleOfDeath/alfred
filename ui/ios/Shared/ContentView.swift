//
//  ContentView.swift
//  Shared
//
//  Created by tmorgan on 2/23/22.
//

import SwiftUI

struct ContentView: View {
    var body: some View {
        VStack {
            Button("Drive Forward", role: .none) {
                drive(forward: true)
            }
            Button("Drive Backward", role: .none) {
                drive(forward: false)
            }
            Button("Turn Clockwise", role: .none) {
                rotate(clockwise: true)
            }
            Button("Turn Counter-Clockwise", role: .none) {
                rotate(clockwise: false)
            }
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
