//
//  ContentView.swift
//  AlfredWatch WatchKit Extension
//
//  Created by tmorgan on 2/24/22.
//

import SwiftUI

struct ContentView: View {
    var body: some View {
        VStack {
            Button("⬆", role: .none) {
                drive(forward: true)
            }
            Button("⬇", role: .none) {
                drive(forward: false)
            }
            Button("➨", role: .none) {
                rotate(clockwise: true)
            }
            Button("⬅", role: .none) {
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
