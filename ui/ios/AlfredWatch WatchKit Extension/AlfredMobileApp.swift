//
//  AlfredMobileApp.swift
//  AlfredWatch WatchKit Extension
//
//  Created by tmorgan on 2/24/22.
//

import SwiftUI

let APIEndpoint = "http://192.168.68.158:8000"

@main
struct AlfredMobileApp: App {
    @SceneBuilder var body: some Scene {
        WindowGroup {
            NavigationView {
                ContentView()
            }
        }

        WKNotificationScene(controller: NotificationController.self, category: "myCategory")
    }
}

func drive(forward: Bool = true) {
    guard let url = URL(string: "\(APIEndpoint)/drive/") else { return }
    var request = URLRequest(url: url)
    request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
    request.httpMethod = "POST"
    request.httpBody = "{\"speed\": \(forward ? 0.5 : -0.5)}".data(using: .utf8)
    let task = URLSession.shared.dataTask(with: request) {(data, response, error) in
        guard let data = data else { return }
        print(String(data: data, encoding: .utf8)!)
    }
    task.resume()
}

func rotate(clockwise: Bool = true) {
    guard let url = URL(string: "\(APIEndpoint)/rotate/") else { return }
    var request = URLRequest(url: url)
    request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
    request.httpMethod = "POST"
    request.httpBody = "{\"speed\": \(clockwise ? 2.5 : -2.5)}".data(using: .utf8)
    let task = URLSession.shared.dataTask(with: request) {(data, response, error) in
        guard let data = data else { return }
        print(String(data: data, encoding: .utf8)!)
    }
    task.resume()
}
