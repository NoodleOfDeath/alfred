//
//  AlfredMobileApp.swift
//  Shared
//
//  Created by tmorgan on 2/23/22.
//

import SwiftUI

let APIEndpoint = "http://batcomputer:8000"

@main
struct AlfredMobileApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
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
