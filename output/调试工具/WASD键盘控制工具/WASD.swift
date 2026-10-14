import Foundation
import CoreBluetooth
import Darwin

func keyCommand(_ byte: UInt8) -> String? {
    switch byte {
    case 87, 119: return "F!"
    case 83, 115: return "B!"
    case 65, 97: return "L!"
    case 68, 100: return "R!"
    case 32, 88, 120: return "STOP!"
    default: return nil
    }
}

struct DriveIntent: Equatable {
    let command: String
    let speed: Int
    init?(_ byte: UInt8) {
        guard let command = keyCommand(byte), command != "STOP!" else { return nil }
        self.command = command
        self.speed = (65...90).contains(byte) ? 300 : 120
    }
}

struct SpeedPlanner {
    enum Phase { case idle, waitingStop, waitingSpeed }
    private(set) var phase: Phase = .idle
    private(set) var speed: Int?
    private(set) var desired: DriveIntent?
    private var deadline = 0.0

    mutating func request(_ intent: DriveIntent, now: Double) -> [String] {
        if desired == intent { return [] } // Keyboard repeat never extends the deadline.
        desired = intent
        if phase == .idle && speed == intent.speed { return [intent.command] }
        phase = .waitingStop; deadline = now + 3
        return ["STOP!"]
    }

    mutating func feedback(state: String, speed: Int, now: Double) -> [String] {
        self.speed = speed
        guard now < deadline, let intent = desired else { return [] }
        if phase == .waitingStop && state == "STOP" {
            phase = .waitingSpeed
            return ["V\(intent.speed)!"]
        }
        if phase == .waitingSpeed && state == "STOP" && speed == intent.speed {
            phase = .idle
            return [intent.command]
        }
        return []
    }

    mutating func cancel() { phase = .idle; desired = nil; deadline = 0 }
    mutating func expire(now: Double) -> Bool {
        guard phase != .idle && now >= deadline else { return false }
        cancel(); return true
    }
}

func dpiFields(_ line: String) -> [String: String] {
    var fields: [String: String] = [:]
    for token in line.split(separator: " ") {
        let pair = token.split(separator: "=", maxSplits: 1)
        if pair.count == 2 { fields[String(pair[0])] = String(pair[1]) }
    }
    return fields
}

struct CommandQueue {
    var pending: [String] = []
    var lastMotion: String?
    mutating func add(_ command: String) {
        if command == "STOP!" {
            pending.removeAll(); lastMotion = nil; pending.append(command)
        } else if command.hasPrefix("V") {
            pending.removeAll { $0.hasPrefix("V") }
            pending.append(command)
        } else if command != lastMotion {
            // A queued speed setting must remain ahead of the motion using it.
            pending.removeAll { $0 != "STOP!" && !$0.hasPrefix("V") }
            pending.append(command); lastMotion = command
        }
    }
}

// Decode bytes before constructing Swift Characters: CRLF is one grapheme,
// so String.firstIndex(of: "\n") does not match a combined CRLF Character.
struct LineDecoder {
    private var pending: [UInt8] = []
    private var discarding = false
    mutating func feed(_ data: Data) -> [String] {
        var lines: [String] = []
        for byte in data {
            if byte == 10 {
                if !discarding {
                    lines.append(String(decoding: pending, as: UTF8.self)
                        .trimmingCharacters(in: .whitespacesAndNewlines))
                }
                pending.removeAll(keepingCapacity: true); discarding = false
            } else if !discarding {
                if pending.count >= 8192 {
                    pending.removeAll(keepingCapacity: true); discarding = true
                } else { pending.append(byte) }
            }
        }
        return lines
    }
}

func isDPI(_ line: String) -> Bool { line.hasPrefix("DPI ") && line.contains("ST=") }
func isStoppedDPI(_ line: String) -> Bool {
    isDPI(line) && line.split(separator: " ").contains("ST=STOP")
}

func bt24WriteIndex(_ channels: [(service: String, characteristic: String)]) -> Int? {
    channels.firstIndex { $0.service == "FFE0" && $0.characteristic == "FFE2" }
}

if CommandLine.arguments.contains("--self-test") {
    assert(keyCommand(119) == "F!" && keyCommand(115) == "B!")
    assert(keyCommand(97) == "L!" && keyCommand(100) == "R!")
    assert(keyCommand(32) == "STOP!" && keyCommand(120) == "STOP!")
    assert(keyCommand(13) == nil)
    var q = CommandQueue()
    q.add("F!"); q.add("F!"); assert(q.pending == ["F!"])
    q.add("B!"); assert(q.pending == ["B!"])
    q.add("STOP!"); assert(q.pending == ["STOP!"] && q.lastMotion == nil)
    q.add("L!"); assert(q.pending == ["STOP!", "L!"])
    q.add("STOP!"); assert(q.pending == ["STOP!"])
    q.add("V300!"); q.add("F!"); assert(q.pending == ["STOP!", "V300!", "F!"])
    q.add("L!"); assert(q.pending == ["STOP!", "V300!", "L!"])
    q.add("STOP!"); assert(q.pending == ["STOP!"])
    let expected = ["DPI T=0 ST=DONE", "DPI T=1 ST=RUN"]
    let packet = Data((expected[0] + "\r\n" + expected[1] + "\n").utf8)
    for size in 1...packet.count {
        var decoder = LineDecoder(); var lines: [String] = []
        for offset in stride(from: 0, to: packet.count, by: size) {
            lines += decoder.feed(packet.subdata(in: offset..<min(offset + size, packet.count)))
        }
        assert(lines == expected && lines.allSatisfy(isDPI))
    }
    var decoder = LineDecoder()
    assert(decoder.feed(Data("\r".utf8)).isEmpty)
    assert(decoder.feed(Data("\n".utf8)) == [""])
    assert(decoder.feed(Data(repeating: 65, count: 9000)).isEmpty)
    assert(decoder.feed(Data("\nDPI T=0 ST=OFF\r\n".utf8)) == ["DPI T=0 ST=OFF"])
    assert(!isDPI("hello") && !isDPI("DPI incomplete"))
    assert(isStoppedDPI("DPI T=0 ST=STOP F=-"))
    assert(!isStoppedDPI("DPI T=0 ST=IDLE F=-"))
    assert(!isStoppedDPI("DPI T=0 ST=STOPPED F=-"))
    assert(bt24WriteIndex([("FFE0", "FFE1"), ("FFE0", "FFE2")]) == 1)
    assert(bt24WriteIndex([("FFE0", "FFE2"), ("FFE0", "FFE1")]) == 0)
    assert(bt24WriteIndex([("ABCD", "FFE2")]) == nil)
    assert(DriveIntent(119)?.speed == 120 && DriveIntent(87)?.speed == 300)
    assert(DriveIntent(83)?.command == "B!" && DriveIntent(65)?.command == "L!")
    assert(DriveIntent(68)?.speed == 300 && DriveIntent(32) == nil)
    let slow = DriveIntent(119)!, fast = DriveIntent(87)!
    var planner = SpeedPlanner()
    _ = planner.feedback(state: "STOP", speed: 120, now: 0)
    assert(planner.request(slow, now: 1) == ["F!"])
    assert(planner.request(fast, now: 2) == ["STOP!"])
    assert(planner.feedback(state: "RUN", speed: 120, now: 2.1).isEmpty)
    assert(planner.feedback(state: "STOP", speed: 120, now: 2.2) == ["V300!"])
    assert(planner.feedback(state: "STOP", speed: 120, now: 2.3).isEmpty)
    assert(planner.feedback(state: "RUN", speed: 300, now: 2.4).isEmpty)
    assert(planner.feedback(state: "STOP", speed: 300, now: 2.5) == ["F!"])
    assert(planner.request(fast, now: 2.6).isEmpty)
    assert(planner.request(slow, now: 3) == ["STOP!"])
    assert(planner.feedback(state: "STOP", speed: 300, now: 3.1) == ["V120!"])
    planner.cancel()
    assert(planner.feedback(state: "STOP", speed: 120, now: 3.2).isEmpty)
    assert(planner.request(fast, now: 4) == ["STOP!"])
    assert(planner.feedback(state: "STOP", speed: 120, now: 4.1) == ["V300!"])
    assert(planner.request(fast, now: 6.9).isEmpty)
    assert(planner.expire(now: 7)) // Firmware rejecting V300 never starts driving.
    assert(planner.feedback(state: "STOP", speed: 300, now: 7.1).isEmpty)
    assert(planner.request(slow, now: 8) == ["STOP!"])
    assert(planner.request(DriveIntent(65)!, now: 8.1) == ["STOP!"])
    assert(planner.feedback(state: "STOP", speed: 120, now: 8.2) == ["V300!"])
    assert(planner.feedback(state: "STOP", speed: 300, now: 8.3) == ["L!"])
    assert(dpiFields("DPI ST=STOP V=300")["V"] == "300")
    print("PASS: WASD, repeat suppression, STOP priority, CRLF/LF, fragmented/multiple lines, oversized line recovery. No Bluetooth manager created.")
    print("PASS: Shift 300/normal 120, STOP→speed confirmation→movement, cancellation, latest intent, rejection timeout, repeated keys never extend deadlines.")
    exit(0)
}

if let index = CommandLine.arguments.firstIndex(of: "--replay-log") {
    guard index + 1 < CommandLine.arguments.count else { print("需要JSON路径。"); exit(1) }
    do {
        let records = try JSONSerialization.jsonObject(with: Data(contentsOf:
            URL(fileURLWithPath: CommandLine.arguments[index + 1]))) as? [[String: Any]] ?? []
        var decoder = LineDecoder(); var count = 0; var stops = 0; var first = ""
        for record in records where record["type"] as? String == "read" {
            for line in decoder.feed(Data((record["data"] as? String ?? "").utf8)) where isDPI(line) {
                count += 1; if isStoppedDPI(line) { stops += 1 }; if first.isEmpty { first = line }
            }
        }
        guard count > 0 else { print("FAIL: 没有识别到DPI。"); exit(1) }
        print("PASS: replay recognized \(count) DPI lines, \(stops) ST=STOP confirmations. Movement gate: \(stops > 0 ? "enabled" : "blocked"). No Bluetooth manager created.")
        print("First: \(first)"); exit(0)
    } catch { print("回放失败：\(error.localizedDescription)"); exit(1) }
}

final class Controller: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    var central: CBCentralManager!
    var devices: [CBPeripheral] = []
    var names: [UUID: String] = [:]
    var selected: CBPeripheral?
    var writes: [CBCharacteristic] = []
    var notifies: [CBCharacteristic] = []
    var tx: CBCharacteristic?
    var servicesLeft = 0
    var queue = CommandQueue()
    var speedPlanner = SpeedPlanner()
    var inFlight = false
    var ready = false
    var sawDPI = false
    var stopConfirmed = false
    var closing = false
    var raw = false
    var savedTerm = termios()
    var input: DispatchSourceRead?
    var signalSources: [DispatchSourceSignal] = []
    var decoders: [ObjectIdentifier: LineDecoder] = [:]
    var log: FileHandle?
    var logFirst = true
    var logPath = ""

    func start() {
        print("先关闭原蓝牙连接，再打开小车电源；保持使能OFF。")
        print("正在扫描BLE蓝牙，约6秒后选择你的小车……")
        central = CBCentralManager(delegate: self, queue: .main)
        for sig in [SIGINT, SIGTERM] {
            signal(sig, SIG_IGN)
            let source = DispatchSource.makeSignalSource(signal: sig, queue: .main)
            source.setEventHandler { [weak self] in self?.quit() }
            source.resume(); signalSources.append(source)
        }
    }

    func centralManagerDidUpdateState(_ manager: CBCentralManager) {
        guard manager.state == .poweredOn else {
            if manager.state == .unauthorized { print("蓝牙权限未允许；在系统设置中允许终端访问蓝牙。") }
            else if manager.state != .unknown && manager.state != .resetting {
                print("蓝牙不可用，请打开Mac蓝牙。")
            } else { return }
            finish(1); return
        }
        manager.scanForPeripherals(withServices: nil)
        DispatchQueue.main.asyncAfter(deadline: .now() + 6) { [weak self] in self?.chooseDevice() }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        if !devices.contains(where: { $0.identifier == peripheral.identifier }) { devices.append(peripheral) }
        names[peripheral.identifier] = advertisementData[CBAdvertisementDataLocalNameKey] as? String ?? peripheral.name ?? "未命名设备"
    }

    func choice(_ count: Int) -> Int? {
        guard let line = readLine(), let n = Int(line.trimmingCharacters(in: .whitespaces)), n > 0, n <= count else { return nil }
        return n - 1
    }

    func chooseDevice() {
        guard !closing else { return }
        central.stopScan()
        if devices.isEmpty {
            print("没有扫描到BLE设备。先断开网页/手机连接后重试；本工具不扫描经典蓝牙SPP。")
            finish(1); return
        }
        for (i, p) in devices.enumerated() { print("\(i + 1). \(names[p.identifier] ?? "?")  \(p.identifier)") }
        print("输入小车的编号，再回车：", terminator: ""); fflush(stdout)
        guard let n = choice(devices.count) else { finish(1); return }
        selected = devices[n]; selected!.delegate = self
        print("连接 \(names[selected!.identifier] ?? "小车")……")
        central.connect(selected!)
        DispatchQueue.main.asyncAfter(deadline: .now() + 15) { [weak self] in
            guard let self = self, !self.ready, !self.closing else { return }
            print("连接或服务发现超时，退出后重试。"); self.quit()
        }
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices(nil)
    }
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("连接失败：\(error?.localizedDescription ?? "未知原因")"); finish(1)
    }
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        if !closing { print("\n蓝牙已断开。需要停车时拨使能OFF。"); finish(1) }
    }
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard error == nil, let services = peripheral.services, !services.isEmpty else {
            print("服务发现失败：\(error?.localizedDescription ?? "没有服务")"); quit(); return
        }
        servicesLeft = services.count
        for service in services { peripheral.discoverCharacteristics(nil, for: service) }
    }
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if error == nil {
            for ch in service.characteristics ?? [] {
                if ch.properties.contains(.write) || ch.properties.contains(.writeWithoutResponse) { writes.append(ch) }
                if ch.properties.contains(.notify) || ch.properties.contains(.indicate) { notifies.append(ch) }
            }
        }
        servicesLeft -= 1
        if servicesLeft == 0 { chooseChannel(peripheral) }
    }

    func chooseChannel(_ peripheral: CBPeripheral) {
        guard !writes.isEmpty else { print("没有找到可写的蓝牙通道。"); quit(); return }
        let name = names[peripheral.identifier] ?? peripheral.name ?? ""
        let isBT24 = name.uppercased().contains("BT24")
        let bt24Index = bt24WriteIndex(writes.map {
            (service: $0.service?.uuid.uuidString ?? "", characteristic: $0.uuid.uuidString)
        })
        if isBT24, let index = bt24Index {
            tx = writes[index]
            print("BT24自动选择发送：FFE0 / FFE2")
        } else if writes.count == 1 { tx = writes[0] }
        else {
            print("可写通道（选择原串口助手使用的发送特征）：")
            for (i, ch) in writes.enumerated() { print("\(i + 1). 服务 \(ch.service?.uuid.uuidString ?? "?") / 特征 \(ch.uuid)") }
            print("输入发送通道编号：", terminator: ""); fflush(stdout)
            guard let n = choice(writes.count) else { quit(); return }
            tx = writes[n]
        }
        let receive = notifies.filter {
            $0.service === tx?.service && (!isBT24 || bt24Index == nil || $0.uuid.uuidString == "FFE1")
        }
        guard !receive.isEmpty else { print("所选通道没有同服务的接收通知，请选串口通道重试。"); quit(); return }
        for ch in receive { peripheral.setNotifyValue(true, for: ch) }
        print("接收通知：\(receive.map { $0.uuid.uuidString }.joined(separator: ", "))")
        ready = true
        makeLog(); startKeyboard()
        queue.add("STOP!"); pump()
        print("\nw/s/a/d：前进/后退/左转/右转，120mm/s；Shift+W/S/A/D：300mm/s")
        print("空格/X停车；Q/Esc停车并退出。换速度先停再走；松Shift不自动减速，按小写方向键恢复120。")
        print("等待DPI和小车ST=STOP响应确认；终端不能检测松键，停车用空格。")
        DispatchQueue.main.asyncAfter(deadline: .now() + 5) { [weak self] in
            guard let self = self, !self.stopConfirmed, !self.closing else { return }
            if self.sawDPI {
                print("\n收到DPI，但小车没有回复ST=STOP。暂不发送移动命令；检查发送通道和蓝牙到主板的接收链路。")
            } else {
                print("\n还没有收到DPI，暂不发送移动命令。确认已烧录手动固件/接收通道是否正确。")
            }
        }
    }

    func startKeyboard() {
        guard isatty(STDIN_FILENO) == 1, tcgetattr(STDIN_FILENO, &savedTerm) == 0 else {
            print("请在终端里启动本工具。"); quit(); return
        }
        var mode = savedTerm
        mode.c_lflag &= ~tcflag_t(ICANON | ECHO)
        mode.c_iflag &= ~tcflag_t(IXON | ICRNL)
        withUnsafeMutableBytes(of: &mode.c_cc) { b in b[Int(VMIN)] = 1; b[Int(VTIME)] = 0 }
        guard tcsetattr(STDIN_FILENO, TCSANOW, &mode) == 0 else { quit(); return }
        raw = true
        input = DispatchSource.makeReadSource(fileDescriptor: STDIN_FILENO, queue: .main)
        input!.setEventHandler { [weak self] in
            guard let self = self, !self.closing else { return }
            var bytes = [UInt8](repeating: 0, count: 128)
            let n = Darwin.read(STDIN_FILENO, &bytes, bytes.count)
            if n <= 0 { self.quit(); return }
            for b in bytes.prefix(n) {
                if b == 113 || b == 81 || b == 27 { self.quit(); break }
                guard let command = keyCommand(b) else { continue }
                if command != "STOP!" && !self.stopConfirmed { print("尚未确认小车ST=STOP响应，移动命令未发送。"); continue }
                if command == "STOP!" {
                    self.speedPlanner.cancel(); self.enqueue([command])
                } else if let intent = DriveIntent(b) {
                    let actions = self.speedPlanner.request(intent, now: ProcessInfo.processInfo.systemUptime)
                    if !actions.isEmpty {
                        print("[请求] \(intent.command) 目标\(intent.speed)mm/s")
                        self.enqueue(actions)
                        DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
                            guard let self = self, !self.closing,
                                self.speedPlanner.expire(now: ProcessInfo.processInfo.systemUptime) else { return }
                            print("切换速度未获确认，已取消行驶并发送STOP!；检查日志V和板上固件速度上限。")
                            self.enqueue(["STOP!"])
                        }
                    }
                }
            }
        }
        input!.resume()
    }

    func enqueue(_ commands: [String]) {
        for command in commands { queue.add(command) }
        pump()
    }

    func pump() {
        guard ready, !inFlight, let p = selected, p.state == .connected, let ch = tx else { return }
        while !queue.pending.isEmpty {
            let withResponse = ch.properties.contains(.write)
            if !withResponse && !p.canSendWriteWithoutResponse { return }
            let command = queue.pending.removeFirst()
            let data = Data(command.utf8)
            inFlight = withResponse
            p.writeValue(data, for: ch, type: withResponse ? .withResponse : .withoutResponse)
            event("write", data); print("[发送] \(command)")
            if withResponse { return }
        }
    }
    func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) { pump() }
    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        inFlight = false
        if let error = error { print("发送失败：\(error.localizedDescription)"); quit(); return }
        pump()
    }
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard error == nil, let data = characteristic.value else { return }
        event("read", data)
        let channel = ObjectIdentifier(characteristic)
        var decoder = decoders[channel] ?? LineDecoder()
        let lines = decoder.feed(data)
        decoders[channel] = decoder
        for line in lines {
            if isDPI(line) {
                if !sawDPI { sawDPI = true; print("收到DPI，等待小车停车响应。") }
                if !stopConfirmed && isStoppedDPI(line) {
                    stopConfirmed = true; print("小车已回复ST=STOP，键盘控制已就绪。")
                }
                let fields = dpiFields(line)
                if let state = fields["ST"], let rawSpeed = fields["V"], let speed = Int(rawSpeed), !closing {
                    let commands = speedPlanner.feedback(state: state, speed: speed,
                        now: ProcessInfo.processInfo.systemUptime)
                    enqueue(commands)
                }
                print(line)
            }
        }
    }

    func makeLog() {
        let folder = URL(fileURLWithPath: CommandLine.arguments[0]).deletingLastPathComponent().appendingPathComponent("接收记录")
        do {
            try FileManager.default.createDirectory(at: folder, withIntermediateDirectories: true)
            let df = DateFormatter(); df.timeZone = TimeZone(identifier: "Asia/Shanghai"); df.dateFormat = "yyyy-MM-dd-HH-mm-ss"
            let file = folder.appendingPathComponent("records-wasd-\(df.string(from: Date())).json")
            FileManager.default.createFile(atPath: file.path, contents: Data("[\n".utf8))
            log = try FileHandle(forWritingTo: file); try log?.seekToEnd(); logPath = file.path
            print("日志：\(file.path)")
        } catch { print("日志文件创建失败：\(error.localizedDescription)") }
    }
    func event(_ type: String, _ bytes: Data) {
        let record: [String: Any] = ["type": type, "data": String(decoding: bytes, as: UTF8.self),
            "timestamp": Int(Date().timeIntervalSince1970 * 1000), "byteLength": bytes.count, "display": "ascii"]
        if let data = try? JSONSerialization.data(withJSONObject: record) {
            try? log?.write(contentsOf: Data((logFirst ? "" : ",\n").utf8)); try? log?.write(contentsOf: data); logFirst = false
        }
    }
    func quit() {
        guard !closing else { return }
        closing = true; speedPlanner.cancel(); queue.add("STOP!"); pump()
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.35) { [weak self] in self?.finish(0) }
    }
    func finish(_ code: Int32) {
        if raw { _ = tcsetattr(STDIN_FILENO, TCSANOW, &savedTerm); raw = false }
        try? log?.write(contentsOf: Data("\n]\n".utf8)); try? log?.close(); log = nil
        if let p = selected, p.state == .connected { central.cancelPeripheralConnection(p) }
        if !logPath.isEmpty { print("\n记录已保存：\(logPath)") }
        print("已退出。停车命令的发送不能代替实物确认；必要时拨使能OFF。")
        exit(code)
    }
}

let controller = Controller()
controller.start()
RunLoop.main.run()
