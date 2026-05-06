#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

namespace {
constexpr int kCommandPort = 50514;
constexpr int kTelemetryPort = 50513;
constexpr int kVisionPort = 50515;  // Optional: send "x y [timestamp]" on this UDP port.
constexpr int kControlPanelHeight = 120;

constexpr int kIdEditIp = 101;
constexpr int kIdEditRobotId = 102;
constexpr int kIdEditSendHz = 103;
constexpr int kIdEditRadius = 104;
constexpr int kIdEditOmega = 105;
constexpr int kIdBtnApply = 201;
constexpr int kIdBtnConnect = 202;
constexpr int kIdBtnDisconnect = 203;

struct Config {
    std::string robot_ip = "172.20.10.2";
    int robot_id = 1;
    double send_hz = 200.0;
    double circle_radius_m = 0.6;
    double angular_speed_rad_s = 1.0;
};

struct TelemetryState {
    bool valid = false;
    bool ball_found = false;
    double voltage = 0.0;
    double bearing = 0.0;
    double confidence = 0.0;
    long long robot_ts_ms = 0;
    std::chrono::steady_clock::time_point last_rx{};
    unsigned long long packets = 0;
};

struct VisionPose {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    long long source_ts = 0;
    std::chrono::steady_clock::time_point last_rx{};
};

struct AppState {
    Config cfg;
    SOCKET send_sock = INVALID_SOCKET;
    SOCKET telemetry_sock = INVALID_SOCKET;
    SOCKET vision_sock = INVALID_SOCKET;
    sockaddr_in robot_addr{};
    TelemetryState telemetry;
    VisionPose vision;
    std::deque<POINT> command_trail;
    double commanded_x = 0.0;
    double commanded_y = 0.0;
    double commanded_vx = 0.0;
    double commanded_vy = 0.0;
    unsigned long long sent_packets = 0;
    unsigned long long sent_packets_last = 0;
    double measured_send_rate = 0.0;
    std::chrono::steady_clock::time_point rate_sample{};
    LARGE_INTEGER perf_freq{};
    LARGE_INTEGER perf_start{};
    bool is_streaming = false;
    bool robot_addr_valid = false;
    int last_send_error = 0;
    std::string status_text = "Not connected";
    HWND hwnd_main = nullptr;
    HWND edit_ip = nullptr;
    HWND edit_robot_id = nullptr;
    HWND edit_send_hz = nullptr;
    HWND edit_radius = nullptr;
    HWND edit_omega = nullptr;
    HWND btn_apply = nullptr;
    HWND btn_connect = nullptr;
    HWND btn_disconnect = nullptr;
};

AppState* g_state = nullptr;

int MaxInt(int a, int b) {
    return (a > b) ? a : b;
}

double MaxDouble(double a, double b) {
    return (a > b) ? a : b;
}

int MinInt(int a, int b) {
    return (a < b) ? a : b;
}

double MinDouble(double a, double b) {
    return (a < b) ? a : b;
}

void PrintUsage() {
    std::printf(
        "Usage: testcpp.exe [robot_ip] [robot_id] [send_hz] [radius_m] [omega_rad_s]\n"
        "Example: testcpp.exe 172.20.10.2 1 200 0.6 1.0\n");
}

Config ParseArgs(int argc, char** argv) {
    Config cfg;
    if (argc >= 2) cfg.robot_ip = argv[1];
    if (argc >= 3) cfg.robot_id = MaxInt(0, std::atoi(argv[2]));
    if (argc >= 4) cfg.send_hz = MaxDouble(20.0, std::atof(argv[3]));
    if (argc >= 5) cfg.circle_radius_m = MaxDouble(0.05, std::atof(argv[4]));
    if (argc >= 6) cfg.angular_speed_rad_s = MaxDouble(0.05, std::atof(argv[5]));
    return cfg;
}

bool InitSockets(AppState& s) {
    WSADATA wsa_data{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        std::printf("WSAStartup failed\n");
        return false;
    }

    s.send_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    s.telemetry_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    s.vision_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s.send_sock == INVALID_SOCKET || s.telemetry_sock == INVALID_SOCKET || s.vision_sock == INVALID_SOCKET) {
        std::printf("socket create failed: %d\n", WSAGetLastError());
        return false;
    }

    u_long non_block = 1;
    ioctlsocket(s.telemetry_sock, FIONBIO, &non_block);
    ioctlsocket(s.vision_sock, FIONBIO, &non_block);

    int sndbuf = 1 << 20;
    setsockopt(s.send_sock, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char*>(&sndbuf), sizeof(sndbuf));

    sockaddr_in bind_tm{};
    bind_tm.sin_family = AF_INET;
    bind_tm.sin_addr.s_addr = INADDR_ANY;
    bind_tm.sin_port = htons(kTelemetryPort);
    if (bind(s.telemetry_sock, reinterpret_cast<sockaddr*>(&bind_tm), sizeof(bind_tm)) == SOCKET_ERROR) {
        std::printf("telemetry bind failed on %d: %d\n", kTelemetryPort, WSAGetLastError());
        return false;
    }

    sockaddr_in bind_vs{};
    bind_vs.sin_family = AF_INET;
    bind_vs.sin_addr.s_addr = INADDR_ANY;
    bind_vs.sin_port = htons(kVisionPort);
    if (bind(s.vision_sock, reinterpret_cast<sockaddr*>(&bind_vs), sizeof(bind_vs)) == SOCKET_ERROR) {
        std::printf("vision bind failed on %d: %d\n", kVisionPort, WSAGetLastError());
        return false;
    }

    return true;
}

void CloseSockets(AppState& s) {
    if (s.send_sock != INVALID_SOCKET) closesocket(s.send_sock);
    if (s.telemetry_sock != INVALID_SOCKET) closesocket(s.telemetry_sock);
    if (s.vision_sock != INVALID_SOCKET) closesocket(s.vision_sock);
    WSACleanup();
}

double ElapsedSeconds(const AppState& s) {
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    return static_cast<double>(now.QuadPart - s.perf_start.QuadPart) / static_cast<double>(s.perf_freq.QuadPart);
}

bool UpdateRobotAddress(AppState& s) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(kCommandPort);
    if (inet_pton(AF_INET, s.cfg.robot_ip.c_str(), &addr.sin_addr) != 1) {
        s.robot_addr_valid = false;
        s.status_text = "Invalid IP address";
        return false;
    }
    s.robot_addr = addr;
    s.robot_addr_valid = true;
    return true;
}

std::string GetWindowTextString(HWND h) {
    char buf[256];
    const int n = GetWindowTextA(h, buf, sizeof(buf));
    if (n <= 0) return "";
    return std::string(buf, static_cast<size_t>(n));
}

void SetUiFromConfig(AppState& s) {
    if (!s.edit_ip) return;
    SetWindowTextA(s.edit_ip, s.cfg.robot_ip.c_str());

    char tmp[64];
    std::snprintf(tmp, sizeof(tmp), "%d", s.cfg.robot_id);
    SetWindowTextA(s.edit_robot_id, tmp);
    std::snprintf(tmp, sizeof(tmp), "%.3f", s.cfg.send_hz);
    SetWindowTextA(s.edit_send_hz, tmp);
    std::snprintf(tmp, sizeof(tmp), "%.3f", s.cfg.circle_radius_m);
    SetWindowTextA(s.edit_radius, tmp);
    std::snprintf(tmp, sizeof(tmp), "%.3f", s.cfg.angular_speed_rad_s);
    SetWindowTextA(s.edit_omega, tmp);
}

void UpdateConnectionStatus(AppState& s) {
    const auto now = std::chrono::steady_clock::now();
    const long long telem_age = s.telemetry.valid
        ? std::chrono::duration_cast<std::chrono::milliseconds>(now - s.telemetry.last_rx).count()
        : 999999;

    if (!s.is_streaming) {
        s.status_text = "Disconnected";
    } else if (!s.robot_addr_valid) {
        s.status_text = "Streaming blocked: invalid IP";
    } else if (!s.telemetry.valid) {
        s.status_text = "Streaming commands, waiting telemetry...";
    } else if (telem_age > 1500) {
        s.status_text = "Streaming, telemetry stale";
    } else {
        s.status_text = "Connected (command + telemetry active)";
    }
}

bool ApplyConfigFromUi(AppState& s) {
    s.cfg.robot_ip = GetWindowTextString(s.edit_ip);
    if (s.cfg.robot_ip.empty()) {
        s.status_text = "IP cannot be empty";
        return false;
    }

    s.cfg.robot_id = MaxInt(0, std::atoi(GetWindowTextString(s.edit_robot_id).c_str()));
    s.cfg.send_hz = MaxDouble(20.0, std::atof(GetWindowTextString(s.edit_send_hz).c_str()));
    s.cfg.send_hz = MinDouble(s.cfg.send_hz, 1000.0);
    s.cfg.circle_radius_m = MaxDouble(0.05, std::atof(GetWindowTextString(s.edit_radius).c_str()));
    s.cfg.circle_radius_m = MinDouble(s.cfg.circle_radius_m, 3.0);
    s.cfg.angular_speed_rad_s = MaxDouble(0.05, std::atof(GetWindowTextString(s.edit_omega).c_str()));
    s.cfg.angular_speed_rad_s = MinDouble(s.cfg.angular_speed_rad_s, 8.0);

    if (!UpdateRobotAddress(s)) {
        return false;
    }
    s.status_text = "Parameters applied";
    return true;
}

void CreateControls(AppState& s, HINSTANCE hinst) {
    const DWORD edit_style = WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL;
    const DWORD btn_style = WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON;
    const DWORD lbl_style = WS_CHILD | WS_VISIBLE;

    CreateWindowExA(0, "STATIC", "Robot IP", lbl_style, 12, 10, 80, 20, s.hwnd_main, nullptr, hinst, nullptr);
    s.edit_ip = CreateWindowExA(0, "EDIT", "", edit_style, 90, 8, 150, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdEditIp), hinst, nullptr);

    CreateWindowExA(0, "STATIC", "ID", lbl_style, 250, 10, 30, 20, s.hwnd_main, nullptr, hinst, nullptr);
    s.edit_robot_id = CreateWindowExA(0, "EDIT", "", edit_style, 280, 8, 45, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdEditRobotId), hinst, nullptr);

    CreateWindowExA(0, "STATIC", "Hz", lbl_style, 335, 10, 30, 20, s.hwnd_main, nullptr, hinst, nullptr);
    s.edit_send_hz = CreateWindowExA(0, "EDIT", "", edit_style, 365, 8, 70, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdEditSendHz), hinst, nullptr);

    CreateWindowExA(0, "STATIC", "Radius", lbl_style, 445, 10, 55, 20, s.hwnd_main, nullptr, hinst, nullptr);
    s.edit_radius = CreateWindowExA(0, "EDIT", "", edit_style, 500, 8, 70, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdEditRadius), hinst, nullptr);

    CreateWindowExA(0, "STATIC", "Omega", lbl_style, 580, 10, 50, 20, s.hwnd_main, nullptr, hinst, nullptr);
    s.edit_omega = CreateWindowExA(0, "EDIT", "", edit_style, 635, 8, 70, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdEditOmega), hinst, nullptr);

    s.btn_apply = CreateWindowExA(0, "BUTTON", "Apply", btn_style, 720, 8, 80, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdBtnApply), hinst, nullptr);
    s.btn_connect = CreateWindowExA(0, "BUTTON", "Connect", btn_style, 810, 8, 90, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdBtnConnect), hinst, nullptr);
    s.btn_disconnect = CreateWindowExA(0, "BUTTON", "Disconnect", btn_style, 910, 8, 100, 24, s.hwnd_main,
        reinterpret_cast<HMENU>(kIdBtnDisconnect), hinst, nullptr);

    SetUiFromConfig(s);
}

std::optional<double> ParseKVDouble(const std::string& payload, const char* key) {
    const std::string token = std::string(key) + "=";
    const auto pos = payload.find(token);
    if (pos == std::string::npos) return std::nullopt;
    auto end = payload.find(',', pos);
    const auto start = pos + token.size();
    if (end == std::string::npos) end = payload.size();
    try {
        return std::stod(payload.substr(start, end - start));
    } catch (...) {
        return std::nullopt;
    }
}

void PollTelemetry(AppState& s) {
    std::array<char, 2048> buffer{};
    for (;;) {
        const int n = recv(s.telemetry_sock, buffer.data(), static_cast<int>(buffer.size() - 1), 0);
        if (n == SOCKET_ERROR) {
            const int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) break;
            break;
        }
        if (n <= 0) break;

        buffer[static_cast<size_t>(n)] = '\0';
        std::string payload(buffer.data());

        const auto voltage = ParseKVDouble(payload, "voltage");
        const auto ball = ParseKVDouble(payload, "ball");
        const auto bearing = ParseKVDouble(payload, "bearing");
        const auto conf = ParseKVDouble(payload, "conf");
        const auto ts = ParseKVDouble(payload, "ts_ms");

        if (voltage) s.telemetry.voltage = *voltage;
        if (ball) s.telemetry.ball_found = (*ball > 0.5);
        if (bearing) s.telemetry.bearing = *bearing;
        if (conf) s.telemetry.confidence = *conf;
        if (ts) s.telemetry.robot_ts_ms = static_cast<long long>(*ts);
        s.telemetry.valid = true;
        s.telemetry.last_rx = std::chrono::steady_clock::now();
        s.telemetry.packets++;
    }
}

void PollVision(AppState& s) {
    std::array<char, 512> buffer{};
    for (;;) {
        const int n = recv(s.vision_sock, buffer.data(), static_cast<int>(buffer.size() - 1), 0);
        if (n == SOCKET_ERROR) {
            const int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) break;
            break;
        }
        if (n <= 0) break;

        buffer[static_cast<size_t>(n)] = '\0';
        std::istringstream iss(buffer.data());
        double x = 0.0;
        double y = 0.0;
        long long ts = 0;
        if (iss >> x >> y) {
            iss >> ts;
            s.vision.valid = true;
            s.vision.x = x;
            s.vision.y = y;
            s.vision.source_ts = ts;
            s.vision.last_rx = std::chrono::steady_clock::now();
        }
    }
}

void SendCircleCommand(AppState& s) {
    if (!s.is_streaming || !s.robot_addr_valid) return;

    const double t = ElapsedSeconds(s);
    const double r = s.cfg.circle_radius_m;
    const double w = s.cfg.angular_speed_rad_s;

    s.commanded_x = r * std::cos(w * t);
    s.commanded_y = r * std::sin(w * t);
    s.commanded_vx = -r * w * std::sin(w * t);
    s.commanded_vy = r * w * std::cos(w * t);

    auto now = std::chrono::system_clock::now();
    const auto epoch = now.time_since_epoch();
    const double unix_time = std::chrono::duration_cast<std::chrono::duration<double>>(epoch).count();

    char msg[256];
    const int len = std::snprintf(
        msg, sizeof(msg), "%d %.6f %.6f %.6f %d %d %.6f",
        s.cfg.robot_id, s.commanded_vx, s.commanded_vy, 0.0, 0, 0, unix_time);
    if (len <= 0) return;

    const int sent = sendto(s.send_sock, msg, len, 0, reinterpret_cast<sockaddr*>(&s.robot_addr), sizeof(s.robot_addr));
    if (sent == SOCKET_ERROR) {
        s.last_send_error = WSAGetLastError();
        return;
    }
    s.sent_packets++;
}

POINT WorldToScreen(double x, double y, RECT rc, double half_range_m) {
    const double cx = (rc.left + rc.right) * 0.5;
    const double cy = (rc.top + rc.bottom) * 0.5;
    const double scale = static_cast<double>(MinInt(rc.right - rc.left, rc.bottom - rc.top)) * 0.45 / half_range_m;
    POINT p{};
    p.x = static_cast<LONG>(std::lround(cx + x * scale));
    p.y = static_cast<LONG>(std::lround(cy - y * scale));
    return p;
}

void DrawCircleMarker(HDC hdc, const POINT& p, int radius, COLORREF color) {
    HBRUSH brush = CreateSolidBrush(color);
    HBRUSH old = static_cast<HBRUSH>(SelectObject(hdc, brush));
    HPEN pen = CreatePen(PS_SOLID, 1, color);
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, pen));
    Ellipse(hdc, p.x - radius, p.y - radius, p.x + radius, p.y + radius);
    SelectObject(hdc, old_pen);
    SelectObject(hdc, old);
    DeleteObject(pen);
    DeleteObject(brush);
}

void DrawScene(HDC hdc, RECT rc, AppState& s) {
    FillRect(hdc, &rc, static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));

    constexpr double kHalfRange = 1.5;
    RECT field_rc = rc;
    field_rc.top += kControlPanelHeight;
    const POINT center = WorldToScreen(0.0, 0.0, field_rc, kHalfRange);
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(55, 55, 55));
    HPEN old_pen = static_cast<HPEN>(SelectObject(hdc, grid_pen));
    MoveToEx(hdc, field_rc.left, center.y, nullptr);
    LineTo(hdc, field_rc.right, center.y);
    MoveToEx(hdc, center.x, field_rc.top, nullptr);
    LineTo(hdc, center.x, field_rc.bottom);
    SelectObject(hdc, old_pen);
    DeleteObject(grid_pen);

    if (s.command_trail.size() > 1) {
        HPEN trail_pen = CreatePen(PS_SOLID, 1, RGB(0, 100, 255));
        old_pen = static_cast<HPEN>(SelectObject(hdc, trail_pen));
        MoveToEx(hdc, s.command_trail[0].x, s.command_trail[0].y, nullptr);
        for (size_t i = 1; i < s.command_trail.size(); ++i) {
            LineTo(hdc, s.command_trail[i].x, s.command_trail[i].y);
        }
        SelectObject(hdc, old_pen);
        DeleteObject(trail_pen);
    }

    const POINT cmd = WorldToScreen(s.commanded_x, s.commanded_y, field_rc, kHalfRange);
    DrawCircleMarker(hdc, cmd, 6, RGB(0, 255, 0));

    if (s.vision.valid) {
        const POINT vis = WorldToScreen(s.vision.x, s.vision.y, field_rc, kHalfRange);
        DrawCircleMarker(hdc, vis, 6, RGB(255, 140, 0));

        HPEN delta_pen = CreatePen(PS_DOT, 1, RGB(255, 200, 0));
        old_pen = static_cast<HPEN>(SelectObject(hdc, delta_pen));
        MoveToEx(hdc, cmd.x, cmd.y, nullptr);
        LineTo(hdc, vis.x, vis.y);
        SelectObject(hdc, old_pen);
        DeleteObject(delta_pen);
    }

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(220, 220, 220));

    const auto now = std::chrono::steady_clock::now();
    const auto telem_age_ms = s.telemetry.valid
        ? std::chrono::duration_cast<std::chrono::milliseconds>(now - s.telemetry.last_rx).count()
        : -1;
    const auto vision_age_ms = s.vision.valid
        ? std::chrono::duration_cast<std::chrono::milliseconds>(now - s.vision.last_rx).count()
        : -1;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "Status: " << s.status_text
        << (s.last_send_error ? ("  last_send_error=" + std::to_string(s.last_send_error)) : "")
        << "\n"
        << "Robot: " << s.cfg.robot_ip << ":" << kCommandPort
        << "  id=" << s.cfg.robot_id
        << "  sendHz(target/measured)=" << s.cfg.send_hz << "/" << s.measured_send_rate << "\n"
        << "Cmd pos(x,y)=(" << s.commanded_x << ", " << s.commanded_y << ") m"
        << "  cmd vel(vx,vy)=(" << s.commanded_vx << ", " << s.commanded_vy << ") m/s\n"
        << "Sent packets=" << s.sent_packets
        << "  Telemetry packets=" << s.telemetry.packets
        << "  telemetry age ms=" << telem_age_ms << "\n"
        << "Telemetry: voltage=" << s.telemetry.voltage
        << "  ball=" << (s.telemetry.ball_found ? "1" : "0")
        << "  bearing=" << s.telemetry.bearing
        << "  conf=" << s.telemetry.confidence
        << "  robot_ts_ms=" << s.telemetry.robot_ts_ms << "\n"
        << "Vision UDP (" << kVisionPort << "): " << (s.vision.valid ? "connected" : "waiting")
        << "  age ms=" << vision_age_ms
        << "  pose=(" << s.vision.x << ", " << s.vision.y << ") m\n"
        << "Legend: green=commanded, orange=vision";

    const std::string text = oss.str();
    TextOutA(hdc, 16, 16, text.c_str(), static_cast<int>(text.size()));
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    switch (msg) {
        case WM_COMMAND: {
            if (!g_state) return 0;
            const int id = LOWORD(w_param);
            if (id == kIdBtnApply) {
                ApplyConfigFromUi(*g_state);
                return 0;
            }
            if (id == kIdBtnConnect) {
                if (ApplyConfigFromUi(*g_state)) {
                    g_state->is_streaming = true;
                    g_state->last_send_error = 0;
                    g_state->status_text = "Connecting...";
                }
                return 0;
            }
            if (id == kIdBtnDisconnect) {
                g_state->is_streaming = false;
                g_state->status_text = "Disconnected";
                return 0;
            }
            return 0;
        }
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProc(hwnd, msg, w_param, l_param);
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--help") {
        PrintUsage();
        return 0;
    }

    AppState state{};
    state.cfg = ParseArgs(argc, argv);
    g_state = &state;

    QueryPerformanceFrequency(&state.perf_freq);
    QueryPerformanceCounter(&state.perf_start);

    if (!InitSockets(state)) {
        CloseSockets(state);
        return 1;
    }

    const HINSTANCE hinst = GetModuleHandle(nullptr);
    const char* class_name = "LatencyCircleWindow";
    WNDCLASSA wc{};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hinst;
    wc.lpszClassName = class_name;
    wc.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(
        0, class_name, "Robot Latency Visualizer (stripped C++ server)",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT, 1200, 840,
        nullptr, nullptr, hinst, nullptr);
    if (!hwnd) {
        std::printf("window creation failed\n");
        CloseSockets(state);
        return 1;
    }
    state.hwnd_main = hwnd;
    CreateControls(state, hinst);
    UpdateRobotAddress(state);

    MSG message{};
    auto last_send = std::chrono::steady_clock::now();
    auto last_frame = std::chrono::steady_clock::now();
    state.rate_sample = last_send;
    const auto frame_period = std::chrono::milliseconds(16);

    bool running = true;
    while (running) {
        while (PeekMessage(&message, nullptr, 0, 0, PM_REMOVE)) {
            if (message.message == WM_QUIT) {
                running = false;
                break;
            }
            TranslateMessage(&message);
            DispatchMessage(&message);
        }
        if (!running) break;

        const auto now = std::chrono::steady_clock::now();
        const auto send_period = std::chrono::duration<double>(1.0 / state.cfg.send_hz);
        if (now - last_send >= send_period) {
            SendCircleCommand(state);
            last_send = now;
        }

        PollTelemetry(state);
        PollVision(state);

        if (now - state.rate_sample >= std::chrono::seconds(1)) {
            const auto delta = state.sent_packets - state.sent_packets_last;
            state.measured_send_rate = static_cast<double>(delta);
            state.sent_packets_last = state.sent_packets;
            state.rate_sample = now;
        }

        UpdateConnectionStatus(state);

        RECT rc{};
        GetClientRect(hwnd, &rc);
        RECT field_rc = rc;
        field_rc.top += kControlPanelHeight;
        POINT p = WorldToScreen(state.commanded_x, state.commanded_y, field_rc, 1.5);
        state.command_trail.push_back(p);
        if (state.command_trail.size() > 900) state.command_trail.pop_front();

        if (now - last_frame >= frame_period) {
            HDC dc = GetDC(hwnd);
            DrawScene(dc, rc, state);
            ReleaseDC(hwnd, dc);
            last_frame = now;
        }

        Sleep(1);
    }

    CloseSockets(state);
    return 0;
}
