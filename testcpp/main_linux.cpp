#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <sstream>
#include <string>

namespace {
constexpr int kCommandPort = 50514;
constexpr int kTelemetryPort = 50513;

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

volatile sig_atomic_t g_running = 1;

void OnSigInt(int) { g_running = 0; }

int MaxInt(int a, int b) { return (a > b) ? a : b; }
double MaxDouble(double a, double b) { return (a > b) ? a : b; }

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

Config ParseArgs(int argc, char** argv) {
    Config cfg;
    if (argc >= 2) cfg.robot_ip = argv[1];
    if (argc >= 3) cfg.robot_id = MaxInt(0, std::atoi(argv[2]));
    if (argc >= 4) cfg.send_hz = MaxDouble(20.0, std::atof(argv[3]));
    if (argc >= 5) cfg.circle_radius_m = MaxDouble(0.05, std::atof(argv[4]));
    if (argc >= 6) cfg.angular_speed_rad_s = MaxDouble(0.05, std::atof(argv[5]));
    return cfg;
}

}  // namespace

int main(int argc, char** argv) {
    Config cfg = ParseArgs(argc, argv);

    std::signal(SIGINT, OnSigInt);

    const int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    const int telemetry_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0 || telemetry_sock < 0) {
        std::perror("socket");
        return 1;
    }

    int flags = fcntl(telemetry_sock, F_GETFL, 0);
    fcntl(telemetry_sock, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in robot_addr{};
    robot_addr.sin_family = AF_INET;
    robot_addr.sin_port = htons(kCommandPort);
    if (inet_pton(AF_INET, cfg.robot_ip.c_str(), &robot_addr.sin_addr) != 1) {
        std::fprintf(stderr, "Invalid robot IP: %s\n", cfg.robot_ip.c_str());
        return 1;
    }

    sockaddr_in local_tm{};
    local_tm.sin_family = AF_INET;
    local_tm.sin_addr.s_addr = INADDR_ANY;
    local_tm.sin_port = htons(kTelemetryPort);
    if (bind(telemetry_sock, reinterpret_cast<sockaddr*>(&local_tm), sizeof(local_tm)) < 0) {
        std::perror("bind telemetry");
        return 1;
    }

    TelemetryState telemetry{};
    unsigned long long sent_packets = 0;
    double measured_hz = 0.0;

    const auto start = std::chrono::steady_clock::now();
    auto last_send = start;
    auto last_hz_sample = start;
    unsigned long long sent_last = 0;
    auto last_print = start;

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();
        const auto t = std::chrono::duration<double>(now - start).count();

        const auto send_period = std::chrono::duration<double>(1.0 / cfg.send_hz);
        if (now - last_send >= send_period) {
            const double r = cfg.circle_radius_m;
            const double w = cfg.angular_speed_rad_s;
            const double vx = -r * w * std::sin(w * t);
            const double vy = r * w * std::cos(w * t);

            auto sys_now = std::chrono::system_clock::now();
            double unix_time = std::chrono::duration_cast<std::chrono::duration<double>>(sys_now.time_since_epoch()).count();

            char msg[256];
            int len = std::snprintf(msg, sizeof(msg), "%d %.6f %.6f %.6f %d %d %.6f",
                                    cfg.robot_id, vx, vy, 0.0, 0, 0, unix_time);
            if (len > 0) {
                sendto(send_sock, msg, static_cast<size_t>(len), 0, reinterpret_cast<sockaddr*>(&robot_addr), sizeof(robot_addr));
                sent_packets++;
            }
            last_send = now;
        }

        std::array<char, 2048> buf{};
        for (;;) {
            int n = recv(telemetry_sock, buf.data(), buf.size() - 1, 0);
            if (n <= 0) break;
            buf[static_cast<size_t>(n)] = '\0';
            std::string payload(buf.data());

            auto v = ParseKVDouble(payload, "voltage");
            auto b = ParseKVDouble(payload, "ball");
            auto be = ParseKVDouble(payload, "bearing");
            auto c = ParseKVDouble(payload, "conf");
            auto ts = ParseKVDouble(payload, "ts_ms");
            if (v) telemetry.voltage = *v;
            if (b) telemetry.ball_found = (*b > 0.5);
            if (be) telemetry.bearing = *be;
            if (c) telemetry.confidence = *c;
            if (ts) telemetry.robot_ts_ms = static_cast<long long>(*ts);
            telemetry.valid = true;
            telemetry.last_rx = now;
            telemetry.packets++;
        }

        if (now - last_hz_sample >= std::chrono::seconds(1)) {
            measured_hz = static_cast<double>(sent_packets - sent_last);
            sent_last = sent_packets;
            last_hz_sample = now;
        }

        if (now - last_print >= std::chrono::milliseconds(250)) {
            long long telem_age = telemetry.valid
                ? std::chrono::duration_cast<std::chrono::milliseconds>(now - telemetry.last_rx).count()
                : -1;
            std::printf(
                "\rIP=%s id=%d sendHz(target/measured)=%.1f/%.1f sent=%llu telem=%llu ageMs=%lld voltage=%.2f ball=%d conf=%.2f      ",
                cfg.robot_ip.c_str(), cfg.robot_id, cfg.send_hz, measured_hz,
                sent_packets, telemetry.packets, telem_age, telemetry.voltage,
                telemetry.ball_found ? 1 : 0, telemetry.confidence);
            std::fflush(stdout);
            last_print = now;
        }

        usleep(1000);
    }

    std::puts("\nStopping...");
    close(send_sock);
    close(telemetry_sock);
    return 0;
}
