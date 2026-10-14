#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> running{true};
std::atomic<int> listening_fd{-1};
std::mutex frame_mutex;
std::condition_variable frame_cv;
std::vector<unsigned char> current_jpeg;
std::uint64_t frame_sequence = 0;
double measured_fps = 0.0;
std::string serial_number = "starting";

void stop_handler(int) {
    running.store(false);
    const int fd = listening_fd.exchange(-1);
    if (fd >= 0) ::close(fd);
}

bool send_all(int fd, const void* data, std::size_t size) {
    auto* ptr = static_cast<const unsigned char*>(data);
    while (size && running.load()) {
        const ssize_t sent = ::send(fd, ptr, size, MSG_NOSIGNAL);
        if (sent <= 0) return false;
        ptr += sent;
        size -= static_cast<std::size_t>(sent);
    }
    return size == 0;
}

bool send_text(int fd, const std::string& text) {
    return send_all(fd, text.data(), text.size());
}

const char* html_page = R"HTML(<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>D435i RGB + Depth</title>
<style>
html,body{height:100%;margin:0;background:#0b1117;color:#e7eef5;font-family:system-ui,sans-serif}
body{display:flex;flex-direction:column}header{padding:10px 16px;background:#17222d;font-size:18px;font-weight:650}
main{flex:1;min-height:0;display:flex;align-items:center;justify-content:center;padding:8px}
img{max-width:100%;max-height:100%;object-fit:contain;background:#000;border-radius:6px}
footer{padding:6px;text-align:center;color:#8899a8;font-size:12px}
</style></head><body>
<header>Intel RealSense D435i · RGB + Depth</header>
<main><img id="view" src="/stream.mjpg" alt="D435i stream"></main>
<footer>左侧 RGB · 右侧深度伪彩色 · 640×360 @ 30 FPS</footer>
<script>const v=document.getElementById('view');v.onerror=()=>setTimeout(()=>v.src='/stream.mjpg?t='+Date.now(),1000);</script>
</body></html>)HTML";

void capture_frames() {
    try {
        rs2::pipeline pipeline;
        rs2::config config;
        config.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 30);
        config.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_BGR8, 30);
        const auto profile = pipeline.start(config);
        const auto device = profile.get_device();
        if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
            serial_number = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        rs2::colorizer colorizer;
        colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 0.0f);
        auto fps_start = std::chrono::steady_clock::now();
        int fps_count = 0;

        while (running.load()) {
            const rs2::frameset frames = pipeline.wait_for_frames(1000);
            const rs2::video_frame color = frames.get_color_frame();
            const rs2::depth_frame depth = frames.get_depth_frame();
            if (!color || !depth) continue;
            const rs2::video_frame depth_color = colorizer.colorize(depth);

            cv::Mat rgb(cv::Size(color.get_width(), color.get_height()), CV_8UC3,
                        const_cast<void*>(color.get_data()), color.get_stride_in_bytes());
            cv::Mat depth_rgb(cv::Size(depth_color.get_width(), depth_color.get_height()), CV_8UC3,
                              const_cast<void*>(depth_color.get_data()), depth_color.get_stride_in_bytes());
            cv::Mat depth_bgr;
            cv::cvtColor(depth_rgb, depth_bgr, cv::COLOR_RGB2BGR);
            cv::Mat combined;
            cv::hconcat(rgb, depth_bgr, combined);
            cv::putText(combined, "RGB", {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.75,
                        {255,255,255}, 2, cv::LINE_AA);
            cv::putText(combined, "DEPTH", {652, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.75,
                        {255,255,255}, 2, cv::LINE_AA);

            std::vector<unsigned char> encoded;
            if (!cv::imencode(".jpg", combined, encoded, {cv::IMWRITE_JPEG_QUALITY, 72})) continue;

            ++fps_count;
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - fps_start).count();
            if (elapsed >= 1.0) {
                measured_fps = fps_count / elapsed;
                fps_count = 0;
                fps_start = now;
                std::cout << "streaming " << measured_fps << " FPS, serial " << serial_number << std::endl;
            }
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                current_jpeg.swap(encoded);
                ++frame_sequence;
            }
            frame_cv.notify_all();
        }
        pipeline.stop();
    } catch (const std::exception& error) {
        std::cerr << "camera error: " << error.what() << std::endl;
        running.store(false);
    }
    frame_cv.notify_all();
}

void stream_client(int fd) {
    if (!send_text(fd,
        "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n")) return;
    std::uint64_t previous = 0;
    while (running.load()) {
        std::vector<unsigned char> jpeg;
        {
            std::unique_lock<std::mutex> lock(frame_mutex);
            frame_cv.wait_for(lock, std::chrono::seconds(2), [&]{
                return frame_sequence != previous || !running.load();
            });
            if (!running.load()) break;
            if (frame_sequence == previous || current_jpeg.empty()) continue;
            jpeg = current_jpeg;
            previous = frame_sequence;
        }
        std::ostringstream part;
        part << "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
             << jpeg.size() << "\r\n\r\n";
        if (!send_text(fd, part.str()) || !send_all(fd, jpeg.data(), jpeg.size()) ||
            !send_text(fd, "\r\n")) break;
    }
}

void handle_client(int fd) {
    char buffer[4096];
    const ssize_t count = ::recv(fd, buffer, sizeof(buffer)-1, 0);
    if (count <= 0) { ::close(fd); return; }
    buffer[count] = '\0';
    const std::string request(buffer);
    if (request.find("GET /stream.mjpg ") != std::string::npos) {
        stream_client(fd);
    } else if (request.find("GET / ") != std::string::npos) {
        const std::string body(html_page);
        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                 << "Cache-Control: no-store\r\nContent-Length: " << body.size()
                 << "\r\nConnection: close\r\n\r\n" << body;
        send_text(fd, response.str());
    } else {
        send_text(fd, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    }
    ::close(fd);
}

int open_server(int port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) throw std::runtime_error("socket failed");
    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        const std::string message = std::string("bind failed: ") + std::strerror(errno);
        ::close(fd); throw std::runtime_error(message);
    }
    if (::listen(fd, 8) < 0) { ::close(fd); throw std::runtime_error("listen failed"); }
    return fd;
}
} // namespace

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::stoi(argv[1]) : 8765;
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    std::signal(SIGPIPE, SIG_IGN);
    try {
        std::thread capture(capture_frames);
        const int server = open_server(port);
        listening_fd.store(server);
        std::cout << "Open http://127.0.0.1:" << port << std::endl;
        while (running.load()) {
            const int client = ::accept(server, nullptr, nullptr);
            if (client < 0) continue;
            std::thread(handle_client, client).detach();
        }
        if (capture.joinable()) capture.join();
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << std::endl;
        running.store(false);
        return 1;
    }
    return 0;
}
