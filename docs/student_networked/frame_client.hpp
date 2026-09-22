// ─────────────────────────────────────────────────────────────────────────
//  frame_client.hpp  - 로봇(Jetson) 프레임 스트리밍 서버에서 WLAN(TCP)으로
//  컬러 + 뎁스 프레임을 받아 cv::Mat 으로 복원하는 Windows 클라이언트.
//
//  ■ 서버: ros2_ws/scripts/bridge/frame_stream_server.py (로봇 측에서 실행)
//
//  ■ 사용법:
//     FrameClient cam;                                                        : 객체 생성(자) 
//     if (!cam.connect("192.168.0.10", 5001)) { ... }   // 로봇 IP            : ip 접속
//     cv::Mat image_bgr, depth_f32;  FrameClient::Intr K;                     : 정보 상자
//     if (cam.grab(image_bgr, depth_f32, K))                                  : 정보 수신
//      {
//         // image_bgr : CV_8UC3 (BGR)                                        : 1
//         // depth_f32 : CV_32FC1, 단위 mm (무효 픽셀 = NaN, 기존 ZED와 동일) : 2
//         // K.fx, K.fy, K.cx, K.cy : 컬러 카메라                             : 3
//      }
//
//  ■ 빌드: ZED SDK - X / OpenCV 링크 - O + ws2_32 링크 - O                   : ZED는 필요 없고, 윈도우 통신 필요!
// ─────────────────────────────────────────────────────────────────────────
#pragma once
#ifndef WIN32_LEAN_AND_MEAN         //(1)여러 소켓 통신 등 불필요 -> windows 헤더 축소 경량 전처리
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX                    //(2)windows.h min/max 매크로 비활성 -> std::min/max 등 api함수 보호
#define NOMINMAX
#endif
#include <winsock2.h>               //(3)★winsock2.h로 단일 소켓으로 명시해 가져와 충돌 방지(추후 torch.h가 끌어올 windows.h보다 먼저 선점)
#include <ws2tcpip.h>               //(4)아이피 같은거 입력시 인식시키기!
#pragma comment(lib, "ws2_32.lib")  //(5)이제 실제 소켓 통신 코드 부르기!
#include <opencv2/opencv.hpp>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <limits>
#include <iostream>

class FrameClient
{
public:                                                                                                         //(6) cpp가 직접 쓰는 것
    struct Intr { float fx = 0, fy = 0, cx = 0, cy = 0; };

    FrameClient() { ensureWsa(); }
    ~FrameClient() { disconnect(); }

    bool connect(const std::string& ip, int port)
    {
        disconnect();
        sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock_ == INVALID_SOCKET) return false;                                                              //(6-1) 연결되지 않았으니, 실패!
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons((u_short)port);
        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) { disconnect(); return false; }
        if (::connect(sock_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) { disconnect(); return false; }
        int one = 1;
        setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        DWORD timeout = 5000;  // ★ 5초 이상 응답 없으면 recv()가 무한대기 대신 실패로 리턴하게 함
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
        return true;
    }
    void disconnect()
    {
        if (sock_ != INVALID_SOCKET) { closesocket(sock_); sock_ = INVALID_SOCKET; }                            //(6-2) 연결되었으니, 청소후 연결안된것!
    }
    bool connected() const { return sock_ != INVALID_SOCKET; }                                                  //(6-3) 연결되지 않았으면, 실행!(6-1 재호출)

    // ★최신 한 프레임 요청 → 수신 성공 => image_bgr / depth_f32(mm) / K 채움.
    bool grab(cv::Mat& image_bgr, cv::Mat& depth_f32, Intr& K) //image_bgr / depth_f32(mm) / K 채움.
    {
        if (sock_ == INVALID_SOCKET) { std::cerr << "[grab] fail: not connected" << std::endl; return false; }  //(7-1) 연결되지 않았으니, 실패!
        const char req = 'G';
        if (!sendAll(&req, 1)) { std::cerr << "[grab] fail: send('G') 실패 -> disconnect" << std::endl; disconnect(); return false; }
        uint8_t hdr[HEADER_SIZE];
        if (!recvAll(hdr, HEADER_SIZE)) { std::cerr << "[grab] fail: header recv 실패 -> disconnect" << std::endl; disconnect(); return false; }
        size_t off = 0;
        if (std::memcmp(hdr, "RBF1", 4) != 0)
        {
            std::cerr << "[grab] fail: magic mismatch (got '" << (char)hdr[0] << (char)hdr[1] << (char)hdr[2] << (char)hdr[3] << "') -> 프로토콜 어긋남, disconnect" << std::endl;
            disconnect();  // ★ 어긋난 채로 계속 재시도해봐야 계속 어긋나므로 연결을 끊어 재동기화
            return false;
        }
        off = 4;
        /*uint32_t frame_id =*/ readU32(hdr, off);
        K.fx = readF32(hdr, off); K.fy = readF32(hdr, off);
        K.cx = readF32(hdr, off); K.cy = readF32(hdr, off);
        /*uint32_t w =*/ readU32(hdr, off);
        /*uint32_t h =*/ readU32(hdr, off);
        float depth_scale = readF32(hdr, off);           // raw 1단위 → mm
        uint32_t color_len = readU32(hdr, off);
        uint32_t depth_len = readU32(hdr, off);
        if (color_len == 0 || depth_len == 0) { std::cerr << "[grab] 서버에 아직 프레임 없음 (color_len/depth_len=0)" << std::endl; return false; }
        if (color_len > 50'000'000 || depth_len > 50'000'000)
        {
            std::cerr << "[grab] fail: color_len=" << color_len << " depth_len=" << depth_len << " 비정상적으로 큼 -> 프로토콜 어긋남, disconnect" << std::endl;
            disconnect();
            return false;
        }
        std::vector<uint8_t> cbuf(color_len), dbuf(depth_len);
        if (!recvAll(cbuf.data(), color_len)) { std::cerr << "[grab] fail: color 본문 recv 실패 -> disconnect" << std::endl; disconnect(); return false; }
        if (!recvAll(dbuf.data(), depth_len)) { std::cerr << "[grab] fail: depth 본문 recv 실패 -> disconnect" << std::endl; disconnect(); return false; }
        // 컬러 디코드 (JPEG → BGR)
        image_bgr = cv::imdecode(cbuf, cv::IMREAD_COLOR);
        if (image_bgr.empty()) { std::cerr << "[grab] fail: color imdecode 실패 (color_len=" << color_len << ")" << std::endl; return false; }
        // 뎁스 디코드 (16bit PNG → CV_16UC1) → mm 실수화, 0=무효→NaN
        cv::Mat depth16 = cv::imdecode(dbuf, cv::IMREAD_UNCHANGED);
        if (depth16.empty() || depth16.type() != CV_16UC1)
        {
            std::cerr << "[grab] fail: depth imdecode 실패 (depth_len=" << depth_len << ", type=" << (depth16.empty() ? -1 : depth16.type()) << ")" << std::endl;
            return false;
        }
        cv::Mat invalid = (depth16 == 0);
        depth16.convertTo(depth_f32, CV_32FC1, depth_scale);  // mm
        depth_f32.setTo(std::numeric_limits<float>::quiet_NaN(), invalid);
        return true;
    }

private:                                                //(8) public이 직접 쓰는 것(cpp에서 public을 연달아 쓸때 private가 중복적으로 쓰면, 지역적으로 할당!)
    static const int HEADER_SIZE = 44;
    SOCKET sock_ = INVALID_SOCKET;                      //(8-1) 연결되지않은 초기화!
    static void ensureWsa()
    {
        static bool inited = false;
        if (!inited) { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); inited = true; }
    }
    bool sendAll(const char* p, size_t n)
    {
        size_t sent = 0;
        while (sent < n)
        {
            int r = ::send(sock_, p + sent, (int)(n - sent), 0);
            if (r <= 0) return false;
            sent += r;
        }
        return true;
    }
    bool recvAll(void* buf, size_t n)
    {
        uint8_t* p = (uint8_t*)buf;
        size_t got = 0;
        while (got < n)
        {
            int r = ::recv(sock_, (char*)p + got, (int)(n - got), 0);
            if (r <= 0) return false;
            got += r;
        }
        return true;
    }
    // little-endian 파서 (Jetson ARM64 / Windows x64 모두 LE)
    static uint32_t readU32(const uint8_t* b, size_t& off)
    {
        uint32_t v; std::memcpy(&v, b + off, 4); off += 4; return v;
    }
    static float readF32(const uint8_t* b, size_t& off)
    {
        float v; std::memcpy(&v, b + off, 4); off += 4; return v;
    }
};