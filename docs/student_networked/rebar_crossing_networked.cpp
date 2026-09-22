// ============================================================================
//  rebar_crossing_networked.cpp   ─ 단일 파일 버전 (ZED 로컬 → 로봇 WLAN 수신)
//
//  · 학생 원본 알고리즘(평면추출→직선→YOLO→교차점) 그대로.
//  · 카메라 입력만: sl::Camera(ZED) → FrameClient(TCP, 로봇 Orbbec 프레임 수신).
//  · 별도 헤더 없음. 이 파일 하나만 빌드하면 됨.
//
//  [빌드] OpenCV + LibTorch 링크. ws2_32 는 아래 #pragma 로 자동(MSVC).
//         ZED SDK 관련 include/lib/링크는 전부 제거할 것.
//  [실행] 로봇에서 frame_stream_server.py 가 떠 있어야 함(robot-control 자동 기동).
//         아래 ROBOT_IP 를 로봇 Tailscale IP 로 맞출 것.
// ============================================================================

// ── winsock 은 반드시 windows.h/torch 보다 먼저 + min/max 매크로 차단 ──
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <fstream>
#include <set>
#include <map>
#include <tuple>
#include <algorithm>
#include <string>
#include <limits>
#include <cstdint>
#include <cstring>
using namespace std;
using namespace cv;

// ============================================================================
//  FrameClient ─ 로봇(Jetson) 프레임 서버에서 컬러+뎁스 수신 (TCP)
//    grab() : image_bgr(CV_8UC3) / depth_f32(CV_32FC1, mm, 무효=NaN) / K(fx..cy)
// ============================================================================
class FrameClient
{
public:
    struct Intr { float fx = 0, fy = 0, cx = 0, cy = 0; };

    FrameClient() { ensureWsa(); }
    ~FrameClient() { disconnect(); }

    bool connect(const string& ip, int port)
    {
        disconnect();
        sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock_ == INVALID_SOCKET) return false;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons((u_short)port);
        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) { disconnect(); return false; }
        if (::connect(sock_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) { disconnect(); return false; }
        int one = 1;
        setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        return true;
    }

    void disconnect()
    {
        if (sock_ != INVALID_SOCKET) { closesocket(sock_); sock_ = INVALID_SOCKET; }
    }
    bool connected() const { return sock_ != INVALID_SOCKET; }

    bool grab(Mat& image_bgr, Mat& depth_f32, Intr& K)
    {
        if (sock_ == INVALID_SOCKET) return false;
        const char req = 'G';
        if (!sendAll(&req, 1)) return false;

        uint8_t hdr[HEADER_SIZE];
        if (!recvAll(hdr, HEADER_SIZE)) return false;

        size_t off = 0;
        if (memcmp(hdr, "RBF1", 4) != 0) return false;
        off = 4;
        readU32(hdr, off);                              // frame_id
        K.fx = readF32(hdr, off); K.fy = readF32(hdr, off);
        K.cx = readF32(hdr, off); K.cy = readF32(hdr, off);
        readU32(hdr, off); readU32(hdr, off);           // w, h
        float depth_scale = readF32(hdr, off);
        uint32_t color_len = readU32(hdr, off);
        uint32_t depth_len = readU32(hdr, off);
        if (color_len == 0 || depth_len == 0) return false;

        vector<uint8_t> cbuf(color_len), dbuf(depth_len);
        if (!recvAll(cbuf.data(), color_len)) return false;
        if (!recvAll(dbuf.data(), depth_len)) return false;

        image_bgr = imdecode(cbuf, IMREAD_COLOR);
        if (image_bgr.empty()) return false;

        Mat depth16 = imdecode(dbuf, IMREAD_UNCHANGED);
        if (depth16.empty() || depth16.type() != CV_16UC1) return false;
        Mat invalid = (depth16 == 0);
        depth16.convertTo(depth_f32, CV_32FC1, depth_scale);            // mm
        depth_f32.setTo(numeric_limits<float>::quiet_NaN(), invalid);   // 0 → NaN (ZED와 동일)
        return true;
    }

private:
    static const int HEADER_SIZE = 44;
    SOCKET sock_ = INVALID_SOCKET;

    static void ensureWsa()
    {
        static bool inited = false;
        if (!inited) { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); inited = true; }
    }
    bool sendAll(const char* p, size_t n)
    {
        size_t sent = 0;
        while (sent < n) { int r = ::send(sock_, p + sent, (int)(n - sent), 0); if (r <= 0) return false; sent += r; }
        return true;
    }
    bool recvAll(void* buf, size_t n)
    {
        uint8_t* p = (uint8_t*)buf; size_t got = 0;
        while (got < n) { int r = ::recv(sock_, (char*)p + got, (int)(n - got), 0); if (r <= 0) return false; got += r; }
        return true;
    }
    static uint32_t readU32(const uint8_t* b, size_t& off) { uint32_t v; memcpy(&v, b + off, 4); off += 4; return v; }
    static float    readF32(const uint8_t* b, size_t& off) { float v;    memcpy(&v, b + off, 4); off += 4; return v; }
};

// ============================================================================
//  ↓↓↓ 여기서부터 학생 원본 알고리즘 (수정 없음) ↓↓↓
// ============================================================================

// 반복 개수 +목표 상하 +관심 영역
const int ITERATIONS1 = 20000;      // 1 3차원 수
const int ITERATIONS2 = 500000;      //   1차원 수
const int LAYER_RANK = 1;           //   추출 평면(0상단근(,단층), 1하단근)
const int ITERATIONS3 = 2500;       // 2 2차원 수
const int STEP = 100;               //   스텝 수
const float OFFSET = 30.f;          //   추출 영역 및 임계 거리
const float DEPTH_MIN = 100.f;
const float DEPTH_MAX = 1000.f;
Mat visualizeDepthMap(const Mat& depth_mat, float depth_min, float depth_max, int colormap = COLORMAP_JET)
{
    Mat depth_clamped = depth_mat.clone();
    for (int y = 0; y < depth_clamped.rows; y++)
    {
        for (int x = 0; x < depth_clamped.cols; x++)
        {
            float& val = depth_clamped.at<float>(y, x);
            if (!isfinite(val) || val < depth_min || val > depth_max) val = 0;
            else val = (val - depth_min) / (depth_max - depth_min) * 255.0f; // 정규화!
        }
    }
    Mat depth_normalized;
    depth_clamped.convertTo(depth_normalized, CV_8U);
    Mat depth_colored;
    applyColorMap(depth_normalized, depth_colored, colormap);
    return depth_colored;
}

struct PlanePoint           // 3차원 점과 평면 + 군집
{
    Vec3f pt0, pt1, pt2;    // 3점 평면(★p0으로만 1차원시 이용)
    float theta;            // [0, π] 양수 통일
    float phi;              // [0, π/2] 반구 통일

    float d;                // 평면까지 거리(★1차원에도 사용)

    Vec3f center;           // 군집 중심점 (theta,phi,d 평균)
    int cluster_id;         // 군집 인덱스
};
struct PlaneResult          // 바닥/목표 평면 + PCD
{
    vector<Vec3f> pts;      // PCD
    Vec3f normal;           // 법선벡터
    Vec3f mean;             // 평균점
};
PlaneResult depthToPointCloud(const Mat& depth_f32, float fx, float fy, float cx, float cy, float depth_min, float depth_max)
{
    PlaneResult cloud;
    cloud.pts.reserve(depth_f32.rows * depth_f32.cols);
    for (int r = 0; r < depth_f32.rows; ++r)
    {
        const float* row_ptr = depth_f32.ptr<float>(r); // 행주소를 기반으로, 다음에 나오는 열진행(빠름!)
        for (int c = 0; c < depth_f32.cols; ++c)
        {
            float z = row_ptr[c];
            if (!isfinite(z) || z < depth_min || z > depth_max) continue;
            // ★★데이터 정리
            cloud.pts.push_back(Vec3f((c - cx) * z / fx, (r - cy) * z / fy, z));
        }
    }
    return cloud;
}
vector<PlanePoint> sampleHoughPlane(const PlaneResult& cloud, int iterations, unsigned int seed = 42)
{
    vector<PlanePoint> hough_pts;
    hough_pts.reserve(iterations);
    mt19937 rng(seed);
    uniform_int_distribution<int> dist(0, (int)cloud.pts.size() - 1);
    for (int iter = 0; iter < iterations; ++iter)
    {
        // 랜덤 3점 선택 (중복 방지)
        int i0, i1, i2;
        i0 = dist(rng);
        do { i1 = dist(rng); } while (i1 == i0);
        do { i2 = dist(rng); } while (i2 == i0 || i2 == i1);
        Vec3f p0 = cloud.pts[i0];
        Vec3f p1 = cloud.pts[i1];
        Vec3f p2 = cloud.pts[i2];
        // 법선벡터 = 외적
        Vec3f v1 = p1 - p0;
        Vec3f v2 = p2 - p0;
        Vec3f n = v1.cross(v2);
        float len = norm(n);
        if (len < 1e-6f) continue;
        n /= len;
        if (n[2] < 0) n = -n; // ★반구 통일 (z 양수 방향)
        float theta = atan2f(n[1], n[0]);
        float phi = acosf(clamp(n[2], -1.f, 1.f));
        if (theta < 0) theta += (float)CV_PI; // ★theta 양수 통일 [0, π]
        float d = -(n.dot(p0)); // ★평면의 방정식 꼴 상수항
        // ★★데이터 정리
        PlanePoint pp;
        pp.pt0 = p0;
        pp.pt1 = p1;
        pp.pt2 = p2;
        pp.theta = theta;
        pp.phi = phi;
        pp.d = d;
        hough_pts.push_back(pp);
    }
    return hough_pts;
}
vector<PlanePoint> clusterHoughPlane(vector<PlanePoint>& hough_pts, int k_min = 1, int k_max = 10)
{
    int N = (int)hough_pts.size();
    float theta_min = FLT_MAX, theta_max = -FLT_MAX;
    float phi_min = FLT_MAX, phi_max = -FLT_MAX;
    float d_min = FLT_MAX, d_max = -FLT_MAX;
    for (int i = 0; i < N; ++i)
    {
        theta_min = min(theta_min, hough_pts[i].theta);
        theta_max = max(theta_max, hough_pts[i].theta);
        phi_min = min(phi_min, hough_pts[i].phi);
        phi_max = max(phi_max, hough_pts[i].phi);
        d_min = min(d_min, hough_pts[i].d);
        d_max = max(d_max, hough_pts[i].d);
    }
    float theta_range = theta_max - theta_min;
    float phi_range = phi_max - phi_min;
    float d_range = d_max - d_min;
    Mat data(N, 3, CV_32F);
    for (int i = 0; i < N; ++i)
    {
        data.at<float>(i, 0) = (hough_pts[i].theta - theta_min) / theta_range * 100.f;
        data.at<float>(i, 1) = (hough_pts[i].phi - phi_min) / phi_range * 100.f;
        data.at<float>(i, 2) = (hough_pts[i].d - d_min) / d_range * 100.f;
    }
    // (1)Elbow method→ 곡률 최대
    vector<float> wcss;
    vector<Mat> all_labels;
    vector<Mat> all_centers;
    for (int k = k_min; k <= k_max; ++k)
    {
        Mat labels, centers;
        float w = kmeans(data, k, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1), 3, KMEANS_PP_CENTERS, centers);
        wcss.push_back(w);
        all_labels.push_back(labels.clone());
        all_centers.push_back(centers.clone());
        printf("k=%d, 3평면 wcss=%.2f\n", k, w);
    }
    // (2)곡률 최대→ best_k
    // ★로그 변환 후 곡률
    int best_k = k_min;
    float max_kappa = -1.f;
    for (int i = 1; i < (int)wcss.size() - 1; ++i)
    {
        float lw_prev = logf(wcss[i - 1]);
        float lw_curr = logf(wcss[i]);
        float lw_next = logf(wcss[i + 1]);
        float dy = (lw_next - lw_prev) / 2.f;
        float d2y = lw_prev - 2.f * lw_curr + lw_next;
        float kappa = fabsf(d2y) / powf(1.f + dy * dy, 1.5f);
        if (kappa > max_kappa) { max_kappa = kappa; best_k = k_min + i; }
    }
    // (3)best_k→ 최종 군집화
    //best_k = 3; //★★★ 야매로 바꾸기!
    Mat& labels = all_labels[best_k - k_min];
    Mat& centers = all_centers[best_k - k_min];
    for (int i = 0; i < N; ++i) hough_pts[i].cluster_id = labels.at<int>(i);
    // ★군집별 카운트 -> 카운트 내림차순 => 기존 label → 내림차순 rank 매핑
    vector<pair<int, int>> counts(best_k);
    for (int k = 0; k < best_k; ++k) counts[k] = { 0, k };
    for (int i = 0; i < N; ++i) counts[hough_pts[i].cluster_id].first++;
    sort(counts.begin(), counts.end(), [](const pair<int, int>& a, const pair<int, int>& b) { return a.first > b.first; });
    vector<int> label_to_rank(best_k);
    for (int r = 0; r < best_k; ++r) label_to_rank[counts[r].second] = r;
    // ★★데이터 정리
    for (int i = 0; i < N; ++i)
    {
        int orig = labels.at<int>(i);
        hough_pts[i].cluster_id = label_to_rank[orig];
        hough_pts[i].center = Vec3f(centers.at<float>(orig, 0) / 100.f * theta_range + theta_min,
            centers.at<float>(orig, 1) / 100.f * phi_range + phi_min, centers.at<float>(orig, 2) / 100.f * d_range + d_min);
    }
    printf("best_k=%d\n", best_k);
    return hough_pts;
}
PlaneResult extractPlaneByRank(const vector<PlanePoint>& hough_pts)
{
    // 1) 바닥 군집의 center 추출!
    Vec3f center;
    for (const PlanePoint& pp : hough_pts)
    {
        if (pp.cluster_id != 0) continue;
        center = pp.center;
        break;
    }
    // 2) 법선 벡터 정의
    Vec3f normal(cosf(center[0]) * sinf(center[1]), sinf(center[0]) * sinf(center[1]), cosf(center[1]));
    //if (normal[2] < 0) normal = -normal; // ★0-90도로 정제했기 때문에 불필요!
    // 3) PCD인 pt0,pt1,pt2 중복없이 추출
    set<tuple<float, float, float>> unique_set;
    vector<Vec3f> pts;
    for (const PlanePoint& pp : hough_pts)
    {
        if (pp.cluster_id != 0) continue;
        for (const Vec3f& p : { pp.pt0, pp.pt1, pp.pt2 })
        {
            auto key = make_tuple(p[0], p[1], p[2]);
            if (unique_set.insert(key).second) pts.push_back(p);
        }
    }
    // 4) nd = x로 한 점 계산
    float d = center[2];
    Vec3f mean = normal * (-d);
    // ★★데이터 정리
    PlaneResult result;
    result.pts = pts;
    result.normal = normal;
    result.mean = mean;
    return result;
}
vector<PlanePoint> computeDistances(const PlaneResult& cloud, const PlaneResult& plane, int iterations, unsigned int seed = 42)
{
    vector<int> indices(cloud.pts.size());
    for (int i = 0; i < (int)cloud.pts.size(); ++i) indices[i] = i;
    mt19937 rng(seed);
    shuffle(indices.begin(), indices.end(), rng);
    vector<PlanePoint> layer_pts;
    layer_pts.reserve(iterations);
    for (int iter = 0; iter < iterations; ++iter)
    {
        const Vec3f& p = cloud.pts[indices[iter]];
        float d = plane.normal.dot(p) - plane.normal.dot(plane.mean);
        // ★★데이터 정리
        PlanePoint lp;
        lp.pt0 = Vec3f(p[0], p[1], p[2]); // ★★★ pt0를 써서 접근!!
        lp.d = d;
        layer_pts.push_back(lp);
    }
    return layer_pts;
}
vector<PlanePoint> clusterLayers(vector<PlanePoint>& layer_pts, int k_min = 1, int k_max = 10)
{
    int N = (int)layer_pts.size();
    Mat data(N, 1, CV_32F);
    for (int i = 0; i < N; ++i) data.at<float>(i, 0) = layer_pts[i].d;
    // 1) Elbow method → 곡률 최대
    vector<float> wcss;
    vector<Mat> all_labels;
    vector<Mat> all_centers;
    for (int k = k_min; k <= k_max; ++k)
    {
        Mat labels, centers;
        float w = kmeans(data, k, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1), 3, KMEANS_PP_CENTERS, centers);
        wcss.push_back(w);
        all_labels.push_back(labels.clone());
        all_centers.push_back(centers.clone());
        printf("k=%d, 1평면 wcss=%.2f\n", k, w);
    }
    // 2) 곡률 최대→ best_k
    // ★로그 변환 후 곡률
    int best_k = k_min;
    float max_kappa = -1.f;
    for (int i = 1; i < (int)wcss.size() - 1; ++i)
    {
        float lw_prev = logf(wcss[i - 1]);
        float lw_curr = logf(wcss[i]);
        float lw_next = logf(wcss[i + 1]);
        float dy = (lw_next - lw_prev) / 2.f;
        float d2y = lw_prev - 2.f * lw_curr + lw_next;
        float kappa = fabsf(d2y) / powf(1.f + dy * dy, 1.5f);
        if (kappa > max_kappa) { max_kappa = kappa; best_k = k_min + i; }
    }
    // 3) best_k → 최종 군집화
    //best_k = 3; //★★★ 야매로 바꾸기!
    Mat& labels = all_labels[best_k - k_min];
    Mat& centers = all_centers[best_k - k_min];
    // ★d 큰 순서로 rank 정렬 (상단근=0, 하단근=1, 바닥=2)
    vector<pair<float, int>> center_vals(best_k);
    for (int c = 0; c < best_k; ++c) center_vals[c] = { centers.at<float>(c, 0), c };
    sort(center_vals.begin(), center_vals.end(), [](const pair<float, int>& a, const pair<float, int>& b) { return a.first < b.first; });
    vector<int> label_to_rank(best_k);
    for (int r = 0; r < best_k; ++r) label_to_rank[center_vals[r].second] = r;
    // ★★데이터 정리 cluster_id 재부여 + d에 군집 중심 거리 저장
    for (int i = 0; i < N; ++i)
    {
        int original_label = labels.at<int>(i);                         // 원본 label 먼저 저장
        layer_pts[i].cluster_id = label_to_rank[original_label];        // rank 재부여
    }
    printf("best_k=%d\n", best_k);
    return layer_pts;
}
PlaneResult extractLayerByRank(const vector<PlanePoint>& layer_pts, int rank)
{
    // 1) rank에 군집 PCD 추출
    vector<Vec3f> pts;
    for (const PlanePoint& lp : layer_pts)
    {
        if (lp.cluster_id != rank) continue;
        pts.push_back(lp.pt0);
    }
    // 2) PCA로 평면 추출
    Mat pts_mat((int)pts.size(), 3, CV_32F);
    for (int i = 0; i < (int)pts.size(); ++i)
    {
        pts_mat.at<float>(i, 0) = pts[i][0];
        pts_mat.at<float>(i, 1) = pts[i][1];
        pts_mat.at<float>(i, 2) = pts[i][2];
    }
    PCA pca(pts_mat, Mat(), PCA::DATA_AS_ROW);
    // 가장 작은 분산 방향 = 법선벡터와 평균점으로 정의
    Vec3f normal(pca.eigenvectors.at<float>(2, 0), pca.eigenvectors.at<float>(2, 1), pca.eigenvectors.at<float>(2, 2));
    if (normal[2] < 0) normal = -normal;
    Vec3f mean(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1), pca.mean.at<float>(0, 2));
    // ★★데이터 정리
    PlaneResult result;
    result.pts = pts;
    result.normal = normal;
    result.mean = mean;
    return result;
}
void savePlaneResultCsv(const PlaneResult& plane, const Mat& image, float fx, float fy, float cx, float cy, const string& path)
{
    ofstream f(path);
    f << "x,y,z,nx,ny,nz,mx,my,mz,fx,fy,cx,cy,img_w,img_h\n";
    for (const Vec3f& p : plane.pts)
        f << p[0] << "," << p[1] << "," << p[2] << "," << plane.normal[0] << "," << plane.normal[1] << "," << plane.normal[2] << ","
        << plane.mean[0] << "," << plane.mean[1] << "," << plane.mean[2]
        << "," << fx << "," << fy << "," << cx << "," << cy << "," << image.cols << "," << image.rows << "\n";
    f.close();
}
void saveHoughPlaneCsv(const vector<PlanePoint>& hough_pts, const string& path)
{
    ofstream f(path);
    f << "theta,phi,d,cluster_id,center_theta,center_phi,center_d\n"; // 3차원 데이터 명시 위해 필요!
    for (const PlanePoint& pp : hough_pts)
        f << pp.theta << "," << pp.phi << "," << pp.d << "," << pp.cluster_id << "," << pp.center[0] << "," << pp.center[1] << "," << pp.center[2] << "\n";
    f.close();
}
void saveLayerCSV(const vector<PlanePoint>& layer_pts, const string& path)
{
    ofstream f(path);
    f << "d,cluster_id\n";
    for (const auto& lp : layer_pts)
        f << lp.d << "," << lp.cluster_id << "\n";
    f.close();
}

struct LineCandidate
{
    float theta;        // [0, π] 양수
    float rho;          // 원점-직선 거리

    float score;        // 헤시안 크기

    float d_theta;      // 윈도우 가로 크기 (Δtheta)(★헤시안에서 윈도우 중심점)
    float d_rho;        // 윈도우 세로 크기 (Δrho)(★헤시안에서 윈도우 중심점)

    Rect2f box;         // 윈도우 박스 (t_min, r_min, width, height)
};
vector<Point2f> projectToImage(const vector<Vec3f>& pts, float fx, float fy, float cx, float cy)
{
    vector<Point2f> pixels;
    pixels.reserve(pts.size());
    for (const Vec3f& p : pts)
    {
        if (p[2] < 1e-6f) continue;  // 분모 z=0 방지
        float c = p[0] * fx / p[2] + cx;
        float r = p[1] * fy / p[2] + cy;
        // ★★데이터 정리
        pixels.push_back(Point2f(c, r));
    }
    return pixels;
}
vector<LineCandidate> sampleHoughLine(const vector<Point2f>& pixels, int iterations, unsigned int seed = 42)
{
    mt19937 rng(seed);
    uniform_int_distribution<int> dist(0, (int)pixels.size() - 1);
    vector<LineCandidate> candidates;
    candidates.reserve(iterations);
    for (int iter = 0; iter < iterations; ++iter)
    {
        // 랜덤 2점 선택 (중복 방지)
        int i0, i1;
        i0 = dist(rng);
        do { i1 = dist(rng); } while (i1 == i0);
        Point2f p0 = pixels[i0];
        Point2f p1 = pixels[i1];
        // 방향벡터 → 법선벡터
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-6f) continue;
        // 반원 통일 (ny, theta 양수 방향) (34->12사분면)
        float nx = -dy / len;
        float ny = dx / len;
        if (ny < 0) { nx = -nx; ny = -ny; }
        float theta = atan2f(ny, nx);
        if (theta < 0) theta += (float)CV_PI;
        float rho = -(nx * p0.x + ny * p0.y);
        LineCandidate lc;
        lc.theta = theta;
        lc.rho = rho;
        candidates.push_back(lc);
    }
    return candidates;
}
vector<LineCandidate> estimateWindowSize(const vector<LineCandidate>& points, unsigned int seed = 42)
{
    int N = (int)points.size();
    float theta_min = FLT_MAX, theta_max = -FLT_MAX;
    float rho_min = FLT_MAX, rho_max = -FLT_MAX;
    for (int i = 0; i < N; ++i)
    {
        theta_min = min(theta_min, points[i].theta);
        theta_max = max(theta_max, points[i].theta);
        rho_min = min(rho_min, points[i].rho);
        rho_max = max(rho_max, points[i].rho);
    }
    float theta_range = theta_max - theta_min;
    float rho_range = rho_max - rho_min;
    vector<Point2f> pts_norm(N);
    for (int i = 0; i < N; ++i) pts_norm[i] = Point2f((points[i].theta - theta_min) / theta_range * 100.f, (points[i].rho - rho_min) / rho_range * 100.f);
    // 모든 점에서 전체 거리 계산 (pair: 거리 + 인덱스)
    vector<vector<pair<float, int>>> all_dists(N);
    for (int i = 0; i < N; ++i) all_dists[i].reserve(N - 1);
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
        {
            float dx = pts_norm[i].x - pts_norm[j].x;
            float dy = pts_norm[i].y - pts_norm[j].y;
            float d = sqrtf(dx * dx + dy * dy);
            all_dists[i].push_back({ d, j });
            all_dists[j].push_back({ d, i });
        }
    for (int i = 0; i < N; ++i) sort(all_dists[i].begin(), all_dists[i].end());
    // 1) best_ref: 누적합 가장 작은 점 인덱스
    float min_cum = FLT_MAX;
    int best_ref = -1;
    for (int i = 0; i < N; ++i)
    {
        float cum = 0.f;
        for (int ki = 0; ki < (int)sqrtf(N); ++ki) cum += all_dists[i][ki].first;
        if (cum < min_cum) { min_cum = cum; best_ref = i; }
    }
    // 2) best_K: best_ref의 fk 곡선에서 엘보우
    int best_K = 1;
    float max_kappa = -FLT_MAX;
    for (int idx = 1; idx <= (int)sqrtf(N) * 20; ++idx)
    {
        float dp = all_dists[best_ref][idx - 1].first;
        float dc = all_dists[best_ref][idx].first;
        float dn = all_dists[best_ref][idx + 1].first;
        float dy = (dn - dp) / 2.f;
        float d2y = dp - 2.f * dc + dn;
        float kappa = fabsf(d2y) / powf(1.f + dy * dy, 1.5f);
        if (kappa > max_kappa) { max_kappa = kappa; best_K = idx + 1; }
        printf("k=%d, d_k=%.4f, kappa=%.6f\n", idx + 1, dc, kappa);
    }
    printf("best_ref=%d, best_K=%d\n", best_ref, best_K);
    // best_ref의 best_K개 이웃 → 윈도우 크기 (ref_dists 중복 계산 제거)
    vector<LineCandidate> result;
    LineCandidate lc_ref;  // best_ref 먼저 추가 (중심점)
    lc_ref.theta = points[best_ref].theta;
    lc_ref.rho = points[best_ref].rho;
    result.push_back(lc_ref);  // index 0

    float tw_min = FLT_MAX, tw_max = -FLT_MAX;
    float rw_min = FLT_MAX, rw_max = -FLT_MAX;
    for (int k = 0; k < best_K; ++k)
    {
        int idx = all_dists[best_ref][k].second;  // ← 인덱스 바로 사용
        LineCandidate lc;
        lc.theta = points[idx].theta;
        lc.rho = points[idx].rho;
        tw_min = min(tw_min, lc.theta);
        tw_max = max(tw_max, lc.theta);
        rw_min = min(rw_min, lc.rho);
        rw_max = max(rw_max, lc.rho);
        result.push_back(lc);
    }
    result[0].d_theta = tw_max - tw_min;
    result[0].d_rho = rw_max - rw_min;
    printf("d_theta=%.4f, d_rho=%.2f\n", result[0].d_theta, result[0].d_rho);
    return result;
}
vector<LineCandidate> computeHessianOnKernel(const vector<LineCandidate>& houghPoints, float d_theta, float d_rho, float rho_min, float rho_max)
{
    // ── 커널 파라미터 ──
    float theta_pad = d_theta / 2.f;
    float rho_pad = d_rho / 2.f;
    int n_theta = STEP;
    int n_rho = STEP;
    float theta_stride = (float)CV_PI / (float)n_theta;
    float rho_stride = (rho_max - rho_min) / (float)n_rho;
    // ── 경계 패딩 ──
    vector<LineCandidate> paddedPoints = houghPoints;
    for (const auto& hp : houghPoints)
    {
        if (hp.theta < theta_pad)
        {
            LineCandidate padded = hp;
            padded.theta = hp.theta + (float)CV_PI;
            padded.rho = -hp.rho;
            paddedPoints.push_back(padded);
        }
        else if (hp.theta >= ((float)CV_PI - theta_pad))
        {
            LineCandidate padded = hp;
            padded.theta = hp.theta - (float)CV_PI;
            padded.rho = -hp.rho;
            paddedPoints.push_back(padded);
        }
    }
    // ── 슬라이딩 윈도우 ──
    vector<LineCandidate> results;
    for (int ri = 0; ri <= n_rho; ++ri)
    {
        float center_rho = rho_min + ri * rho_stride;
        for (int si = 0; si <= n_theta; ++si)
        {
            float center_theta = si * theta_stride;
            float t_min = center_theta - theta_pad;
            float t_max = center_theta + theta_pad;
            float r_min = center_rho - rho_pad;
            float r_max = center_rho + rho_pad;
            // ── 커널 내 점들 수집 ──
            vector<pair<Point2f, Point2f>> inKernel;  // (정규화, 원본)
            for (const auto& hp : paddedPoints)
            {
                if (hp.theta > t_min && hp.theta < t_max && hp.rho > r_min && hp.rho < r_max)
                {
                    float thn = ((hp.theta - t_min) / (t_max - t_min) - 0.5f) * 0.5f * 20.f;
                    float rhn = ((hp.rho - r_min) / (r_max - r_min) - 0.5f) * 0.5f * 20.f;
                    inKernel.push_back({ Point2f(thn, rhn), Point2f(hp.theta, hp.rho) });
                }
            }
            if ((int)inKernel.size() < 2) continue;
            // ── 가중 평균 ──
            Point2f mun(0, 0), mu(0, 0);
            for (const auto& wp : inKernel)
            {
                mun += wp.first;
                mu += wp.second;
            }
            mun *= (1.f / (float)inKernel.size());
            mu *= (1.f / (float)inKernel.size());
            // ── 헤시안 행렬 계산 ──
            float h11 = 0, h12 = 0, h22 = 0, sum_c = 0;
            for (const auto& wp : inKernel)
            {
                float dx = wp.first.x - mun.x;
                float dy = wp.first.y - mun.y;
                float c = expf(-(dx * dx + dy * dy));
                h11 += c * dx * dx;
                h12 += c * dx * dy;
                h22 += c * dy * dy;
                sum_c += c;
            }
            h11 = 4.f * h11 - 2.f * sum_c;
            h22 = 4.f * h22 - 2.f * sum_c;
            h12 = 4.f * h12;
            // ── 고유값 계산 ──
            float trace = h11 + h22;
            float det = h11 * h22 - h12 * h12;
            float disc = trace * trace - 4.f * det;
            if (disc < 0) continue;
            float lambda = (trace - sqrtf(disc)) * 0.5f;
            // ── 결과 저장 ──
            LineCandidate res;
            res.score = (lambda < 0.f) ? -lambda : 0.f; // ★★★
            res.theta = mu.x;
            res.rho = mu.y;
            res.d_theta = center_theta;   // ★중심점 재활용
            res.d_rho = center_rho;       // ★중심점 재활용
            res.box = Rect2f(t_min, r_min, d_theta, d_rho);
            results.push_back(res);
        }
    }
    return results;
}
vector<LineCandidate> selectByElbowKMeans(const vector<LineCandidate>& points, int num_clusters = 2, int num_top_select = 1)
{
    int N = (int)points.size();
    if (N < num_clusters) return points;
    Mat data(N, 1, CV_32F);
    for (int i = 0; i < N; ++i) data.at<float>(i, 0) = points[i].score;
    Mat labels, centers;
    kmeans(data, num_clusters, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1), 3, KMEANS_PP_CENTERS, centers);
    // score 내림차순 → 상위 num_top_select 클러스터 선택
    vector<pair<float, int>> center_vals(num_clusters);
    for (int c = 0; c < num_clusters; ++c) center_vals[c] = { centers.at<float>(c, 0), c };
    std::sort(center_vals.begin(), center_vals.end(), [](const pair<float, int>& a, const pair<float, int>& b) { return a.first > b.first; });
    int actual_select = min(num_top_select, num_clusters);
    vector<int> selected_clusters;
    for (int i = 0; i < actual_select; ++i) selected_clusters.push_back(center_vals[i].second);
    vector<LineCandidate> selected;
    for (int i = 0; i < N; ++i)
    {
        int cid = labels.at<int>(i);
        if (find(selected_clusters.begin(), selected_clusters.end(), cid) != selected_clusters.end())
            selected.push_back(points[i]);
    }
    return selected;
}
vector<LineCandidate> clusterAndAverageLines(const vector<LineCandidate>& points, float d_theta, float d_rho, float epsilon = 0.5f)
{
    float eps_theta = d_theta;
    float eps_rho = d_rho;
    int N = (int)points.size();
    vector<Point2f> pts_scaled(N);
    for (int i = 0; i < N; ++i) pts_scaled[i] = Point2f(points[i].theta / eps_theta, points[i].rho / eps_rho);
    // ── 1차 DBSCAN ──
    vector<int> labels(N, -1);
    int cluster_id = 0;
    for (int i = 0; i < N; ++i)
    {
        if (labels[i] != -1) continue;
        vector<int> neighbors;
        for (int j = 0; j < N; ++j)
        {
            float dx = pts_scaled[i].x - pts_scaled[j].x;
            float dy = pts_scaled[i].y - pts_scaled[j].y;
            if (sqrtf(dx * dx + dy * dy) < epsilon) neighbors.push_back(j);
        }
        //if ((int)neighbors.size() < 5) continue; //★★★
        cluster_id++;
        labels[i] = cluster_id;
        size_t k = 0;
        while (k < neighbors.size())
        {
            int nb = neighbors[k];
            if (labels[nb] == -1)
            {
                labels[nb] = cluster_id;
                for (int j = 0; j < N; ++j)
                {
                    float dx = pts_scaled[nb].x - pts_scaled[j].x;
                    float dy = pts_scaled[nb].y - pts_scaled[j].y;
                    if (sqrtf(dx * dx + dy * dy) < epsilon && find(neighbors.begin(), neighbors.end(), j) == neighbors.end()) neighbors.push_back(j);
                }
            }
            k++;
        }
    }
    // ── 군집 평균 계산  ──
    if (cluster_id == 0) return {};
    vector<Point2f> cluster_means(cluster_id + 1, Point2f(0, 0));
    vector<int>     cluster_cnt(cluster_id + 1, 0);
    for (int i = 0; i < N; ++i)
    {
        int c = labels[i];
        cluster_means[c].x += points[i].theta;
        cluster_means[c].y += points[i].rho;
        cluster_cnt[c]++;
    }
    for (int c = 1; c <= cluster_id; ++c)
        if (cluster_cnt[c] > 0)
        {
            //if (c == -1) continue; //★★★
            cluster_means[c].x /= cluster_cnt[c];
            cluster_means[c].y /= cluster_cnt[c];
        }
    // ── 경계 탐지 및 0도 근방 변환 ──
    const float boundary = 10.f * (float)CV_PI / 180.f;
    vector<int> boundary_ids;
    for (int c = 1; c <= cluster_id; ++c)
        if (cluster_means[c].x < boundary || cluster_means[c].x >(float)CV_PI - boundary) boundary_ids.push_back(c);
    for (int bid : boundary_ids)
        if (cluster_means[bid].x > (float)CV_PI - boundary)
        {
            cluster_means[bid].x -= (float)CV_PI;
            cluster_means[bid].y = -cluster_means[bid].y;
        }
    // ── 2차 DBSCAN (경계 군집 평균값끼리) ──
    vector<int> merge_map(cluster_id + 1);
    for (int i = 0; i <= cluster_id; ++i) merge_map[i] = i;
    if (boundary_ids.size() > 1)
    {
        // 변환된 평균값 정규화
        vector<Point2f> bscaled;
        for (int bid : boundary_ids) bscaled.push_back(Point2f(cluster_means[bid].x / eps_theta, cluster_means[bid].y / eps_rho));
        int nb = (int)bscaled.size();
        vector<int> blabels(nb, -1);
        int bcid = 0;
        for (int i = 0; i < nb; ++i)
        {
            if (blabels[i] != -1) continue;
            vector<int> nbrs;
            for (int j = 0; j < nb; ++j)
            {
                float dx = bscaled[i].x - bscaled[j].x;
                float dy = bscaled[i].y - bscaled[j].y;
                if (sqrtf(dx * dx + dy * dy) < epsilon) nbrs.push_back(j);
            }
            if ((int)nbrs.size() <= 1) { blabels[i] = 0; continue; }
            bcid++;
            for (int idx : nbrs) blabels[idx] = bcid;
        }
        for (int bl = 1; bl <= bcid; ++bl)
        {
            vector<int> group;
            for (int i = 0; i < nb; ++i)
                if (blabels[i] == bl) group.push_back(boundary_ids[i]);
            if ((int)group.size() > 1)
                for (int idx : group) merge_map[idx] = group[0];
        }
    }
    // ── 라벨 병합 + 재매핑 ──
    map<int, int> remap;
    int new_cid = 0;
    for (int c = 1; c <= cluster_id; ++c)
    {
        int root = merge_map[c];
        if (remap.find(root) == remap.end()) remap[root] = ++new_cid;
    }
    cluster_id = new_cid;
    // ── 최종 평균점 계산 ──
    vector<Point2f> result_means(cluster_id + 1, Point2f(0, 0));
    vector<int>     result_cnt(cluster_id + 1, 0);
    for (int c = 1; c <= (int)merge_map.size() - 1; ++c)
    {
        int root = merge_map[c];
        int new_c = remap[root];
        result_means[new_c].x += cluster_means[c].x;
        result_means[new_c].y += cluster_means[c].y;
        result_cnt[new_c]++;
    }
    vector<LineCandidate> avg_lines;
    for (int c = 1; c <= cluster_id; ++c)
    {
        if (result_cnt[c] == 0) continue;
        LineCandidate lc;
        lc.theta = result_means[c].x / result_cnt[c];
        lc.rho = result_means[c].y / result_cnt[c];
        // wrap-around 보정
        if (lc.theta < 0.f) { lc.theta += (float)CV_PI; lc.rho = -lc.rho; }
        avg_lines.push_back(lc);
    }
    return avg_lines;
}
void savePixelCsv(const vector<Point2f>& pixels, const Mat& image_bgr, const string& path)
{
    ofstream f(path);
    f << "u,v\n";
    for (const Point2f& px : pixels)
    {
        int u = cvRound(px.x);
        int v = cvRound(px.y);
        if (u < 0 || u >= image_bgr.cols) continue;
        if (v < 0 || v >= image_bgr.rows) continue;
        f << u << "," << v << "\n";
    }
    f.close();
}
void saveLineCsv(const vector<LineCandidate>& points, const string& path)
{
    ofstream f(path);
    f << "theta,rho,score,d_theta,d_rho,box_x,box_y,box_w,box_h\n";
    for (const LineCandidate& lc : points)
        f << lc.theta << "," << lc.rho << "," << lc.score << ","
        << lc.d_theta << "," << lc.d_rho << ","
        << lc.box.x << "," << lc.box.y << ","
        << lc.box.width << "," << lc.box.height << "\n";
    f.close();
}

struct Detection
{
    float cx, cy;   // 원본 좌표 기준 중심점
    float w, h;     // 원본 기준 크기
    float conf;     // 신뢰도
    int cls;        // 0=H, 1=V
    Point2f corners[4];  // 시계방향: ptMin 기준 4꼭짓점
};
Mat maskForYolo(const Mat& image_bgr, const vector<LineCandidate>& avg_lines, float offset)
{
    const int H = image_bgr.rows;
    const int W = image_bgr.cols;
    // 마스크 영상 공간 만들기! (0=배경(114), 255=살릴 픽셀)
    Mat mask = Mat::zeros(H, W, CV_8UC1);
    for (const auto& lc : avg_lines)
    {
        float cos_t = cos(lc.theta);
        float sin_t = sin(lc.theta);
        for (int y = 0; y < H; ++y)
        {
            uchar* mp = mask.ptr<uchar>(y);
            for (int x = 0; x < W; ++x)
            {
                float dist = abs((float)x * cos_t + (float)y * sin_t + lc.rho);
                if (dist <= offset) mp[x] = 255;
            }
        }
    }
    // 결과 이미지
    Mat result(H, W, CV_8UC3, Scalar(0, 0, 0));
    image_bgr.copyTo(result, mask);   // mask=255인 픽셀만 원본 복사
    return result;
}
vector<Detection> inferOBB(torch::jit::script::Module& model, const Mat& frame, int imgSize, float confThr, int targetCls, torch::Device device)
{
    int origH = frame.rows, origW = frame.cols;
    // 레터박스
    float scale = min((float)imgSize / origH, (float)imgSize / origW);
    int newH = (int)(origH * scale);
    int newW = (int)(origW * scale);
    int padH = (imgSize - newH) / 2;
    int padW = (imgSize - newW) / 2;
    Mat resized;
    resize(frame, resized, Size(newW, newH));
    Mat processed(imgSize, imgSize, CV_8UC3, Scalar(114, 114, 114));
    resized.copyTo(processed(Rect(padW, padH, newW, newH)));
    // 텐서변환 + 추론
    Mat imgRGB;
    cvtColor(processed, imgRGB, COLOR_BGR2RGB);
    imgRGB.convertTo(imgRGB, CV_32F, 1.0 / 255.0);
    auto tensor = torch::from_blob(imgRGB.data, { 1, imgSize, imgSize, 3 }, torch::kFloat32).permute({ 0, 3, 1, 2 }).contiguous().to(device);
    vector<torch::jit::IValue> inputs = { tensor };
    auto output = model.forward(inputs).toTensor().cpu();
    // 필터링 + 역변환
    vector<Detection> detections;
    int numDet = output.size(1);
    for (int i = 0; i < numDet; i++)
    {
        float conf = output[0][i][4].item<float>();
        int   cls = (int)output[0][i][5].item<float>();
        if (conf < confThr) continue;
        if (cls != 0 && cls != 1) continue;  // H, V만!
        float x1 = output[0][i][0].item<float>();
        float y1 = output[0][i][1].item<float>();
        float x2 = output[0][i][2].item<float>();
        float y2 = output[0][i][3].item<float>();
        Detection det;
        det.cx = ((x1 + x2) / 2.0f - padW) / scale;
        det.cy = ((y1 + y2) / 2.0f - padH) / scale;
        det.w = (x2 - x1) / scale;
        det.h = (y2 - y1) / scale;
        det.conf = conf;
        det.cls = cls;
        detections.push_back(det);
    }
    return detections;
}
vector<Detection> clusterToRebarBoxes(const vector<LineCandidate>& avg_lines, const vector<Detection>& detections, float offset)
{
    const float DEG45 = (float)CV_PI / 4.f;
    const float DEG135 = 3.f * (float)CV_PI / 4.f;
    vector<Detection> result;
    for (const auto& lc : avg_lines)
    {
        float cos_t = cos(lc.theta);
        float sin_t = sin(lc.theta);
        // 이 직선이 H인지 V인지
        int lineCls = (lc.theta < DEG45 || lc.theta > DEG135) ? 1 : 0; // 1=V, 0=H
        // 소속 박스 수집
        vector<Detection> belonged;
        for (const auto& det : detections)
        {
            if (det.cls != lineCls) continue; // ★직선과 같은 라벨만!
            float dist = abs(det.cx * cos_t + det.cy * sin_t + lc.rho);
            if (dist <= offset) belonged.push_back(det);
        }
        if (belonged.size() < 2) continue; // 평균땜시 놔야함
        // 평균 w, h
        float avg_w = 0.f, avg_h = 0.f;
        for (const auto& d : belonged) { avg_w += d.w; avg_h += d.h; }
        avg_w /= belonged.size();
        avg_h /= belonged.size();
        // ★ 1) PCA로 방향벡터 추출 (2D)
        Mat pts_mat((int)belonged.size(), 2, CV_32F);
        for (int i = 0; i < (int)belonged.size(); i++)
        {
            pts_mat.at<float>(i, 0) = belonged[i].cx;
            pts_mat.at<float>(i, 1) = belonged[i].cy;
        }
        PCA pca(pts_mat, Mat(), PCA::DATA_AS_ROW);
        Point2f dir(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
        Point2f centroid(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));
        auto proj = [&](const Detection& d) { return (d.cx - centroid.x) * dir.x + (d.cy - centroid.y) * dir.y; };
        float tMin = proj(*min_element(belonged.begin(), belonged.end(), [&](const Detection& a, const Detection& b) { return proj(a) < proj(b); }));
        float tMax = proj(*max_element(belonged.begin(), belonged.end(), [&](const Detection& a, const Detection& b) { return proj(a) < proj(b); }));
        Point2f ptMin = centroid + dir * tMin;
        Point2f ptMax = centroid + dir * tMax;
        // 2) 철근 법선벡터 (두께)
        Point2f normal(-dir.y, dir.x);
        // H=avg_h 방향으로 확장, V=avg_w 방향으로 확장
        float half_len = ((lineCls == 0) ? avg_w : avg_h) / 2.f;    // 직선방향 확장
        float half_thick = ((lineCls == 0) ? avg_h : avg_w) / 2.f;  // 수직방향 확장
        Point2f ptMinExt = ptMin - dir * half_len;  // 말단에서 직선방향 확장
        Point2f ptMaxExt = ptMax + dir * half_len;
        Detection rb;
        rb.cls = lineCls;
        rb.corners[0] = ptMinExt - normal * half_thick;
        rb.corners[1] = ptMinExt + normal * half_thick;
        rb.corners[2] = ptMaxExt + normal * half_thick;
        rb.corners[3] = ptMaxExt - normal * half_thick;
        result.push_back(rb);
    }
    return result;
}
vector<Point2f> computeIntersections(const vector<Detection>& rebarBoxes, int imgW, int imgH)
{
    // H/V 분리
    vector<Detection> h_boxes, v_boxes;
    for (const auto& rb : rebarBoxes)
    {
        if (rb.cls == 0) h_boxes.push_back(rb);
        else             v_boxes.push_back(rb);
    }
    vector<Point2f> intersections;
    for (const auto& h : h_boxes)
    {
        for (const auto& v : v_boxes)
        {
            // 각 OBB를 마스크로 변환
            Mat h_mask = Mat::zeros(imgH, imgW, CV_8UC1);
            Mat v_mask = Mat::zeros(imgH, imgW, CV_8UC1);
            vector<Point> h_pts, v_pts;
            for (int i = 0; i < 4; i++)
            {
                h_pts.push_back(Point(cvRound(h.corners[i].x), cvRound(h.corners[i].y)));
                v_pts.push_back(Point(cvRound(v.corners[i].x), cvRound(v.corners[i].y)));
            }
            fillPoly(h_mask, vector<vector<Point>>{h_pts}, Scalar(255));
            fillPoly(v_mask, vector<vector<Point>>{v_pts}, Scalar(255));
            // 교차 영역
            Mat inter_mask;
            bitwise_and(h_mask, v_mask, inter_mask);
            if (countNonZero(inter_mask) == 0) continue;
            // 교차 영역 중심점
            Moments mo = moments(inter_mask, true);
            if (mo.m00 < 1e-6) continue;
            Point2f center((float)(mo.m10 / mo.m00), (float)(mo.m01 / mo.m00));
            intersections.push_back(center);
        }
    }
    return intersections;
}

// ============================================================================
//  main ─ 로봇(Orbbec) 프레임을 WLAN 으로 받아 동일 파이프라인 실행
// ============================================================================

// ── 로봇(서버) 접속 정보 ── (환경에 맞게 수정)
static const string ROBOT_IP   = "100.99.144.107";  // 로봇 rebar-jetson Tailscale IP
static const int    ROBOT_PORT = 5001;              // frame_stream_server.py 포트

int main()
{
    // ── 카메라(원격) 연결 ──
    FrameClient cam;
    if (!cam.connect(ROBOT_IP, ROBOT_PORT))
    {
        cerr << "로봇 프레임 서버 접속 실패: " << ROBOT_IP << ":" << ROBOT_PORT << endl;
        return 1;
    }
    cout << "로봇 프레임 서버 연결됨: " << ROBOT_IP << ":" << ROBOT_PORT << endl;

    // 내참(fx,fy,cx,cy)은 프레임 헤더에서 매번 갱신 (서버=컬러 카메라 내참)
    float fx = 0, fy = 0, cx = 0, cy = 0;

    // ── 욜로26 ──
    string modelPath = "C:\\Z_Project1\\rebar_ex1\\weights\\best.torchscript";
    torch::Device device(torch::kCPU);
    torch::jit::script::Module model;
    try { model = torch::jit::load(modelPath, device); model.eval(); }
    catch (const c10::Error& e) { cerr << "모델 로드 실패: " << e.what() << endl; return -1; }

    // ★ 실시간 + 캡쳐/CSV
    Mat depth_f32, image_bgr;       // OPCV- mm단위 깊이맵과 원본 이미지 (서버 수신)
    FrameClient::Intr K;
    while (true)
    {
        // (1) 로봇에서 최신 프레임 수신 (원격 grab)
        if (!cam.grab(image_bgr, depth_f32, K))
        {
            if (!cam.connected()) { cerr << "연결 끊김" << endl; break; }
            waitKey(5);
            continue;
        }
        fx = K.fx; fy = K.fy; cx = K.cx; cy = K.cy;

        // (2) 깊이맵 시각화 (실시간)
        Mat depth_visual = visualizeDepthMap(depth_f32, DEPTH_MIN, DEPTH_MAX, COLORMAP_JET);
        imshow("Depth Map", depth_visual);
        int key = waitKey(1);

        if (key == ' ')
        {
            auto t_start1 = chrono::high_resolution_clock::now();
            // (3) 평면
            PlaneResult cloud = depthToPointCloud(depth_f32, fx, fy, cx, cy, DEPTH_MIN, DEPTH_MAX);
            vector<PlanePoint> hough_pts = sampleHoughPlane(cloud, ITERATIONS1);
            hough_pts = clusterHoughPlane(hough_pts);
            PlaneResult plane = extractPlaneByRank(hough_pts);
            vector<PlanePoint> layer_pts = computeDistances(cloud, plane, ITERATIONS2);
            layer_pts = clusterLayers(layer_pts);
            PlaneResult layer_plane = extractLayerByRank(layer_pts, LAYER_RANK);
            savePlaneResultCsv(plane, image_bgr, fx, fy, cx, cy, "C:/image_640x360/1/plane_result.csv");
            savePlaneResultCsv(layer_plane, image_bgr, fx, fy, cx, cy, "C:/image_640x360/1/layer_result.csv");
            saveHoughPlaneCsv(hough_pts, "C:/image_640x360/1/hough_plane.csv");
            saveLayerCSV(layer_pts, "C:/image_640x360/1/layer_pts.csv");
            auto t_end1 = chrono::high_resolution_clock::now();
            printf("plane time: %.2f ms\n", chrono::duration<float, milli>(t_end1 - t_start1).count());

            auto t_start2 = chrono::high_resolution_clock::now();
            // (4) 직선
            vector<Point2f> pixels = projectToImage(layer_plane.pts, fx, fy, cx, cy);
            vector<LineCandidate> candidates = sampleHoughLine(pixels, ITERATIONS3);
            vector<LineCandidate> knn_result = estimateWindowSize(candidates);
            float d_theta = knn_result[0].d_theta; float d_rho = knn_result[0].d_rho;
            float diag = sqrtf((float)(image_bgr.cols * image_bgr.cols + image_bgr.rows * image_bgr.rows));
            vector<LineCandidate> hessian = computeHessianOnKernel(candidates, d_theta, d_rho, -diag, diag);
            vector<LineCandidate> selected = selectByElbowKMeans(hessian);
            vector<LineCandidate> avg_lines = clusterAndAverageLines(selected, d_theta, d_rho);
            savePixelCsv(pixels, image_bgr, "C:/image_640x360/1/pixels.csv");
            saveLineCsv(candidates, "C:/image_640x360/1/hough_line.csv");
            saveLineCsv(knn_result, "C:/image_640x360/1/knn_result.csv");
            saveLineCsv(hessian, "C:/image_640x360/1/hessian.csv");
            saveLineCsv(selected, "C:/image_640x360/1/selected.csv");
            saveLineCsv(avg_lines, "C:/image_640x360/1/line_result.csv");
            auto t_end2 = chrono::high_resolution_clock::now();
            printf("line time: %.2f ms\n", chrono::duration<float, milli>(t_end2 - t_start2).count());

            auto t_start3 = chrono::high_resolution_clock::now();
            // (5) 철근-교차 -> 3D위치는 savePixelCsv로 추출해서 접근
            Mat yolo_input = maskForYolo(image_bgr, avg_lines, OFFSET);
            vector<Detection> detections = inferOBB(model, yolo_input, 640, 0.001f, 0, device);
            vector<Detection> rebarBoxes = clusterToRebarBoxes(avg_lines, detections, OFFSET);
            vector<Point2f> intersections = computeIntersections(rebarBoxes, image_bgr.cols, image_bgr.rows);
            savePixelCsv(intersections, image_bgr, "C:/image_640x360/1/intersection.csv");
            auto t_end3 = chrono::high_resolution_clock::now();
            printf("rebar time: %.2f ms\n", chrono::duration<float, milli>(t_end3 - t_start3).count());

            // (6) 영상 캡쳐
            imwrite("C:/image_640x360/1/1_original.png", image_bgr);
            imwrite("C:/image_640x360/1/2_depth.png", depth_visual);
            Mat result_img = Mat::zeros(image_bgr.size(), image_bgr.type());
            for (const Point2f& px : pixels)
            {
                int u = cvRound(px.x); int v = cvRound(px.y);
                if (u < 0 || u >= result_img.cols || v < 0 || v >= result_img.rows) continue;
                result_img.at<Vec3b>(v, u) = image_bgr.at<Vec3b>(v, u);
            }
            imwrite("C:/image_640x360/1/3_result.png", result_img);
            Mat yolo_vis = yolo_input.clone();
            for (const auto& det : detections)
            {
                int x1 = (int)(det.cx - det.w / 2); int y1 = (int)(det.cy - det.h / 2);
                int x2 = (int)(det.cx + det.w / 2); int y2 = (int)(det.cy + det.h / 2);
                Scalar color = (det.cls == 0) ? Scalar(255, 0, 0) : Scalar(0, 0, 255);
                rectangle(yolo_vis, Point(x1, y1), Point(x2, y2), color, 2);
                putText(yolo_vis, (det.cls == 0) ? "H" : "V", Point(x1, y1 - 5), FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
            imwrite("C:/image_640x360/1/4_yoloi.png", yolo_input);
            imwrite("C:/image_640x360/1/5_yolo_box.png", yolo_vis);
            Mat vis = yolo_input.clone();
            for (const auto& rb : rebarBoxes)
            {
                vector<Point> pts;
                for (int i = 0; i < 4; i++) pts.push_back(Point(cvRound(rb.corners[i].x), cvRound(rb.corners[i].y)));
                Scalar color = (rb.cls == 0) ? Scalar(255, 0, 0) : Scalar(0, 0, 255);
                polylines(vis, pts, true, color, 3, LINE_AA);
            }
            imwrite("C:/image_640x360/1/6_rebar.png", vis);
            Mat vis_inter = yolo_input.clone();
            for (const auto& pt : intersections) circle(vis_inter, Point(cvRound(pt.x), cvRound(pt.y)), 10, Scalar(0, 255, 0), -1);
            imwrite("C:/image_640x360/1/7_intersection.png", vis_inter);
        }

        if (key == 27) break; // ESC → 종료
    }
    cam.disconnect();
    return 0;
}
