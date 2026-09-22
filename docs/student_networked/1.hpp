#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <fstream>
#include <set>
#include <tuple>
#include <map>
#include <algorithm>
using namespace std;
using namespace cv;

// 반복 횟수 + 좌표 스케일 + 임계 거리
const int ITERATIONS1 = 10000;      // 1) 3차원 평면 후보 샘플링 횟수
const int ITERATIONS2 = 50000;      //    1차원 거리(층) 샘플링 횟수
const int LAYER_RANK = 1;           //    목표 층 랭크(0=상단층, 1=하단층)
const int ITERATIONS3 = 2500;       // 2) 2차원 직선 후보 샘플링 횟수
const int STEP = 30;                //    커널 격자 크기
const int ROI = 120;                //    ROI 영상 크기
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
            else val = (val - depth_min) / (depth_max - depth_min) * 255.0f; // 정규화
        }
    }
    Mat depth_normalized;
    depth_clamped.convertTo(depth_normalized, CV_8U);
    Mat depth_colored;
    applyColorMap(depth_normalized, depth_colored, colormap);
    return depth_colored;
}



// ─────────────────────────────────────────────────────────────────────────
// [1단계] 층 분류
// ─────────────────────────────────────────────────────────────────────────
struct PlanePoint           // 3차원 평면 후보 + 랭크
{
    Vec3f pt0, pt1, pt2;    // 표본 3점(pt0만 이후 1차원 거리 계산에 재사용)
    float theta;            // [0, π] 방위각
    float phi;              // [0, π/2] 극각

    float d;                // 원점에서 평면까지 거리(1차원 거리 계산 시 재사용)

    Vec3f center;           // 군집 중심점(theta,phi,d 기준)
    int cluster_id;         // 군집 인덱스(랭크)
};
struct PlaneResult          // 바닥/목표층 결과 + PCD
{
    vector<Vec3f> pts;      // PCD
    Vec3f normal;           // 평면 법선
    Vec3f mean;             // 평면 대표점
};
PlaneResult depthToPointCloud(const Mat& depth_f32, float fx, float fy, float cx, float cy, float depth_min, float depth_max)
{
    PlaneResult cloud;
    cloud.pts.reserve(depth_f32.rows * depth_f32.cols);
    for (int r = 0; r < depth_f32.rows; ++r)
    {
        const float* row_ptr = depth_f32.ptr<float>(r); // 행주소를 미리계산, 매번픽셀마다 계산안하게(최적화)
        for (int c = 0; c < depth_f32.cols; ++c)
        {
            float z = row_ptr[c];
            if (!isfinite(z) || z < depth_min || z > depth_max) continue;
            // 역투영 공식
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
        // 랜덤 3점 추출 (중복 제외)
        int i0, i1, i2;
        i0 = dist(rng);
        do { i1 = dist(rng); } while (i1 == i0);
        do { i2 = dist(rng); } while (i2 == i0 || i2 == i1);
        Vec3f p0 = cloud.pts[i0];
        Vec3f p1 = cloud.pts[i1];
        Vec3f p2 = cloud.pts[i2];
        // 두 벡터의 외적 = 법선
        Vec3f v1 = p1 - p0;
        Vec3f v2 = p2 - p0;
        Vec3f n = v1.cross(v2);
        float len = norm(n);
        if (len < 1e-6f) continue;
        n /= len;
        if (n[2] < 0) n = -n; // 카메라 방향(z축 양수) 통일
        float theta = atan2f(n[1], n[0]);
        float phi = acosf(clamp(n[2], -1.f, 1.f));
        if (theta < 0) theta += (float)CV_PI; // theta 범위 통일 [0, π]
        float d = -(n.dot(p0)); // 평면방정식 상수항 계산
        // 결과 저장
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
vector<PlanePoint> clusterHoughPlane(vector<PlanePoint>& hough_pts, int k_min = 1, int k_max = 5)
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
    // (1) k_min~k_max 범위에서 k-means 반복
    vector<float> wcss;
    vector<Mat> all_labels;
    vector<Mat> all_centers;
    for (int k = k_min; k <= k_max; ++k)
    {
        Mat labels, centers;
        float w = kmeans(data, k, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 20, 1), 10, KMEANS_PP_CENTERS, centers);
        wcss.push_back(w);
        all_labels.push_back(labels.clone());
        all_centers.push_back(centers.clone());
        //printf("k=%d, 3차원 wcss=%.2f\n", k, w);
    }
    // (2) 로그 곡률 기반 best_k 탐색 (Elbow)
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
    // (3) best_k 결과로 최종 확정
    Mat& labels = all_labels[best_k - k_min];
    Mat& centers = all_centers[best_k - k_min];
    for (int i = 0; i < N; ++i) hough_pts[i].cluster_id = labels.at<int>(i);
    // 클러스터별 개수 카운트 -> 개수 내림차순 정렬 -> 원래 label을 랭크로 재매핑
    vector<pair<int, int>> counts(best_k);
    for (int k = 0; k < best_k; ++k) counts[k] = { 0, k };
    for (int i = 0; i < N; ++i) counts[hough_pts[i].cluster_id].first++;
    sort(counts.begin(), counts.end(), [](const pair<int, int>& a, const pair<int, int>& b) { return a.first > b.first; });
    vector<int> label_to_rank(best_k);
    for (int r = 0; r < best_k; ++r) label_to_rank[counts[r].second] = r;
    // 결과 저장
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
    // 1) 바닥면(랭크0) 클러스터 중심 획득
    Vec3f center;
    for (const PlanePoint& pp : hough_pts)
    {
        if (pp.cluster_id != 0) continue;
        center = pp.center;
        break;
    }
    // 2) 구면좌표 -> 법선벡터 변환
    Vec3f normal(cosf(center[0]) * sinf(center[1]), sinf(center[0]) * sinf(center[1]), cosf(center[1]));
    // 3) PCD의 pt0,pt1,pt2 중복제거 취합
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
    // 4) 대표점 = 법선 방향으로 원점에서 d만큼
    float d = center[2];
    Vec3f mean = normal * (-d);
    // 결과 저장
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
        // 결과 저장 (pt0에 넣어서 재사용)
        PlanePoint lp;
        lp.pt0 = Vec3f(p[0], p[1], p[2]);
        lp.d = d;
        layer_pts.push_back(lp);
    }
    return layer_pts;
}
vector<PlanePoint> clusterLayers(vector<PlanePoint>& layer_pts, int k_min = 1, int k_max = 5)
{
    int N = (int)layer_pts.size();
    Mat data(N, 1, CV_32F);
    for (int i = 0; i < N; ++i) data.at<float>(i, 0) = layer_pts[i].d;
    // 1) k_min~k_max 범위에서 k-means 반복
    vector<float> wcss;
    vector<Mat> all_labels;
    vector<Mat> all_centers;
    for (int k = k_min; k <= k_max; ++k)
    {
        Mat labels, centers;
        float w = kmeans(data, k, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 20, 1), 10, KMEANS_PP_CENTERS, centers);
        wcss.push_back(w);
        all_labels.push_back(labels.clone());
        all_centers.push_back(centers.clone());
        //printf("k=%d, 1차원 wcss=%.2f\n", k, w);
    }
    // 2) 로그 곡률 기반 best_k 탐색 (Elbow)
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
    // 3) best_k 결과로 최종 확정
    Mat& labels = all_labels[best_k - k_min];
    Mat& centers = all_centers[best_k - k_min];
    // d값이 작을수록 랭크가 낮도록 정렬 (상단근=0, 하단근=1, 바닥=2 ...)
    vector<pair<float, int>> center_vals(best_k);
    for (int c = 0; c < best_k; ++c) center_vals[c] = { centers.at<float>(c, 0), c };
    sort(center_vals.begin(), center_vals.end(), [](const pair<float, int>& a, const pair<float, int>& b) { return a.first < b.first; });
    vector<int> label_to_rank(best_k);
    for (int r = 0; r < best_k; ++r) label_to_rank[center_vals[r].second] = r;
    // 결과 저장: cluster_id 재매핑
    for (int i = 0; i < N; ++i)
    {
        int original_label = labels.at<int>(i);
        layer_pts[i].cluster_id = label_to_rank[original_label];
    }
    printf("best_k=%d\n", best_k);
    return layer_pts;
}
PlaneResult extractLayerByRank(const vector<PlanePoint>& layer_pts, int rank)
{
    // 1) 목표 랭크에 해당하는 PCD 추출
    vector<Vec3f> pts;
    for (const PlanePoint& lp : layer_pts)
    {
        if (lp.cluster_id != rank) continue;
        pts.push_back(lp.pt0);
    }
    // 2) PCA로 평면 재적합
    Mat pts_mat((int)pts.size(), 3, CV_32F);
    for (int i = 0; i < (int)pts.size(); ++i)
    {
        pts_mat.at<float>(i, 0) = pts[i][0];
        pts_mat.at<float>(i, 1) = pts[i][1];
        pts_mat.at<float>(i, 2) = pts[i][2];
    }
    PCA pca(pts_mat, Mat(), PCA::DATA_AS_ROW);
    // 가장 작은 분산 방향 = 법선벡터와 동일방향 취급
    Vec3f normal(pca.eigenvectors.at<float>(2, 0), pca.eigenvectors.at<float>(2, 1), pca.eigenvectors.at<float>(2, 2));
    if (normal[2] < 0) normal = -normal;
    Vec3f mean(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1), pca.mean.at<float>(0, 2));
    // 결과 저장
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
    f << "theta,phi,d,cluster_id,center_theta,center_phi,center_d\n";
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



// ─────────────────────────────────────────────────────────────────────────
// [2단계] 선 분류
// ─────────────────────────────────────────────────────────────────────────
struct LineCandidate
{
    float theta;        // [0, π] 방위각
    float rho;          // 원점-직선 거리

    float score;        // 헤시안 크기

    float d_theta;      // 윈도우 커널 크기(θ) (헤시안커널에선 슬라이딩 중심으로 재사용)
    float d_rho;        // 윈도우 커널 크기(ρ) (헤시안커널에선 슬라이딩 중심으로 재사용)

    Rect2f box;          // 커널 박스(t_min, r_min, width, height)
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
        // 결과 저장
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
        // 랜덤 2점 추출 (중복 제외)
        int i0, i1;
        i0 = dist(rng);
        do { i1 = dist(rng); } while (i1 == i0);
        Point2f p0 = pixels[i0];
        Point2f p1 = pixels[i1];
        // 방향벡터 -> 법선벡터
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-6f) continue;
        // 부호 통일 (ny, theta 범위 통일) (3,4상한->1,2상한)
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
    // 모든 점끼리 전체 거리 계산 (pair: 거리 + 인덱스)
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
    // 1) best_ref: 주변이 가장 밀집한 점 인덱스
    float min_cum = FLT_MAX;
    int best_ref = -1;
    for (int i = 0; i < N; ++i)
    {
        float cum = 0.f;
        for (int ki = 0; ki < (int)sqrtf(N); ++ki) cum += all_dists[i][ki].first;
        if (cum < min_cum) { min_cum = cum; best_ref = i; }
    }
    // 2) best_K: best_ref의 k별 거리곡률로 결정
    int best_K = 1;
    float max_kappa = -FLT_MAX;
    for (int idx = 1; idx <= (int)sqrtf(N) * 10; ++idx)
    {
        float dp = all_dists[best_ref][idx - 1].first;
        float dc = all_dists[best_ref][idx].first;
        float dn = all_dists[best_ref][idx + 1].first;
        float dy = (dn - dp) / 2.f;
        float d2y = dp - 2.f * dc + dn;
        float kappa = fabsf(d2y) / powf(1.f + dy * dy, 1.5f);
        if (kappa > max_kappa) { max_kappa = kappa; best_K = idx + 1; }
        //printf("k=%d, d_k=%.4f, kappa=%.6f\n", idx + 1, dc, kappa);
    }
    printf("best_K=%d\n", best_K);
    // best_ref와 best_K를 이용해 윈도우 크기 산출 (ref_dists 중복 계산 방지)
    vector<LineCandidate> result;
    LineCandidate lc_ref;  // best_ref 원본 추가 (중심점)
    lc_ref.theta = points[best_ref].theta;
    lc_ref.rho = points[best_ref].rho;
    result.push_back(lc_ref);  // index 0

    float tw_min = FLT_MAX, tw_max = -FLT_MAX;
    float rw_min = FLT_MAX, rw_max = -FLT_MAX;
    for (int k = 0; k < best_K; ++k)
    {
        int idx = all_dists[best_ref][k].second;  // 그 인덱스 바로 참조
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
    // 슬라이딩 커널 파라미터 설정
    float theta_pad = d_theta / 2.f;
    float rho_pad = d_rho / 2.f;
    int n_theta = STEP;
    int n_rho = STEP;
    float theta_stride = (float)CV_PI / (float)n_theta;
    float rho_stride = (rho_max - rho_min) / (float)n_rho;
    // 경계 랩어라운드 패딩 추가
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
    // 슬라이딩 윈도우 전체 순회
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
            // 현재 커널 안에 속한 점만 취합
            vector<pair<Point2f, Point2f>> inKernel;  // (정규화좌표, 원본좌표)
            for (const auto& hp : paddedPoints)
            {
                if (hp.theta > t_min && hp.theta < t_max && hp.rho > r_min && hp.rho < r_max)
                {
                    float thn = ((hp.theta - t_min) / (t_max - t_min) - 0.5f) * 0.5f * 10.f;
                    float rhn = ((hp.rho - r_min) / (r_max - r_min) - 0.5f) * 0.5f * 10.f;
                    inKernel.push_back({ Point2f(thn, rhn), Point2f(hp.theta, hp.rho) });
                }
            }
            if ((int)inKernel.size() < 2) continue;
            // 정규화좌표/원본좌표 각각 평균 계산
            Point2f mun(0, 0), mu(0, 0);
            for (const auto& wp : inKernel)
            {
                mun += wp.first;
                mu += wp.second;
            }
            mun *= (1.f / (float)inKernel.size());
            mu *= (1.f / (float)inKernel.size());
            // 가우시안 가중 헤시안 행렬 계산
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
            // 최소 고유값(능선 강도) 계산
            float trace = h11 + h22;
            float det = h11 * h22 - h12 * h12;
            float disc = trace * trace - 4.f * det;
            if (disc < 0) continue;
            float lambda = (trace - sqrtf(disc)) * 0.5f;
            // 결과 저장
            LineCandidate res;
            res.score = (lambda < 0.f) ? -lambda : 0.f; // 음수 고유값일수록 능선(직선) 강함
            res.theta = mu.x;
            res.rho = mu.y;
            res.d_theta = center_theta;   // 윈도우 중심 재활용
            res.d_rho = center_rho;       // 윈도우 중심 재활용
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
    kmeans(data, num_clusters, labels, TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 20, 1), 10, KMEANS_PP_CENTERS, centers);
    // score 내림차순 정렬 후 상위 num_top_select 클러스터만 채택
    vector<pair<float, int>> center_vals(num_clusters);
    for (int c = 0; c < num_clusters; ++c) center_vals[c] = { centers.at<float>(c, 0), c };
    sort(center_vals.begin(), center_vals.end(), [](const pair<float, int>& a, const pair<float, int>& b) { return a.first > b.first; });
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
    // 1차 DBSCAN
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
    // 클러스터별 평균값 계산
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
            cluster_means[c].x /= cluster_cnt[c];
            cluster_means[c].y /= cluster_cnt[c];
        }
    // 경계(0/π 부근) 탐색 후 0쪽 부호 변환
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
    // 2차 DBSCAN (경계 클러스터 평균끼리만) 병합
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
    // 최종 라벨 재정리 + 평균값 갱신
    map<int, int> remap;
    int new_cid = 0;
    for (int c = 1; c <= cluster_id; ++c)
    {
        int root = merge_map[c];
        if (remap.find(root) == remap.end()) remap[root] = ++new_cid;
    }
    cluster_id = new_cid;
    // 최종 병합된 평균 결과 계산
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
        // wrap-around 복원
        if (lc.theta < 0.f) { lc.theta += (float)CV_PI; lc.rho = -lc.rho; }
        avg_lines.push_back(lc);
    }
    return avg_lines;
}
vector<Point2f> computeLineIntersections(const vector<LineCandidate>& avg_lines, int imgW, int imgH)
{
    int N = (int)avg_lines.size();
    vector<Point2f> pts;
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j)
        {
            float cosA = cosf(avg_lines[i].theta), sinA = sinf(avg_lines[i].theta);
            float cosB = cosf(avg_lines[j].theta), sinB = sinf(avg_lines[j].theta);
            float D = cosA * sinB - sinA * cosB;
            if (fabsf(D) < 1e-6f) continue; // 평행 -> 교차 없음
            Point2f pt;
            pt.x = (avg_lines[j].rho * sinA - avg_lines[i].rho * sinB) / D;
            pt.y = (avg_lines[i].rho * cosB - avg_lines[j].rho * cosA) / D;
            if (pt.x < 0 || pt.x >= imgW || pt.y < 0 || pt.y >= imgH) continue; // 화면 밖 교차는 제외
            pts.push_back(pt);
        }
    return pts;
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



// ─────────────────────────────────────────────────────────────────────────
// [3단계] 결속 위치 + 상태
// 1. AI 분류 - computeIntersections
// 2. 3차원 교차점 -> 카메라 시점
// 3. 캘리+투시
// ─────────────────────────────────────────────────────────────────────────