// ─────────────────────────────────────────────────────────────────────────
//  main_networked.cpp
//  기존 ZED 로컬 카메라 대신, 로봇(Jetson)에서 WLAN(TCP)으로 프레임을 받아
//  동일한 교차점 검출 파이프라인을 실행한다.
//
//  ■ 원본 대비 바뀐 것은 "카메라 입력부" 뿐. 알고리즘 함수 700여 줄은 그대로.
//     - #include <sl/Camera.hpp>        →  제거
//     - sl::Camera 초기화 / 설정 / grab  →  FrameClient 로 교체
//     - depth_f32 / image_bgr / fx..cy   →  서버에서 받은 값 사용 (의미 동일)
//
//  ■ 준비 (한 번만):
//     1) 학생 원본 파일(결속위치코드_전재하)에서
//          (a) 3행  #include <sl/Camera.hpp>  삭제
//          (b) 맨 아래 int main(){ ... } 전체 삭제
//        하고 파일명을 rebar_pipeline.hpp 로 저장 (구조체/함수/상수만 남김).
//     2) frame_client.hpp 를 같은 폴더에 둔다.
//     3) 이 파일(main_networked.cpp)을 빌드 대상으로 추가.
//
//  ■ 빌드 링크: OpenCV + Torch(기존과 동일) + ws2_32.  ZED SDK 불필요.
//  ■ 실행: 로봇에서 frame_stream_server.py 를 먼저 띄운 뒤 이 프로그램 실행.
// ─────────────────────────────────────────────────────────────────────────
// ★ frame_client.hpp 를 "가장 먼저" 포함 → winsock2.h 가 torch/windows.h 보다 앞서서
//    winsock 재정의 충돌 + min/max 매크로 충돌을 원천 차단한다.
#include "frame_client.hpp"

// 원본이 <sl/Camera.hpp> 를 통해 "간접 포함"하던 표준 헤더들 — sl 제거로 사라졌으니 명시.
#include <fstream>              // ofstream (saveXxxCsv)
#include <set>                  // set<tuple<...>>
#include <map>                  // map<int,int>
#include <tuple>               // make_tuple
#include <algorithm>            // sort, min_element, find ...

#include "rebar_pipeline.hpp"   // ← 학생 원본에서 sl include + main 제거한 것 (알고리즘)

// ── 로봇(서버) 접속 정보 ───  (환경에 맞게 수정)
static const char* ROBOT_IP = "192.168.0.10";   // 로봇 Jetson IP (또는 Tailscale IP)
static const int   ROBOT_PORT = 5001;           // frame_stream_server.py 포트

int main()
{
    // ── 카메라(원격) 연결 ───
    FrameClient cam;
    if (!cam.connect(ROBOT_IP, ROBOT_PORT))
    {
        cerr << "로봇 프레임 서버 접속 실패: " << ROBOT_IP << ":" << ROBOT_PORT << endl;
        return 1;
    }
    cout << "로봇 프레임 서버 연결됨: " << ROBOT_IP << ":" << ROBOT_PORT << endl;

    // 내참(fx,fy,cx,cy)은 프레임 헤더에서 매번 갱신됨 (서버=컬러 카메라 내참)
    float fx = 0, fy = 0, cx = 0, cy = 0;

    // ── 욜로26 (기존과 동일) ───
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
            // 아직 프레임 없음 / 일시적 수신 실패 → 잠깐 대기 후 재시도
            if (!cam.connected()) { cerr << "연결 끊김" << endl; break; }
            waitKey(5);
            continue;
        }
        fx = K.fx; fy = K.fy; cx = K.cx; cy = K.cy;

        // (2) 깊이맵 시각화 (실시간) — 기존과 동일
        Mat depth_visual = visualizeDepthMap(depth_f32, DEPTH_MIN, DEPTH_MAX, COLORMAP_JET);
        imshow("Depth Map", depth_visual);
        int key = waitKey(1);



        if (key == ' ')
        {
            auto t_start1 = chrono::high_resolution_clock::now();
            // (3) 평면
            PlaneResult cloud = depthToPointCloud(depth_f32, fx, fy, cx, cy, DEPTH_MIN, DEPTH_MAX);
            vector<PlanePoint> hough_pts = sampleHoughPlane(cloud, ITERATIONS1);                                            // 3차원   배경
            hough_pts = clusterHoughPlane(hough_pts);                                                                       // 불안 층 군집
            PlaneResult plane = extractPlaneByRank(hough_pts);                                                              // 바닥 층
            vector<PlanePoint> layer_pts = computeDistances(cloud, plane, ITERATIONS2);                                     // 1차원   배경
            layer_pts = clusterLayers(layer_pts);                                                                           // 안정 층 군집
            PlaneResult layer_plane = extractLayerByRank(layer_pts, LAYER_RANK);                                            // 목표 층
            // CSV 파일
            savePlaneResultCsv(plane, image_bgr, fx, fy, cx, cy, "C:/image_640x360/1/plane_result.csv");                    // 1)바닥  정보
            savePlaneResultCsv(layer_plane, image_bgr, fx, fy, cx, cy, "C:/image_640x360/1/layer_result.csv");              // 1)목표  정보
            saveHoughPlaneCsv(hough_pts, "C:/image_640x360/1/hough_plane.csv");                                             // 2)3차원 정보
            saveLayerCSV(layer_pts, "C:/image_640x360/1/layer_pts.csv");                                                    // 3)1차원 정보
            auto t_end1 = chrono::high_resolution_clock::now();
            printf("plane time: %.2f ms\n", chrono::duration<float, milli>(t_end1 - t_start1).count());



            auto t_start2 = chrono::high_resolution_clock::now();
            // (4) 직선
            vector<Point2f> pixels = projectToImage(layer_plane.pts, fx, fy, cx, cy);
            vector<LineCandidate> candidates = sampleHoughLine(pixels, ITERATIONS3);                                        // 2차원   배경
            vector<LineCandidate> knn_result = estimateWindowSize(candidates);                                              // 단밀집  지역
            float d_theta = knn_result[0].d_theta; float d_rho = knn_result[0].d_rho;
            float diag = sqrtf((float)(image_bgr.cols * image_bgr.cols + image_bgr.rows * image_bgr.rows));
            vector<LineCandidate> hessian = computeHessianOnKernel(candidates, d_theta, d_rho, -diag, diag);                // 3차원   배경
            vector<LineCandidate> selected = selectByElbowKMeans(hessian);                                                  // 다밀집  지역
            vector<LineCandidate> avg_lines = clusterAndAverageLines(selected, d_theta, d_rho);                             // 목표    직선
            // CSV 파일
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
            Mat yolo_input = maskForYolo(image_bgr, avg_lines, OFFSET);                                                     // 영상
            vector<Detection> detections = inferOBB(model, yolo_input, 640, 0.001f, 0, device);                             // 단일
            vector<Detection> rebarBoxes = clusterToRebarBoxes(avg_lines, detections, OFFSET);                             // 철근
            vector<Point2f> intersections = computeIntersections(rebarBoxes, image_bgr.cols, image_bgr.rows);               // 교차
            savePixelCsv(intersections, image_bgr, "C:/image_640x360/1/intersection.csv");
            auto t_end3 = chrono::high_resolution_clock::now();
            printf("rebar time: %.2f ms\n", chrono::duration<float, milli>(t_end3 - t_start3).count());



            // (6) 영상 캡쳐 (기존과 동일)
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
                int x1 = (int)(det.cx - det.w / 2), y1 = (int)(det.cy - det.h / 2);
                int x2 = (int)(det.cx + det.w / 2), y2 = (int)(det.cy + det.h / 2);
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
