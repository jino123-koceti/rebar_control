// ★ frame_client.hpp 를 "가장 먼저" 포함 -> winsock2.h 를 다른 windows.h 포함보다 앞세워 재정의 충돌 방지.
#include "frame_client.hpp"
#include "1.hpp"
#include <filesystem>          // 교차점 크롭 저장 폴더 생성
#include <ctime>               // 캡쳐 시각(초 단위) 폴더명 생성
using namespace std;
using namespace cv;
static const char* ROBOT_IP = "100.99.144.107";   // 로봇 Jetson IP (또는 Tailscale IP)
static const int   ROBOT_PORT = 5001;             // frame_stream_server.py 포트
// ── ROI 설정 ────────────────────────────────────────────────────
static const int ROI_WIDTH = 500;
static const int ROI_HEIGHT = 600;
static const int ROI_OFFSET_Y = 80;

int main()
{
    FrameClient cam;
    if (!cam.connect(ROBOT_IP, ROBOT_PORT))
    {
        cerr << "로봇 프레임 서버 접속 실패: " << ROBOT_IP << ":" << ROBOT_PORT << endl;
        return 1;
    }
    cout << "로봇 프레임 서버 연결됨: " << ROBOT_IP << ":" << ROBOT_PORT << endl;
    std::filesystem::create_directories("C:/image_640x360/1"); // 상위 폴더 보장
    float fx = 0, fy = 0, cx = 0, cy = 0;
    Mat depth_f32, image_bgr;
    FrameClient::Intr K;
    while (true)
    {
        if (!cam.grab(image_bgr, depth_f32, K))
        {
            if (!cam.connected())
            {
                cerr << "연결 끊김 -> 재연결 시도..." << endl;
                if (cam.connect(ROBOT_IP, ROBOT_PORT)) cout << "재연결 성공" << endl;
                else { cerr << "재연결 실패, 잠시 후 재시도" << endl; waitKey(300); }
                continue;
            }
            waitKey(5);
            continue;
        }
        fx = K.fx; fy = K.fy; cx = K.cx; cy = K.cy;
        // grab 성공할 때마다 카운트, 1초에 한 번만 출력 (실시간으로 새 프레임이 계속 오는지 확인용)
        static int frameCount = 0;
        static auto lastPrint = chrono::steady_clock::now();
        frameCount++;
        auto nowTp = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(nowTp - lastPrint).count() >= 1000)
        {
            cout << "[grab] 누적 성공 프레임 수: " << frameCount << endl;
            lastPrint = nowTp;
        }



        // ── 매 프레임 고정 ROI로 실시간 깊이맵 시각화 ────────────────────────────────────────────
        Rect roi(image_bgr.cols / 2 - ROI_WIDTH / 2, image_bgr.rows / 2 - ROI_HEIGHT / 2 + ROI_OFFSET_Y, ROI_WIDTH, ROI_HEIGHT);
        roi = roi & Rect(0, 0, image_bgr.cols, image_bgr.rows);
        Mat depth_visual = visualizeDepthMap(depth_f32, DEPTH_MIN, DEPTH_MAX, COLORMAP_JET);
        Mat depth_view = depth_visual.clone();
        rectangle(depth_view, roi, Scalar(0, 255, 255), 2);
        imshow("Depth Map", depth_view);
        int key = waitKey(1) & 0xFF; // ★ Windows에서 상위 비트 잡음 섞이는 것 방지



        if (key == ' ')
        {
            // ★ 캡쳐 시각(초 단위, YYMMDD_HHMMSS)으로 폴더명 생성 -> 재실행해도 항상 고유, 이전 캡쳐 덮어쓰지 않음
            time_t t = time(nullptr);
            tm tmNow;
            localtime_s(&tmNow, &t);
            char dirBuf[64];
            strftime(dirBuf, sizeof(dirBuf), "C:/image_640x360/1/%y%m%d_%H%M%S/", &tmNow);
            string outDir = dirBuf;
            std::filesystem::create_directories(outDir + "cross");

            // ★파이프라인 전체를 ROI 안에서만 실행 (★fx,fy(초점거리)는 ROI와 무관하게 그대로 두고, cx,cy(주점)만 ROI 좌상단만큼 평행이동)
            Rect cropRect = roi;
            if (cropRect.width <= 0 || cropRect.height <= 0)
            {
                cerr << "ROI 크기가 잘못됐습니다. ROI_WIDTH/ROI_HEIGHT를 확인하세요." << endl;
                continue;
            }
            Mat proc_bgr = image_bgr(cropRect).clone();
            Mat proc_depth = depth_f32(cropRect).clone();
            Mat proc_depth_visual = visualizeDepthMap(proc_depth, DEPTH_MIN, DEPTH_MAX, COLORMAP_JET);
            float pcx = cx - cropRect.x, pcy = cy - cropRect.y; // ★주점은 빼어서 ROI를 기존의 화면의 원점에 갖다 놓는 것!



            auto t_start1 = chrono::high_resolution_clock::now();
            // (3) 층 분류 + 평면 추출
            PlaneResult cloud = depthToPointCloud(proc_depth, fx, fy, pcx, pcy, DEPTH_MIN, DEPTH_MAX);
            vector<PlanePoint> hough_pts = sampleHoughPlane(cloud, ITERATIONS1);                                            // 3차원   배경
            hough_pts = clusterHoughPlane(hough_pts);                                                                       // ★바닥  층 군집(재갱신)
            PlaneResult plane = extractPlaneByRank(hough_pts);                                                              // 바닥    층
            // ★ computeDistances 는 내부에서 cloud.pts 를 셔플해 순서대로 소비하므로, ROI로 영역이 좁아지면 포인트 수가 줄어드니 여기서 상한을 걸어준다.
            int iters2 = min<int>(ITERATIONS2, (int)cloud.pts.size());
            if (cloud.pts.empty())
            {
                cerr << "ROI 안에 유효한 뎁스 포인트가 없습니다. 다른 영역을 지정하세요." << endl;
                continue;
            }
            vector<PlanePoint> layer_pts = computeDistances(cloud, plane, iters2);                                          // 1차원   배경
            layer_pts = clusterLayers(layer_pts);                                                                           // ★목표  층 군집(재갱신)
            PlaneResult layer_plane = extractLayerByRank(layer_pts, LAYER_RANK);                                            // 목표    층
            // CSV 파일 (바닥점 + 목표점 + 3차원점 + 1차원점)
            savePlaneResultCsv(plane, proc_bgr, fx, fy, pcx, pcy, outDir + "plane_result.csv");
            savePlaneResultCsv(layer_plane, proc_bgr, fx, fy, pcx, pcy, outDir + "layer_result.csv");
            saveHoughPlaneCsv(hough_pts, outDir + "hough_plane.csv");
            saveLayerCSV(layer_pts, outDir + "layer_pts.csv");
            auto t_end1 = chrono::high_resolution_clock::now();
            printf("plane time: %.2f ms\n", chrono::duration<float, milli>(t_end1 - t_start1).count());



            auto t_start2 = chrono::high_resolution_clock::now();
            // (4) 선 추출 + 교차점 산출
            vector<Point2f> pixels = projectToImage(layer_plane.pts, fx, fy, pcx, pcy);
            vector<LineCandidate> candidates = sampleHoughLine(pixels, ITERATIONS3);                                        // 2차원   배경
            vector<LineCandidate> knn_result = estimateWindowSize(candidates);                                              // ★최밀집크기
            float d_theta = knn_result[0].d_theta; float d_rho = knn_result[0].d_rho; float diag = sqrtf((float)(proc_bgr.cols * proc_bgr.cols + proc_bgr.rows * proc_bgr.rows));
            vector<LineCandidate> hessian = computeHessianOnKernel(candidates, d_theta, d_rho, -diag, diag);                // ★3차원 배경
            vector<LineCandidate> selected = selectByElbowKMeans(hessian);                                                  // 고밀집  직선
            vector<LineCandidate> avg_lines = clusterAndAverageLines(selected, d_theta, d_rho);                             // 목표    직선
            vector<Point2f> interPts = computeLineIntersections(avg_lines, proc_bgr.cols, proc_bgr.rows);
            // CSV 파일 (직선점 + 허프점 + 최밀집 + 고밀집 + kmc + dbs + 교차점)
            savePixelCsv(pixels, proc_bgr, outDir + "pixels.csv");
            saveLineCsv(candidates, outDir + "hough_line.csv");
            saveLineCsv(knn_result, outDir + "knn_result.csv");
            saveLineCsv(hessian, outDir + "hessian.csv");
            saveLineCsv(selected, outDir + "selected.csv");
            saveLineCsv(avg_lines, outDir + "line_result.csv");
            savePixelCsv(interPts, proc_bgr, outDir + "intersection.csv");
            auto t_end2 = chrono::high_resolution_clock::now();
            printf("line time: %.2f ms\n", chrono::duration<float, milli>(t_end2 - t_start2).count());



            // (5) 영상 캡쳐
            imwrite(outDir + "1_original.png", proc_bgr);
            imwrite(outDir + "2_depth.png", proc_depth_visual);
            Mat result_img = Mat::zeros(proc_bgr.size(), proc_bgr.type());
            for (const Point2f& px : pixels)
            {
                int u = cvRound(px.x); int v = cvRound(px.y);
                if (u < 0 || u >= result_img.cols || v < 0 || v >= result_img.rows) continue;
                result_img.at<Vec3b>(v, u) = proc_bgr.at<Vec3b>(v, u);
            }
            imwrite(outDir + "3_result.png", result_img);
            Mat vis_lines = proc_bgr.clone();
            for (const auto& lc : avg_lines)
            {
                float cos_t = cosf(lc.theta), sin_t = sinf(lc.theta);
                // x*cos_t + y*sin_t + rho = 0 위의 한 점(원점에서 가장 가까운 점) + 직선 방향으로 diag만큼 연장
                float x0 = -lc.rho * cos_t, y0 = -lc.rho * sin_t;
                Point2f p1(x0 - diag * sin_t, y0 + diag * cos_t);
                Point2f p2(x0 + diag * sin_t, y0 - diag * cos_t);
                line(vis_lines, p1, p2, Scalar(255, 255, 255), 2, LINE_AA);
            }
            imwrite(outDir + "4_lines.png", vis_lines);
            Mat vis_cross = proc_bgr.clone();
            int half = ROI / 2;
            for (size_t i = 0; i < interPts.size(); ++i)
            {
                const Point2f& pt = interPts[i];
                Rect wantBox(cvRound(pt.x) - half, cvRound(pt.y) - half, ROI, ROI);     // 원하는 크기(화면 밖으로 나갈 수 있음)
                Rect box = wantBox & Rect(0, 0, proc_bgr.cols, proc_bgr.rows);          // 실제로 잘라올 수 있는 영역
                if (box.width <= 0 || box.height <= 0) continue;
                rectangle(vis_cross, wantBox, Scalar(0, 255, 0), 2);                    // 초록 ROI 상자(의도한 크기 기준)
                circle(vis_cross, pt, 4, Scalar(0, 0, 255), -1);                        // 빨간 교차점
                // ★ 가장자리라 wantBox가 잘렸으면, 잘린 방향(좌/우/상/하)만큼 114 회색 패딩을 넣어 항상 ROI x ROI로 맞춤
                int padLeft = box.x - wantBox.x;
                int padTop = box.y - wantBox.y;
                int padRight = (wantBox.x + wantBox.width) - (box.x + box.width);
                int padBottom = (wantBox.y + wantBox.height) - (box.y + box.height);
                Mat crop = proc_bgr(box).clone();
                if (padLeft > 0 || padTop > 0 || padRight > 0 || padBottom > 0) copyMakeBorder(crop, crop, padTop, padBottom, padLeft, padRight, BORDER_CONSTANT, Scalar(114, 114, 114));
                char buf[256];
                snprintf(buf, sizeof(buf), "%scross/%03d.png", outDir.c_str(), (int)i);
                imwrite(buf, crop);                                                     // 교차점 하나당 개별 영상 (분류기 입력용, 항상 ROI x ROI)
            }
            imwrite(outDir + "5_intersections.png", vis_cross);
            cout << "[캡쳐] 저장 완료 -> " << outDir << endl;
        }
        if (key == 27) break; // ESC → 종료
    }
    cam.disconnect();
    return 0;
}
