#!/usr/bin/env python3
"""
Simple test to check if GMSL cameras are accessible via V4L2
This bypasses ZED SDK entirely to verify hardware is working
"""
import cv2
import sys

def test_camera(device_id):
    """Test if a camera device can be opened and read"""
    print(f"\n[Testing /dev/video{device_id}]")

    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"  ✗ Failed to open /dev/video{device_id}")
        return False

    # Try to get camera properties
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"  ✓ Camera opened successfully")
    print(f"    Resolution: {int(width)}x{int(height)}")
    print(f"    FPS: {fps}")

    # Try to read a frame
    ret, frame = cap.read()

    if ret and frame is not None:
        print(f"  ✓ Frame captured: {frame.shape}")
        cv2.imwrite(f'/tmp/camera_{device_id}_test.jpg', frame)
        print(f"  ✓ Saved to /tmp/camera_{device_id}_test.jpg")
        cap.release()
        return True
    else:
        print(f"  ✗ Failed to read frame")
        cap.release()
        return False

def main():
    print("=" * 60)
    print("ZED GMSL Camera V4L2 Test")
    print("=" * 60)

    # Test video0, video2, video4, video6 (typically left cameras of each ZED)
    camera_devices = [0, 2, 4, 6]

    results = {}
    for device_id in camera_devices:
        results[device_id] = test_camera(device_id)

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    working_cameras = [dev for dev, status in results.items() if status]

    if working_cameras:
        print(f"✓ {len(working_cameras)} camera(s) working: /dev/video{working_cameras}")
        print("\nConclusion: Hardware is OK!")
        print("The problem is with ZED SDK not recognizing GMSL cameras.")
        print("\nSolution: Upgrade ZED SDK from 4.2.5 to 5.1")
        return 0
    else:
        print("✗ No cameras working")
        print("\nThis indicates a driver or hardware problem.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
