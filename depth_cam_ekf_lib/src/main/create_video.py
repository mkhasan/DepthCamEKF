
import cv2
import sys
import os

def main():

    # RTSP URL
    # Example: rtsp://username:password@192.168.1.100:554/stream
    rtsp_url = "rtsp://root:root@192.168.2.208:554/cam0_0"

    if len(sys.argv) > 1:
        rtsp_url = sys.argv[1]

    # Configure FFmpeg to use TCP transport
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

    # Open RTSP stream
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Failed to open RTSP stream!")
        return

    # Get stream properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fps = cap.get(cv2.CAP_PROP_FPS)

    if fps <= 0 or fps > 120:
        fps = 25.0

    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps}")

    # Create AVI video writer
    output_file = "output.avi"

    fourcc = cv2.VideoWriter_fourcc(*'MJPG')

    writer = cv2.VideoWriter(
        output_file,
        fourcc,
        fps,
        (width, height),
        True
    )

    if not writer.isOpened():
        print("Failed to create AVI file!")
        cap.release()
        return

    print("Recording started. Press ESC to stop.")

    while True:

        # Capture frame from RTSP
        ret, frame = cap.read()

        if not ret:
            print("Failed to receive frame!")
            break

        if frame is None or frame.size == 0:
            continue

        # Write frame into AVI
        writer.write(frame)

        # Display captured frame
        # cv2.imshow("RTSP Recording", frame)
        #
        # if cv2.waitKey(1) == 27:  # ESC
        #     break

    # Release resources
    writer.release()
    cap.release()

    cv2.destroyAllWindows()

    print(f"Recording saved to {output_file}")


if __name__ == "__main__":
    main()
