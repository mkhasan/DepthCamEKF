
import cv2
import os
import sys

def video_to_pgm(video_file, output_dir):

    # Create output directory if it does not exist
    os.makedirs(output_dir, exist_ok=True)

    # Open video file
    cap = cv2.VideoCapture(video_file)

    if not cap.isOpened():
        print("Error: Cannot open video:", video_file)
        return

    frame_index = 0

    while True:

        # Read next frame
        ret, frame = cap.read()

        if not ret:
            break

        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Generate output filename
        filename = os.path.join(
            output_dir,
            f"frame_{frame_index:06d}.pgm"
        )

        # Save grayscale frame as PGM
        if not cv2.imwrite(filename, gray_frame):
            print("Error saving:", filename)
            break

        print("Saved:", filename)

        frame_index += 1

    cap.release()

    print("Finished. Total frames saved:", frame_index)


if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python3 video_to_pgm.py <video_file> <output_directory>")
        sys.exit(1)

    video_file = sys.argv[1]
    output_dir = sys.argv[2]

    video_to_pgm(video_file, output_dir)
