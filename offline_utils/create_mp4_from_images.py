import sys
import glob
import cv2


def make_video(image_folder, video_name, fps=30):
    image_list = sorted(glob.glob(image_folder + '/*.jpg'))
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    height, width, channels = cv2.imread(image_list[0]).shape
    video = cv2.VideoWriter(video_name, fourcc, fps, (width, height))

    for image in image_list:
        frame = cv2.imread(image)
        video.write(frame)

    cv2.destroyAllWindows()
    video.release()

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python script.py <image_folder> <video_name>")
        sys.exit(1)
    image_folder = sys.argv[1]
    video_name = sys.argv[2]
    make_video(image_folder, video_name)
