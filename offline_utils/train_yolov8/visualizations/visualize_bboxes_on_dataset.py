import cv2
import matplotlib.pyplot as plt
import os
import yaml
import random

def visualize_bboxes(images_path, labels_path, data_path, image_name):
    with open(data_path, 'r') as f:
        classes = yaml.load(f, Loader=yaml.FullLoader)
    
    # Generate random colors for each class
    colors = [tuple([random.randint(0, 255) for _ in range(3)]) for _ in range(len(classes['names']))]
    
    # Read the image
    img = cv2.imread(os.path.join(images_path, image_name))
    # Convert the image from BGR to RGB
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Read the corresponding label file
    label_file = os.path.splitext(image_name)[0] + ".txt"
    label_path = os.path.join(labels_path, label_file)
    with open(label_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            label, x, y, w, h = line.strip().split(' ')
            label = int(label)
            x, y, w, h = map(float, (x, y, w, h))
            xmin = int((x - w / 2) * img.shape[1])
            ymin = int((y - h / 2) * img.shape[0])
            xmax = int((x + w / 2) * img.shape[1])
            ymax = int((y + h / 2) * img.shape[0])
            class_name = classes['names'][label]
            color = colors[label]
            # Draw the bounding box on the image
            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), color, 2)
            # Put the class label above the bounding box
            cv2.putText(img, class_name, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    # Display the image
    plt.imshow(img)
    plt.show()


def main():
  images_path = "/home/msccomputer/Desktop/msc_dataset/test/images"
  labels_path = "/home/msccomputer/Desktop/msc_dataset/test/labels"
  data_path = "/home/msccomputer/Desktop/msc_dataset/data.yaml"
  image_name = "c_141_jpg.rf.7577d2bb2e3554d611dcf68b7987deef.jpg"

  visualize_bboxes(images_path, labels_path, data_path, image_name)

if __name__ == '__main__':
  main()