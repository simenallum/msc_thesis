import json
import rosbag
import rospy
import cv2
from datetime import datetime
import time
import numpy as np
from collections import defaultdict
from yolov8_ros.msg import BoundingBox, BoundingBoxes



from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix
from tqdm import tqdm

class_labels = ["human", "floating-object"]


def convert_to_ros_time(date_time_str):
    dt = datetime.strptime(date_time_str, '%Y-%m-%dT%H:%M:%S.%f')
    return rospy.Time.from_sec(dt.timestamp())

def write_to_bagfile(json_file, image_folder, video_name):
    with open(json_file) as f:
        data = json.load(f)

    images = [d for d in data["images"] if d["source"]["folder_name"].strip() == video_name]
    bridge = CvBridge()
    bag = rosbag.Bag(video_name + '.bag', 'w')

    # Create image dict
    imgToAnns = defaultdict(list)
    for ann in data['annotations']:
        imgToAnns[ann['image_id']].append(ann)

    start_time = time.time()
    with tqdm(images, desc='Processing Images', total=len(images)) as pbar:
        for idx, image in enumerate(pbar):
            try:
                # Load image
                img = cv2.imread(image_folder + image['file_name'].replace(".png", ".jpg"))
                img = cv2.resize(img, (1280, 720)) # Convert resolution to 720p
                h = image["height"]
                w = image["width"]

                # Convert to ROS message
                msg = bridge.cv2_to_imgmsg(img, "bgr8")
                msg.header.stamp = convert_to_ros_time(image['date_time'])
                bag.write('/anafi/image', msg, t=msg.header.stamp)

                boundingBoxes = BoundingBoxes()
                gt_boundingBoxes = BoundingBoxes()
                bboxes = []
                gt_bboxes = []
                # Make BBs message
                for ann in imgToAnns[image["id"]]:
                    box = np.array(ann['bbox'], dtype=np.float64)
                    box[0] = (box[0] / w) * 1280
                    box[1] = (box[1] / h) * 720
                    box[2] = (box[2] / w) * 1280
                    box[3] = (box[3] / h) * 720
                    box[2] = box[0] + box[2]
                    box[3] = box[1] + box[3]
                        
                    cls = 0 if ann['category_id'] == 1 or ann['category_id'] == 2 else 1

                    try:
                        track_id = -ann['track_id']
                    except:
                        track_id = 999
                    
                    boundingBox = BoundingBox()
                    boundingBox.probability = 0.8
                    boundingBox.xmin = int(box[0])
                    boundingBox.ymin = int(box[1])
                    boundingBox.xmax = int(box[2])
                    boundingBox.ymax = int(box[3])
                    boundingBox.id = cls
                    boundingBox.Class = class_labels[cls]

                    bboxes.append(boundingBox)

                    # Swap out class id for track id for the GT boxes
                    boundingBox.id = int(track_id)
                    gt_bboxes.append(boundingBox)
                
                boundingBoxes.bounding_boxes = bboxes
                boundingBoxes.frame = msg

                gt_boundingBoxes.bounding_boxes = gt_bboxes
                gt_boundingBoxes.frame = msg

                bag.write('/yolo/search/boxes', boundingBoxes, t=msg.header.stamp)
                bag.write('/track_GT', gt_boundingBoxes, t=msg.header.stamp)


                # Write GPS data
                gps_msg = NavSatFix()
                gps_msg.header.stamp = msg.header.stamp
                gps_msg.latitude = image['meta']['gps_latitude']
                gps_msg.longitude = image['meta']['gps_longitude']
                gps_msg.altitude = image['meta']['altitude']
                bag.write('/gps_data', gps_msg, t=msg.header.stamp)

                # Write gimbal_pitch
                pitch_msg = Float32()
                pitch_msg.data = image['meta']['gimbal_pitch']
                bag.write('/gimbal_pitch', pitch_msg, t=msg.header.stamp)

                # Write compass_heading
                heading_msg = Vector3Stamped()
                heading_msg.header.stamp = msg.header.stamp
                heading_msg.vector.z = np.deg2rad(image['meta']['compass_heading'])
                bag.write('/compass_heading', heading_msg, t=msg.header.stamp)

                # Write gimbal_heading
                gimbal_heading_msg = Float32()
                gimbal_heading_msg.data = image['meta']['gimbal_heading']
                bag.write('/gimbal_heading', gimbal_heading_msg, t=msg.header.stamp)

                # Write velocity
                velocity_msg = TwistStamped()
                velocity_msg.header.stamp = msg.header.stamp
                velocity_msg.twist.linear.x = image['meta']['xspeed']
                velocity_msg.twist.linear.y = image['meta']['yspeed']
                velocity_msg.twist.linear.z = image['meta']['zspeed']
                bag.write('/velocity', velocity_msg, t=msg.header.stamp)
            except Exception as e:
                print('Error: '+ str(e))

            # Update progress bar with estimated time left
            elapsed_time = time.time() - start_time
            time_left = (len(images) - idx) * (elapsed_time / (idx + 1))
            pbar.set_description(f"Processing Images: ETA {time_left:.2f}s")

    bag.close()

    return bag


def main():
    # Path to the json file
    json_file = "/home/msccomputer/Desktop/seadronesea_labels/instances_train_objects_in_water.json"

    write_to_bagfile(json_file, "/home/msccomputer/Desktop/SeaDroneSea_RAW/train/", "DJI_0065")

if __name__ == "__main__":
    main()