
import rospy
import tf2_ros as tf2
import numpy as np

from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped

class BoundingBoxPoseEstimator():

    def __init__(self, image_height, image_width, focal_length, camera_offsets, image_center_x, image_center_y):
        self.img_height = image_height
        self.img_width = image_width
        self.focal_length = focal_length
        self.camera_offsets = camera_offsets
        self.image_center_x = image_center_x
        self.image_center_y = image_center_y
        
        self.tfBuffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tfBuffer)

    def remove_bad_bounding_boxes(self, bounding_boxes):
        """
        Removes bounding boxes that are not relatively square.
        """
        ret = [bb for bb in bounding_boxes.bounding_boxes if self._is_good_bb(bb)]
        return ret

    def estimate_position_from_helipad_perimeter(self, bounding_boxes, perimeter_radius_mm=400, bb_scale_factor=1.0):
        best_bb = self._find_best_bb_of_class(bounding_boxes, "platform")

        if best_bb != None:
            center_px = self._est_center_of_bb(best_bb)
            radius_px = bb_scale_factor * self._est_radius_of_bb(best_bb)
            if not (all(center_px) and radius_px): # TODO: Check if these are ever run
                return None
        else:
            return None

        x_cam, y_cam, z_cam = self._pixel_to_camera_coordinates(center_px, radius_px, perimeter_radius_mm)
        x_helipad_mm, y_helipad_mm, z_helipad_mm = self._camera_to_drone_body_coordinates(x_cam, y_cam, z_cam)

        # Convert to meters
        x_helipad = x_helipad_mm / 1000
        y_helipad = y_helipad_mm / 1000
        z_helipad = z_helipad_mm / 1000

        return x_helipad, y_helipad, z_helipad


    def _pixel_to_camera_coordinates(self, center_px, radius_px, radius_mm):

        # Image center
        x_c = self.image_center_x
        y_c = self.image_center_y

        # Distance from image center to object center
        d_x = x_c - center_px[0]
        d_y = y_c - center_px[1]

        # Find altitude from similar triangles
        z_camera = self.focal_length * radius_mm / radius_px

        # Find x and y coordiantes using similar triangles as well. The signs are
        # used so that the x-coordinate is positive to the right and the y-coordinate
        # is positive upwards
        x_camera = -(z_camera * d_x / self.focal_length)
        y_camera = -(z_camera * d_y / self.focal_length)

        return x_camera, y_camera, z_camera

    def _camera_to_drone_body_coordinates(self, x_camera, y_camera, z_camera):

        point = PointStamped()
        point.header.frame_id = "camera"
        point.header.stamp = rospy.Time()

        point.point.x = x_camera
        point.point.y = y_camera
        point.point.z = z_camera

        bodypoint = self.tfBuffer.transform(point, "body", rospy.Duration(0.1))

        return bodypoint.point.x , bodypoint.point.y, bodypoint.point.z

    def _is_good_bb(self, bb):
        """
        Returns true for bounding boxes that are within the desired proportions,
        them being relatively square.
        Bbs that have one side being 5 times or longer than the other are discarded.

        input:
            bb: yolo bounding box

        output.
            discard or not: bool
        """
        bb_w = bb.xmax - bb.xmin
        bb_h = bb.ymax - bb.ymin
        if 0.2 > bb_w/bb_h > 5:
            return False
        else:
            return True

    def _est_radius_of_bb(self, bb):
        width = bb.xmax - bb.xmin
        height = bb.ymax - bb.ymin
        radius = (width + height)/4
        return radius

    def _est_center_of_bb(self, bb):
        width = bb.xmax - bb.xmin
        height = bb.ymax - bb.ymin
        center = [bb.xmin + width/2.0 ,bb.ymin + height/2.0]
        map(int, center)
        return center

    def _find_best_bb_of_class(self, bounding_boxes, classname):
        matches =  list(item for item in bounding_boxes if item.Class == classname)
        try:
            best = max(matches, key=lambda x: x.probability)
        except ValueError as e:
            best = None
        return best