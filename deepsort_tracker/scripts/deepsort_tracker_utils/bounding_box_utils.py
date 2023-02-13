import numpy as np
import cv2

def convert_bboxes(bboxes):
    result = []
    for bbox in bboxes:
        left = bbox.xmin
        top = bbox.ymin
        width = bbox.xmax - bbox.xmin
        height = bbox.ymax - bbox.ymin
        result.append(([left, top, width, height], bbox.probability, bbox.Class))
    return result

def draw_detections(image, tracks, mask_alpha=0.3):
    boxes = []
    scores = []
    track_ids = []
    class_names = []

    for i in range(len(tracks)):
        boxes.append(tracks[i][0])
        scores.append(tracks[i][1])
        track_ids.append(tracks[i][2])
        class_names.append(tracks[i][3])

    mask_img = image.copy()
    det_img = image.copy()

    # Create a list of colors for each track_id where each color is a tuple of 3 integer values
    rng = np.random.default_rng(3)
    colors = rng.uniform(0, 255, size=(len(set(track_ids)), 3)).astype(int)

    # Map track_ids to colors
    color_map = {track_id: color for track_id, color in zip(sorted(set(track_ids)), colors)}

    img_height, img_width = image.shape[:2]
    size = min([img_height, img_width]) * 0.0006
    text_thickness = int(min([img_height, img_width]) * 0.001)

    # Draw bounding boxes and labels of detections
    for box, score, track_id, class_name in zip(boxes, scores, track_ids, class_names):
        color = color_map[track_id]
        color = tuple(color.tolist())
                
        x1, y1, x2, y2 = box.astype(int)

        # Draw rectangle
        cv2.rectangle(det_img, (x1, y1), (x2, y2), color, 2)

        # Draw fill rectangle in mask image
        cv2.rectangle(mask_img, (x1, y1), (x2, y2), color, -1)

        label = f'Class ID: {class_name}, Track: {track_id}'
        caption = f'{label} {int(score * 100)}%'
        (tw, th), _ = cv2.getTextSize(text=caption, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                      fontScale=size, thickness=text_thickness)
        th = int(th * 1.2)

        cv2.rectangle(det_img, (x1, y1),
                      (x1 + tw, y1 - th), color, -1)
        cv2.rectangle(mask_img, (x1, y1),
                      (x1 + tw, y1 - th), color, -1)
        cv2.putText(det_img, caption, (x1, y1),
                    cv2.FONT_HERSHEY_SIMPLEX, size, (255, 255, 255), text_thickness, cv2.LINE_AA)

        cv2.putText(mask_img, caption, (x1, y1),
                    cv2.FONT_HERSHEY_SIMPLEX, size, (255, 255, 255), text_thickness, cv2.LINE_AA)

    return cv2.addWeighted(mask_img, mask_alpha, det_img, 1 - mask_alpha, 0)