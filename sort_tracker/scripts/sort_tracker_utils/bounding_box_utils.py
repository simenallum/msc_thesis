import numpy as np
import cv2

def convert_bboxes(bboxes):
    result = []
    for bbox in bboxes:
        xmin = bbox.xmin
        ymin = bbox.ymin
        xmax = bbox.xmax
        ymax = bbox.ymax
        result.append([xmin, ymin, xmax, ymax, bbox.probability, bbox.id])
    return np.array(result)

def draw_detections(image, tracks, mask_alpha=0.3):
    boxes = []
    scores = []
    class_id = []
    track_ids = []

    for i in range(len(tracks)):
        boxes.append([int(tracks[i][0]), int(tracks[i][1]), int(tracks[i][2]), int(tracks[i][3])])
        track_ids.append(tracks[i][4])
        class_id.append(tracks[i][5])
        scores.append(tracks[i][6])

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
    for box, score, track_id, class_id in zip(boxes, scores, track_ids, class_id):
        color = color_map[track_id]
        color = tuple(color.tolist())
                
        x1, y1, x2, y2 = box

        # Draw rectangle
        cv2.rectangle(det_img, (x1, y1), (x2, y2), color, 2)

        # Draw fill rectangle in mask image
        cv2.rectangle(mask_img, (x1, y1), (x2, y2), color, -1)

        label = f'Class ID: {class_id}, Track: {track_id}'
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