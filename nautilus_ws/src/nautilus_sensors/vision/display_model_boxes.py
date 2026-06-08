import cv2 
import numpy as np
   
def draw_detection(annotated_frame, model_type, object_id, confidence, depth_value,
                   dist_center, box_cx, box_cy, x1, y1, x2, y2,color, points=None):
        if model_type == 'obb' and points is not None:
            cv2.polylines(annotated_frame, [points], True, color, 2)
        elif model_type == 'bbox':
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
        else:
             raise ValueError("Invalid argument passed to draw_detections, accepted model types are obb or bbox")
        
        label = f"{object_id} | {confidence:.2f}"

        cv2.putText(annotated_frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(annotated_frame, f"{depth_value:.2f}mm", (box_cx, box_cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(annotated_frame, f"{dist_center:.2f}px", (box_cx, box_cy + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

def obb_model_coordinates(box):
    xywhr = box.xywhr[0].cpu().numpy()
    box_cx = int(xywhr[0])
    box_cy = int(xywhr[1])

    # ----------- CORNERS -----------
    points = box.xyxyxyxy[0].cpu().numpy().astype(int)

    x_coords = points[:, 0]
    y_coords = points[:, 1]

    x1, x2 = x_coords.min(), x_coords.max()
    y1, y2 = y_coords.min(), y_coords.max()

    # ----------- DEPTH ZONE -----------
    depth_zone_h = abs(y2 - y1)
    depth_zone_w = abs(x2 - x1)

    if depth_zone_w > depth_zone_h:
        half = 0.40 * depth_zone_h
    else:
        half = 0.40 * depth_zone_w

    half = max(1, min(8, int(np.ceil(half))))

    return box_cx, box_cy, x1, y1, x2, y2, half, points

def bbox_model_coordinates(box):
    xywh = box.xywh[0].cpu().numpy()
    box_cx = int(xywh[0])
    box_cy = int(xywh[1])

    x1, y1, x2, y2 = map(int, box.xyxy[0])

    half = 1

    return box_cx, box_cy, x1, y1, x2, y2, half