from enums.ObjectID import ObjectID
from vision.display_model_boxes import draw_detection

COLOR_IN_DETECTION = (0, 255, 0)
COLOR_NOT_IN_DETECTION = (255, 0, 0)

def slalom_organizer(slalom_tab, objects, annotated_frame, payload, type_yolo):
    if len(slalom_tab) == 0:
        return objects, payload

    selected_ids = {}

    if ObjectID.SLALOM_CENTER not in objects:
        valid_slaloms = [s for s in slalom_tab if s["depth"] != -1000]

        if valid_slaloms:
            closest_side = min(valid_slaloms, key=lambda s: s["depth"])
        else:
            closest_side = min(slalom_tab, key=lambda s: s["depth"])

        selected_ids[id(closest_side)] = ObjectID.SLALOM_SIDE

        payload.extend([
            float(ObjectID.SLALOM_SIDE),
            float(closest_side["dist_center"]),
            float(closest_side["depth"]),
            0.0
        ])

    else:
        slalom_middle_cx = objects[ObjectID.SLALOM_CENTER]["box_cx"]
        slalom_left = None
        slalom_right = None

        for slalom in slalom_tab:
            depth_value = slalom["depth"]
            box_cx = slalom["box_cx"]

            if box_cx < slalom_middle_cx:
                if slalom_left is None or depth_value < slalom_left["depth"]:
                    slalom_left = slalom

            elif box_cx > slalom_middle_cx:
                if slalom_right is None or depth_value < slalom_right["depth"]:
                    slalom_right = slalom

        if slalom_left is not None:
            objects[ObjectID.SLALOM_LEFT] = slalom_left
            selected_ids[id(slalom_left)] = ObjectID.SLALOM_LEFT

            payload.extend(
                [float(ObjectID.SLALOM_LEFT), float(slalom_left["dist_center"]), float(slalom_left["depth"]), 0.0])

        if slalom_right is not None:
            objects[ObjectID.SLALOM_RIGHT] = slalom_right
            selected_ids[id(slalom_right)] = ObjectID.SLALOM_RIGHT

            payload.extend(
                [float(ObjectID.SLALOM_RIGHT), float(slalom_right["dist_center"]), float(slalom_right["depth"]), 0.0])

    for slalom in slalom_tab:
        display_id = selected_ids.get(id(slalom), ObjectID.SLALOM_SIDE)
        color = COLOR_IN_DETECTION if id(slalom) in selected_ids else COLOR_NOT_IN_DETECTION

        draw_detection(
            annotated_frame,
            type_yolo,
            display_id,
            slalom["confidence"], slalom["depth"],
            slalom["dist_center"],
            slalom["box_cx"], slalom["box_cy"],
            slalom["x1"], slalom["y1"],
            slalom["x2"], slalom["y2"],
            color, slalom["points"])

    return objects, payload


def target_organizer(target_tab, objects, annotated_frame, payload, type_yolo):
    if len(target_tab) == 0:
        return objects, payload

    pictograms = []

    for object_id, obj in objects.items():
        if object_id in [
            ObjectID.FIRE_TRUCK,
            ObjectID.AMBULANCE,
            ObjectID.FIRE,
            ObjectID.BLOOD
        ]:
            pictograms.append({
                "id": object_id,
                "obj": obj,
                "box_cx": obj["box_cx"],
                "box_cy": obj["box_cy"]
            })

    picto_to_target_id = {
        ObjectID.FIRE_TRUCK: ObjectID.TARGET_TRUCK,
        ObjectID.AMBULANCE: ObjectID.TARGET_AMBULANCE,
        ObjectID.FIRE: ObjectID.TARGET_FIRE,
        ObjectID.BLOOD: ObjectID.TARGET_BLOOD
    }

    used_pictos = set()
    selected_targets = {}

    MAX_DIST_PX = 220

    for target in target_tab:
        best_picto_index = None
        best_dist = float("inf")

        for i, picto in enumerate(pictograms):
            if i in used_pictos:
                continue

            dx = target["box_cx"] - picto["box_cx"]
            dy = target["box_cy"] - picto["box_cy"]
            dist = (dx * dx + dy * dy) ** 0.5

            if dist < best_dist:
                best_dist = dist
                best_picto_index = i

        if best_picto_index is not None and best_dist < MAX_DIST_PX:
            picto_id = pictograms[best_picto_index]["id"]
            assigned_id = picto_to_target_id[picto_id]
            used_pictos.add(best_picto_index)
        else:
            assigned_id = ObjectID.TARGET

        selected_targets[id(target)] = assigned_id

        payload.extend([
            float(assigned_id),
            float(target["dist_center"]),
            float(target["depth"]),
            0.0
        ])

    for target in target_tab:
        display_id = selected_targets.get(id(target), ObjectID.TARGET)
        color = COLOR_IN_DETECTION if id(target) in selected_targets else COLOR_NOT_IN_DETECTION

        draw_detection(
            annotated_frame,
            type_yolo,
            display_id,
            target["confidence"],
            target["depth"],
            target["dist_center"],
            target["box_cx"],
            target["box_cy"],
            target["x1"],
            target["y1"],
            target["x2"],
            target["y2"],
            color=color,
            points=target["points"]
        )

    return objects, payload
