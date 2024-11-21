from ultralytics import YOLO

import utils


class YoloTracker(object):
    def __init__(self, model_name: str, model_folder: str, image_size: tuple, verbose: bool = False):
        self.model_name = model_name
        self.model_folder = model_folder
        self.model_path = model_folder + model_name
        self.image_size = image_size
        # self.x_center = int(image_size[0] / 2)
        # self.y_center = int(image_size[1] / 2)
        self.verbose = verbose

        # Download model in folder if not present, and load it
        self.model = YOLO(model=self.model_path, verbose=verbose)

        self.target_class_name = None
        self.target_class_id : int = -1
        self.no_target_info = {
            'num_targets': 0,
            'highest_confidence': 0,
            'normalized_center_x': 0,
            'normalized_center_y': 0,
        }

    def select_target(self, target_name: str) -> None:
        if self.target_class_name != target_name:
            # target to track (only one allowed)
            try:
                self.target_class_id = utils.get_class_id_from_name(class_name=target_name, class_dict=self.model.names)
                self.target_class_name = target_name
                if self.verbose:
                    print(f'target: {target_name}')
                    print(f'target_class_id: {self.target_class_id}')
            except ValueError as e:
                print('Error in selecting new target.')
                print(e)

    def find_target(self, frame, target_name: str, save: bool = False) -> dict:
        self.select_target(target_name)
        # output:
        #   - number of detections
        #   - confidence of the best detection
        #   - x distance from the center of the image (proportional to the width of the image)
        #   - y distance from the center of the image (proportional to the height of the image)
        # x and y have range [-0.5, 0.5] where:
        #   [-0.5, 0] means the target is left or above of center respectively
        #   [0, 0.5] means the target is right or below of center respectively

        if self.target_class_name is None:
            if self.verbose:
                print(f'target_class_name is None.')
            return self.no_target_info

        results = self.model.predict(
            source=frame,
            imgsz=self.image_size,
            # vid_stride=10,
            save=save,
            # stream=True,
            show=False,
            stream_buffer=False,
            # persist=True,
            classes=[self.target_class_id],
            verbose=False,
        )

        target_info = {
            'num_targets': 0,
            'highest_confidence': 0,
            'normalized_center_x': 0,
            'normalized_center_y': 0,
        }
        try:
            confidence = results[0].boxes.conf
            if len(confidence) > 0:
                # print(f'confidence: {confidence}')
                max_confidence_id = confidence.argmax()
                # print(f'max_confidence_id: {max_confidence_id}')
                normalized_center_x, normalized_center_y, _, _ = results[0].boxes.xywhn[max_confidence_id]
                target_info['num_targets'] = len(confidence)
                target_info['highest_confidence'] = confidence[max_confidence_id].item()
                target_info['normalized_center_x'] = normalized_center_x - 0.5
                target_info['normalized_center_y'] = normalized_center_y - 0.5
                return target_info
            else:
                return self.no_target_info
        except AttributeError:
            return self.no_target_info

        
        
