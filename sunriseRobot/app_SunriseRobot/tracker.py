from ultralytics import YOLO

import args
import utils
import global_constants as gc


class YoloTracker(object):
    def __init__(self, **kwargs):
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'tracker_config.yaml',
            **kwargs,
        )
        self.model_name = parameters['model_name']
        self.model_folder = gc.APP_FOLDER_PATH + parameters['model_folder']
        self.model_path = self.model_folder + self.model_name
        self.image_size = parameters['image_size']
        # self.x_center = int(image_size[0] / 2)
        # self.y_center = int(image_size[1] / 2)
        self.verbose = parameters['verbose']
        if self.verbose:
            print(f'Using Computer Vision model: {self.model_name}')

        # Download model in folder if not present, and load it
        self.model = YOLO(model=self.model_path, verbose=self.verbose)

        self.target_class_name = None
        self.target_class_id : int = -1
        self.no_target_info = {
            'num_targets': 0,
            'highest_confidence': 0,
            'distance_from_center_x': 0,
            'distance_from_center_y': 0,
        }

    def select_target(self, target_name: str) -> None:
        if self.target_class_name != target_name:
            # target to track (only one allowed)
            try:
                self.target_class_id = utils.get_class_id_from_name(class_name=target_name, class_dict=self.model.names)
                self.target_class_name = target_name
                if self.verbose:
                    print(f'target: {target_name}')
                    # print(f'target_class_id: {self.target_class_id}')
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
        # x and y have range [-1, 1] where:
        #   [-1, 0] means the target is left or above of center respectively
        #   [0, 1] means the target is right or below of center respectively

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
            'distance_from_center_x': 0,
            'distance_from_center_y': 0,
        }
        try:
            confidence = results[0].boxes.conf
            print(f'num_targets: {len(confidence)}')
            if len(confidence) > 0:
                # print(f'confidence: {confidence}')
                max_confidence_id = confidence.argmax()
                # print(f'max_confidence_id: {max_confidence_id}')
                normalized_center_x, normalized_center_y, _, _ = results[0].boxes.xywhn[max_confidence_id]
                # print(f'Target Center X: {normalized_center_x}')
                target_info['num_targets'] = len(confidence)
                target_info['highest_confidence'] = confidence[max_confidence_id].item()
                target_info['distance_from_center_x'] = -(normalized_center_x - 0.5) * 2
                target_info['distance_from_center_y'] = (normalized_center_y - 0.5) * 2
                # print(f'X distance from img center: {target_info["distance_from_center_x"]}')
                return target_info
            else:
                return self.no_target_info
        except AttributeError:
            return self.no_target_info

        
        
