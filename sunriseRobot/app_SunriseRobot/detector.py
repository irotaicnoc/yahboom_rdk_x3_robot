import warnings

from ultralytics import YOLO

import args
import utils
import global_constants as gc


class YoloDetector(object):
    def __init__(self, **kwargs):
        parameters = args.import_args(
            yaml_path=gc.CONFIG_FOLDER_PATH + 'detector_config.yaml',
            **kwargs,
        )

        self.model_name = parameters['model_name']
        self.model_folder = gc.GENERIC_MODEL_FOLDER_PATH
        self.model_path = self.model_folder + self.model_name
        self.camera_image_size = parameters['camera_image_size']
        self.verbose = parameters['verbose']
        self.confidence_threshold = parameters['confidence_threshold']
        self.device = gc.CPU_DEVICE
        if gc.TPU_DEVICE in parameters['model_name']:
            self.device = gc.TPU_DEVICE

        if self.verbose >= 1:
            print(f'Using Computer Vision model: {self.model_name}')

        if self.device == gc.TPU_DEVICE:
            try:
                if self.verbose >= 1:
                    print('\ton edge TPU device.')
                self.model = YOLO(model=self.model_path, task='detect', verbose=self.verbose)
                self.model_class_dict = self.model.names

            except Exception as e:
                warnings.warn(f'could not initialize TPU model {self.model_name} in folder {self.model_folder}...')
                if self.verbose >= 1:
                    print(e)
                self.device = gc.CPU_DEVICE
                self.model_name = parameters['backup_model_name']
                self.model_folder = gc.GENERIC_MODEL_FOLDER_PATH
                self.model_path = self.model_folder + self.model_name
                if self.verbose >= 1:
                    print(f'Switching to backup CPU model {self.model_name} in folder {self.model_folder}...')

        if self.device == gc.CPU_DEVICE:
            # Download model in folder if not present, and load it
            self.model = YOLO(model=self.model_path, verbose=self.verbose)
            self.model_class_dict = self.model.names

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
            old_target_class_id = self.target_class_id
            old_target_class_name = self.target_class_name
            try:
                self.target_class_id = utils.get_class_id_from_name(
                    class_name=target_name,
                    class_dict=self.model_class_dict,
                )
                self.target_class_name = target_name
                if self.verbose >= 2:
                    print(f'target: {target_name}')
                    # print(f'target_class_id: {self.target_class_id}')
            except ValueError as e:
                warnings.warn('Error in selecting new target.')
                print(e)
                print(f'Restoring previous target "{old_target_class_name}"')
                self.target_class_id = old_target_class_id
                self.target_class_name = old_target_class_name

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

        # frame = utils.format_camera_frames(
        #     frame=frame,
        #     original_width=self.camera_image_size[0],
        #     original_height=self.camera_image_size[1],
        # )

        if self.target_class_name is None:
            warnings.warn(f'Target_class_name is None.')
            return self.no_target_info

        results = self.model.predict(
            source=0,
            imgsz=self.camera_image_size,
            # vid_stride=10,
            project=gc.OUTPUT_FOLDER_PATH,
            save=save,
            conf=self.confidence_threshold,
            # stream=True,
            show=False,
            stream_buffer=False,
            # persist=True,
            classes=[self.target_class_id],
            verbose=False,
        )
        try:
            confidence = results[0].boxes.conf
            if len(confidence) > 0:
                target_info = {
                    'num_targets': 0,
                    'highest_confidence': 0,
                    'distance_from_center_x': 0,
                    'distance_from_center_y': 0,
                }
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
        except:
            # print(f'Detection Exception: {e}')
            return self.no_target_info
