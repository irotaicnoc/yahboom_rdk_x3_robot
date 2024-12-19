import cv2
from pathlib import Path
from ultralytics import YOLO

import args
import utils
import global_constants as gc


def test(**kwargs) -> None:
    parameters = args.import_args(yaml_path='configs/fine_tune_yolo_model.yaml', **kwargs)

    # model to test
    trained_model_path = f'{gc.MAGIC_MODEL_FOLDER_PATH}magic_detector_yolo11x_640_480_edgetpu.tflite'
    trained_model = YOLO(model=trained_model_path)

    test_image_path_generator = Path(f'{gc.APP_FOLDER_PATH}test_images/').glob('*.jpg')
    for test_image_path in test_image_path_generator:
        test_image = cv2.imread(test_image_path)
        test_height, test_width = test_image.shape[:2]
        print(f'Test image (height, width): ({test_height}, {test_width})')
        if parameters['verbose'] >= 1:
            print(f'Using bg {test_image_path}')

        results = trained_model(test_image)
        image_np = results[0].plot()
        utils.display_image(image=image_np, window_name='REAL test images')


if __name__ == '__main__':
    test()
