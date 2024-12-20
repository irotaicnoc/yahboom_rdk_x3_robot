import cv2
from pathlib import Path
from ultralytics import YOLO

import utils
import global_constants as gc


def test_magic_detector() -> None:
    # magic detector model to test
    # trained_model_path = f'{gc.MAGIC_MODEL_FOLDER_PATH}magic_detector_yolo11l.pt'
    trained_model_path = f'{gc.MAGIC_MODEL_FOLDER_PATH}magic_detector_yolo11x.pt'
    # trained_model_path = f'{gc.MAGIC_MODEL_FOLDER_PATH}magic_detector_yolo11l_640_480_edgetpu.tflite'
    trained_model = YOLO(model=trained_model_path, task='detect')

    test_image_path_generator = Path(f'{gc.APP_FOLDER_PATH}test_images/').glob('*.*')
    for test_image_path in test_image_path_generator:
        str_path = str(test_image_path)
        print(f'image path: {str_path}')
        test_image = cv2.imread(filename=str_path)
        # utils.display_image(image=test_image, window_name='Test image')
        # cv2.resize(src=test_image, dsize=(640, 640), dst=test_image)
        print(f'Test image shape: {test_image.shape}')
        # exchange the height and width
        # test_image = test_image.transpose(1, 0, 2)
        # test_image = test_image.transpose(2, 0, 1)
        # print(f'Test image shape: {test_image.shape}')

        results = trained_model(test_image)
        image_np = results[0].plot()
        utils.display_image(image=image_np, window_name='Detection result')


if __name__ == '__main__':
    test_magic_detector()
