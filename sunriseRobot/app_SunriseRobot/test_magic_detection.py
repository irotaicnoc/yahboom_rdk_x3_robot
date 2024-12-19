import cv2
from pathlib import Path
from ultralytics import YOLO

import utils
import global_constants as gc


def test_magic_detector() -> None:
    # magic detector model to test
    trained_model_path = f'{gc.MAGIC_MODEL_FOLDER_PATH}magic_detector_yolo11x_640_480_edgetpu.tflite'
    trained_model = YOLO(model=trained_model_path, task='detect')

    test_image_path_generator = Path(f'{gc.APP_FOLDER_PATH}test_images/').glob('*.jpg')
    for test_image_path in test_image_path_generator:
        str_path = str(test_image_path)
        print(f'image path: {str_path}')
        test_image = cv2.imread(filename=str_path)
        # cv2.resize(src=test_image, dsize=(640, 640), dst=test_image)
        test_height, test_width = test_image.shape[:2]
        print(f'Test image (height, width): ({test_height}, {test_width})')
        # exchange the height and width
        test_image = test_image.transpose(1, 0, 2)

        results = trained_model(test_image)
        image_np = results[0].plot()
        utils.display_image(image=image_np, window_name='REAL test images')


if __name__ == '__main__':
    test_magic_detector()
