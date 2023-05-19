# Read the calibration data from the yaml file and return the camera matrix and distortion coefficients

import yaml
import numpy as np

def get_calibration_data(calibration_file):
    with open(calibration_file, 'r') as stream:
        try:
            data = yaml.safe_load(stream)
            camera_matrix = np.array(data['camera_matrix']['data']).reshape((3, 3))
            distortion_coefficients = np.array(data['distortion_coefficients']['data'])
            return camera_matrix, distortion_coefficients
        except yaml.YAMLError as exc:
            print(exc)
            return None, None
        

        