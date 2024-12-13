import math
import numpy as np

def find_manIndex(jacobian):
    jac_transpose = jacobian.T
    determinant = np.linalg.det(jacobian*jac_transpose)
    manIndex = math.sqrt(abs(determinant))
    return manIndex

def calc_scaled_avg_manIndex(array):
    sum_array = sum(array)
    max_array = max(array)
    len_array = len(array)
    scaled_avg_manIndex = sum_array/(max_array*len_array)
    return scaled_avg_manIndex
