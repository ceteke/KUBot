import numpy as np
import rospy
from affordance_core import IterationError

def round_array(arr):
    for i in range(len(arr)):
        arr[i] = float('%.6f'%round(arr[i],6))

def pre_proc_features(feature_arr):
    preproc_features = []
    features_to_use = [1,2,3,17,19,20,21]

    for f in features_to_use:
        preproc_features.append(feature_arr[f])

    hist_data = feature_arr[27:335]
    hist_data_preproc = []
    hist_ranges = {45:3}
    hist_order = [45,45,45]
    crr_index = 0

    for ho in hist_order:
        step = hist_ranges[ho]
        preproc = [np.sum(hist_data[crr_index+i*step:crr_index+step*(i+1)]) for i in range(ho/step)]
        preproc_features += preproc
        crr_index += ho

    rospy.loginfo("Feature reduction %d => %d" % (len(feature_arr),len(preproc_features)))

    return preproc_features

def pc_features_to_array(pc_feats):
    transform = pc_feats.transform
    translation = transform.translation
    rotation = transform.rotation
    pc_centroid = pc_feats.points_centroid
    pc_min = pc_feats.points_min
    pc_max = pc_feats.points_max
    rgba_color = pc_feats.rgba_color
    bb_center = pc_feats.bb_center
    bb_dims = pc_feats.bb_dims
    pc_size = pc_feats.num_points #???

    bb_volume = bb_dims.x * bb_dims.y * bb_dims.z
    bb_area = 2*(bb_dims.x*bb_dims.y + bb_dims.x*bb_dims.z + bb_dims.y*bb_dims.z)
    if bb_dims.x == 0 or bb_volume == 0 or pc_size == 0:
        raise IterationError("Meaningless features..")
    bb_aspect_ratio = bb_dims.y / bb_dims.x
    bb_area_over_volume = bb_area / bb_volume
    compactness = bb_volume / pc_size

    result = [334, bb_center.x, bb_center.y, bb_center.z, pc_feats.bb_angle,
                pc_centroid.x, pc_centroid.y, pc_centroid.z, pc_min.x, pc_min.y,
                pc_min.z, pc_max.x, pc_max.y, pc_max.z, rgba_color.r, rgba_color.g,
                rgba_color.b, pc_feats.hue, pc_size, bb_dims.x, bb_dims.y, bb_dims.z, bb_volume,
                bb_area, bb_aspect_ratio, bb_area_over_volume, compactness]

    result = result + list(pc_feats.data)
    round_array(result)

    return result, pre_proc_features(result)
