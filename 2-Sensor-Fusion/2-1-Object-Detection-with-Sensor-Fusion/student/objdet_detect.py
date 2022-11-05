# ------------------------------------------------------------------------------
# Project "3D Object Detection with LiDAR Data"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Detect 3D objects in point clouds using deep learning.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

### General package imports
import easydict
import numpy as np
import torch

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Model-related imports
# The DarkNet model (Complex YOLO)
from tools.objdet_models.darknet.models.darknet2pytorch import Darknet as darknet
from tools.objdet_models.darknet.utils.evaluation_utils import post_processing_v2
# The FPN ResNet model (SFA3D)
from tools.objdet_models.resnet.models import fpn_resnet
from tools.objdet_models.resnet.utils.evaluation_utils import decode
from tools.objdet_models.resnet.utils.evaluation_utils import post_processing
from tools.objdet_models.resnet.utils.torch_utils import _sigmoid


### Load model-related parameters into an `EasyDict` (ID_S3_EX1)
def load_configs_model(
        model_name: str='darknet', configs: easydict.EasyDict=None
) -> easydict.EasyDict:
    """Loads the model configurations into an `EasyDict` instance.

    :param model_name: the desired pre-trained model to load,
        can be one of: ['darknet', 'fpn_resnet'].
    :param configs: the EasyDict instance to update.
    :returns: configs, the updated `EasyDict` instance containing the
        model parameters to use.
    """

    ### Step 0 : Initialising the path and `EasyDict` variables
    # Instantiate a new config dict if none has been provided
    if not configs:
        configs = easydict.EasyDict()  
    # Set the parent directory of this file to enable relative paths
    curr_path = os.path.dirname(os.path.realpath(__file__))
    parent_path = configs.model_path = os.path.abspath(
        os.path.join(curr_path, os.pardir)
    )    
    ### Step 1 : Update parameters according to model type
    # If using the DarkNet model, update with relevant parameters
    if model_name == "darknet":
        configs.model_path = os.path.join(
            parent_path, 'tools', 'objdet_models', 'darknet'
        )
        configs.pretrained_filename = os.path.join(
            configs.model_path, 'pretrained', 'complex_yolov4_mse_loss.pth'
        )
        configs.arch = 'darknet'
        configs.batch_size = 4
        configs.cfgfile = os.path.join(
            configs.model_path, 'config', 'complex_yolov4.cfg'
        )
        configs.conf_thresh = 0.5
        configs.distributed = False
        configs.img_size = 608
        configs.nms_thresh = 0.4
        configs.num_samples = None
        configs.num_workers = 4
        configs.pin_memory = True
        configs.use_giou_loss = False
    # If using the FPN ResNet / SFA3D model, update with relevant parameters
    elif model_name == 'fpn_resnet':
        ####### ID_S3_EX1-3 START #######     
        print("student task ID_S3_EX1-3")
        # Path to the pre-trained model weights
        configs.model_path = os.path.join(
            parent_path, 'tools', 'objdet_models', 'resnet'
        )
        configs.pretrained_filename = os.path.join(
            configs.model_path,'pretrained', 'fpn_resnet_18_epoch_300.pth'
        )
        ### Step 2 : Adding and updating configs defined in `sfa/test.py`
        # The folder name to use for saving logs, trained models, etc.
        configs.saved_fn = 'fpn_resnet'    # Equivalent to 'fpn_resnet_18'
        # The name of the model architecture
        configs.arch = 'fpn_resnet'
        # The path to the pre-trained model
        configs.pretrained_path = configs.pretrained_filename
        # Number of convolutional layers to use
        configs.num_layers = 18
        # The number of top 'K'
        configs.K = 50
        # If True, cuda is not used
        configs.no_cuda = False
        # GPU index to use
        configs.gpu_idx = 0
        # Subset of dataset to run and debug
        configs.num_samples = None
        # Number of threads for loading data 
        configs.num_workers = 1
        # Number of samples per mini-batch
        configs.batch_size = 1
        # 
        configs.peak_thresh = 0.2
        # If True, output image of testing phase will be saved
        configs.save_test_output = False
        # Type of test output (can be one of: ['image', 'video'])
        configs.output_format = 'image'
        # The video filename to use (if the output format is 'video')
        configs.output_video_fn = 'out_fpn_resnet'
        # The width of the output
        configs.output_width = 608
        configs.pin_memory = True
        # Is False when testing on a single GPU only
        configs.distributed = False
        # The input Bird's-Eye View (BEV) image size
        configs.input_size = (608, 608)
        # The bounding box anchor size for balanced L1 loss
        configs.hm_size = (152, 152)
        # The down-sampling ratio S s.t. (H/S, W/S, C)
        # for C num. classes and heatmap for main center of size (H, W)
        configs.down_ratio = 4
        # Max. number of predictions to keep, i.e., the maximum number
        # of predictions whose center confidences are greater than 0.2
        configs.max_objects = 50
        configs.imagenet_pretrained = False
        # The num. channels in the convolutional layer for the output head
        # 64 for ResNet and 256 for DLA
        configs.head_conv = 64
        configs.num_classes = 3
        # The center offset, i.e., (H/S, W/S, 2)
        configs.num_center_offset = 2
        # The z-coordinate dimension, i.e., (H/S, W/S, 1)
        configs.num_z = 1
        # The number of channels in the input, i.e., (H, W, 3)
        configs.num_dim = 3
        # Model estimates the complex-valued yaw angles
        # The `im` and `re` fractions are directly regressed using `l1_loss`
        configs.num_direction = 2    # sin, cos
        # Parameters used for the regression tasks, i.e., the
        # Sigmoid / Focal / L1 / Balanced L1 loss functions
        configs.heads = {
            'hm_cen': configs.num_classes,
            'cen_offset': configs.num_center_offset,
            'direction': configs.num_direction,
            'z_coor': configs.num_z,
            'dim': configs.num_dim
        }
        configs.num_input_features = 4
        configs.root_dir = '../'
        configs.dataset_dir = os.path.join(
            configs.root_dir, 'dataset'
        )
        if configs.save_test_output:
            configs.result_dir = os.path.join(
                configs.root_dir, 'results', configs.saved_fn
            )
        ####### ID_S3_EX1-3 END #######     
    else:
        raise ValueError("Error: Invalid model name '{model_name}'")
    ### Configurations for training on either GPU vs. CPU
    configs.no_cuda = True # if true, cuda is not used
    configs.gpu_idx = 0  # GPU index to use.
    configs.device = torch.device(
        'cpu' if configs.no_cuda else 'cuda:{}'.format(configs.gpu_idx)
    )
    return configs


### Load all object detection parameters into an `EasyDict` instance (ID_S3_EX2)
def load_configs(
        model_name: str='fpn_resnet', configs: easydict.EasyDict=None
):
    """Returns the modified `EasyDict` instance with model parameters.

    :param model_name: the model to load the configuration settings for.
    :param configs: the `EasyDict` instance to store the configuration settings.
    :returns: configs, the configured `EasyDict` instance.
    """

    ### Instantiate the `EasyDict` instance if none has been passed
    if configs==None:
        configs = easydict.EasyDict()    
    ### Set the Bird's-Eye View (BEV) map parameters
    # The detection range in metres (m)
    configs.lim_x = [0, 50]
    configs.lim_y = [-25, 25]
    configs.lim_z = [-1, 3]
    # The reflected LiDAR intensity range
    configs.lim_r = [0, 1.0]
    # The pixel resolution of the BEV map image
    configs.bev_width = 608
    configs.bev_height = 608 
    ### Fetch and set the model-dependent parameters
    configs = load_configs_model(model_name, configs)
    ### Set the visualisation parameters
    # The width of the resulting image (height may vary)
    configs.output_width = 608
    # The bounding box colours for each class
    # Here we have {'Pedestrian': 0, 'Car': 1, 'Cyclist': 2}
    configs.obj_colors = [[0, 255, 255], [0, 0, 255], [255, 0, 0]]
    return configs


### Create model according to selected model type (ID_S3_EX1-4)
def create_model(
        configs: easydict.EasyDict
) -> torch.nn.Module:
    """Returns a `torch.nn.Module` instance from the specification in `configs.`

    :param configs: the `EasyDict` instance specifying the object detection
        neural network architecture to build.
    :returns: model, the instantiated neural network submodule configured with
        the `torch.nn.Module` base class.
    """

    ### Check for availability of `model` file
    exists = os.path.isfile(configs.pretrained_filename)
    assert exists, f"No file at {configs.pretrained_filename}"
    ### Create the `model` according to the specified architecture name
    if configs.arch == 'darknet' and configs.cfgfile:
        print('Using the DarkNet model architecture')
        # Build the DarkNet model according to the defined specifications
        model = darknet(
                    cfgfile=configs.cfgfile,
                    use_giou_loss=configs.use_giou_loss
        )
    elif 'fpn_resnet' in configs.arch:
        print('Using the ResNet model architecture with Feature Pyramid')
        ####### ID_S3_EX1-4 START #######     
        # Build the ResNet model according to the defined specifications
        model = fpn_resnet.get_pose_net(
                    num_layers=configs.num_layers,
                    heads=configs.heads,
                    head_conv=configs.head_conv,
                    imagenet_pretrained=configs.imagenet_pretrained
        )
        print("student task ID_S3_EX1-4")
        ####### ID_S3_EX1-4 END #######
    else:
        assert False, f"Undefined model backbone: '{configs.arch}'"
    ### Load the pre-trained `model` weights
    model.load_state_dict(
        torch.load(configs.pretrained_filename, map_location='cpu')
    )
    print(f"Loaded weights from '{configs.pretrained_filename}'\n")
    ### Set `model` to evaluation state
    configs.device = torch.device(
        'cpu' if configs.no_cuda else f"cuda:{configs.gpu_idx}"
    )
    # Load the `torch.nn.Module` instance either to CPU or GPU
    model = model.to(device=configs.device)
    model.eval()
    # Return the configured and loaded model
    return model


### Detect trained objects in Bird's-Eye View map (ID_S3_EX2)
def detect_objects(
        input_bev_maps: , model: torch.nn.Module, configs: easydict.EasyDict
) -> :
    """Perform inference and post-process the object detections.

    :param input_bev_maps: the BEV images to perform inference over, i.e.,
        the images containing objects to detect.
    :param model: the pre-trained object detection net as a `torch.nn.Module`
        instance.
    :param configs: the `EasyDict` instance containing the confidence and NMS
        threshold values and a path to the pre-trained model.
    :returns: objects, the set of predictions.
    """

    # deactivate autograd engine during test to reduce memory usage and speed up computations
    with torch.no_grad():  

        # perform inference
        outputs = model(input_bev_maps)

        # decode model output into target object format
        if 'darknet' in configs.arch:

            # perform post-processing
            output_post = post_processing_v2(outputs, conf_thresh=configs.conf_thresh, nms_thresh=configs.nms_thresh) 
            detections = []
            for sample_i in range(len(output_post)):
                if output_post[sample_i] is None:
                    continue
                detection = output_post[sample_i]
                for obj in detection:
                    x, y, w, l, im, re, _, _, _ = obj
                    yaw = np.arctan2(im, re)
                    detections.append([1, x, y, 0.0, 1.50, w, l, yaw])    

        elif 'fpn_resnet' in configs.arch:
            # decode output and perform post-processing
            
            ####### ID_S3_EX1-5 START #######     
            #######
            print("student task ID_S3_EX1-5")

            #######
            ####### ID_S3_EX1-5 END #######     

            

    ####### ID_S3_EX2 START #######     
    #######
    # Extract 3d bounding boxes from model response
    print("student task ID_S3_EX2")
    objects = [] 

    ## step 1 : check whether there are any detections

        ## step 2 : loop over all detections
        
            ## step 3 : perform the conversion using the limits for x, y and z set in the configs structure
        
            ## step 4 : append the current object to the 'objects' array
        
    #######
    ####### ID_S3_EX2 START #######   
    
    return objects    

