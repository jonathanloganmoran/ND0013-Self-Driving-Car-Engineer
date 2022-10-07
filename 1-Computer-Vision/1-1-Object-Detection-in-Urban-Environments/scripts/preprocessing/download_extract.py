from download_process import download_tfr
import pandas as pd
import waymo_open_dataset


def extract_frame_data(
	frame: waymo_open_dataset.dataset_pb2.Frame,
	attr_dict: dict,
	use_laser_counts=False
) -> dict:
	"""Extracts the attribute data from a single Frame instance.  

	:param frame: the Waymo Open Dataset `Frame` instance.
	:param attr_dict: the dict instance containing the <key, None>
		pairs to fetch from the `Frame`.
	:param use_laser_counts: bool (optional), if True, the `laser_object_counts`
		are retrieved from `frame.context.stats`. Otherwise, 
		`camera_object_counts` are retrieved.
	:returns: attr_dict, the dict instance populated with the data from `frame`.
	"""

	def object_type_name(x: int):
		"""Returns the string class label mapping to the input class id."""
		return label_pb2.Label.Type.Name(x)

	### Get the requested stats
	attr_dict.update({
		key: frame.context.stats.key for key in attr_dict.keys()
	})
	### Get the object counts
	if use_laser_counts:
		attr_dict.update({
			object_type_name(x.type): x.count for x in frame.context.stats.laser_object_counts
	})
	else:
		attr_dict.update({
			object_type_name(x.type): x.count for x in frame.context.stats.camera_object_counts
	})
	return attr_dict


def process_tfrs(
	filename_paths: str
) -> pd.DataFrame:
	"""Creates a TFRecordDataset and extracts the attribute data.

	:param filename_paths: list of local paths to the downloaded records.
	:returns: df_frame, a Pandas DataFrame of all extracted attribute data.
	"""

	### Create a DataFrame instance to store all frame data
	df_frame = pd.DataFrame()
	### The scene attributes in `context.stats` to fetch from the frame
	attr_dict = {
		time_of_day: None,
		location: None,
		weather: None
	}
	for fn_path in filename_paths:
		dataset = tf.data.TFRecordDataset(fn_path, compression_type='')
		for data in dataset:
			logging.info(f'Processing {fn_path}')
			frame = open_dataset.Frame()
			frame.ParseFromString(bytearray(data.numpy()))
			frame_data = extract_frame_data(frame, attr_dict)
			df_frame = df_frame.append(frame_data, ignore_index=True)
	return df_frame


def download_and_process(path_to_filenames: str, data_dir: str, delete_records=False):
    """Downloads the requested files and converts them to TF-compatible format.

    Decorated with Ray to run asynchronously on separate Python workers.
    See: https://docs.ray.io/en/latest/ray-core/tasks.html#tasks

    :param path_to_filenames: the file path of the text file containing all `.tfrecord` 
    	files to download from GCS. This should be a list of strings starting with
    	'gs://'. The file paths should also include the bucket name.
    :param data_dir: the path to the local directory to store the downloaded files.
    :param delete_records: bool (optional), flag to remove the downloaded `.tfrecords` from 
    	the local drive if True.
    """

	### Opening the list of file paths to download from GCS with gsutil
	with open(path_to_filenames, 'r') as f:
	    filename_paths = f.read().splitlines()
	logger.info(
		f'Downloading {
				len(filename_paths)
	    } files. Be patient, this will take a long time.'
	)
    ### Re-import the logger for multiprocesing
    logger = get_module_logger(__name__)
    ### List of all local file paths of the downloaded `.tfrecord` files
    local_paths = []
    for fn_path in filename_paths:
    	local_path = download_tfr(fn_path, data_dir)
    	local_paths.append(local_path)
    ### Process the `.tfrecord` files and return their attribute data as a DataFrame
    df_frame = process_tfrs(local_paths, data_dir)
    ### Delete the original `.tfrecord` files to save space
    if delete_records:
	    for fn_path in filename_paths:
	    	logger.info(f'Deleting {fn_path}')
	    	os.remove(fn_path)
    return df_frame