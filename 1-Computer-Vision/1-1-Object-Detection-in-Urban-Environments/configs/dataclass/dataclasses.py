from dataclasses import dataclass


@dataclass
class Dataset:
	cwd: str
	data_dir: str
	label_map_path: str
	size: int
	destination: str
	train: str
	evals: str
	test: str


@dataclass
class Hyperparameters:
	epochs: int
	batch_size: int
	lr:	float
	lr_decay: bool


@dataclass
class TFModel:
	dir_base: str
	dir_out: str
	dir_checkpoint: str
	pipeline_config: str
	train_steps: int
	eval_on_train_data: bool
	sample_1_of_n_eval_examples: int
	sample_1_of_n_eval_on_train_examples: int
	eval_timeout: int
	use_tpu: bool
	tpu_name: str 
	checkpoint_every_n: int
	num_workers: int


@dataclass
class SSDResNet50Config:
	model: TFModel
	hyperparameters: Hyperparameters
	dataset: Dataset