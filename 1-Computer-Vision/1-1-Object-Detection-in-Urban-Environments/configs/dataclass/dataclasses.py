from dataclasses import dataclass
from typing import Optional


@dataclass
class Dataset:
	cwd: str
	data_dir: str
	label_map_path: str
	size: int
	destination: str
	train: str
	val: str
	test: str
	train_test_split: int 
	train_val_split: int


@dataclass
class Hyperparameters:
	epochs: int
	batch_size: int
	lr:	Optional[float]
	lr_decay: Optional[bool] = False


@dataclass
class TFModel:
    dir_base: str
    model_out: str
    checkpoint_dir: str
    pipeline_config_path: str
    num_train_steps: int
    eval_on_train_data: Optional[bool] = False
    sample_1_of_n_eval_examples: int
    sample_1_of_n_eval_on_train_examples: Optional[int]
    eval_timeout: int
    checkpoint_every_n: int
    record_summaries: bool
    num_workers: int
    eval_on_train_data: Optional[bool] = False
    use_tpu: Optional[bool] = False
    tpu_name: Optional[str] = None 
    
    
@dataclass
class SSDResNet50Config:
	model: TFModel
	hyperparameters: Hyperparameters
	dataset: Dataset