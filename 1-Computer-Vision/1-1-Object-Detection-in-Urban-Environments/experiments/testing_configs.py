import hydra
from hydra.core.config_store import ConfigStore
import os
from configs.dataclass.dataclasses import SSDResNet50Config


cs = ConfigStore.instance()
cs.store(name='model_config', node=SSDResNet50Config)


os.environ['HYDRA_FULL_ERROR'] = '1'
@hydra.main(version_base=None, config_path='../configs', config_name='config')
def test_dataset(cfg: SSDResNet50Config):
	print(cfg.keys())


@hydra.main(version_base=None, config_path='../configs', config_name='config')
def test_model(cfg: SSDResNet50Config):
	print(cfg.model.keys())
	print(cfg.dataset.keys())



if __name__ == '__main__':
	test_dataset()
	test_model()