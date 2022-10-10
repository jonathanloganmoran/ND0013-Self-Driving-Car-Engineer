from configs.dataclass.dataclasses import SSDResNet50Config
#from configs.decorators.decorator import hydra_with_ray_remote
import hydra
from hydra import compose, initialize
from omegaconf import DictConfig, OmegaConf
from hydra.core.config_store import ConfigStore
import os
from configs.dataclass.dataclasses import SSDResNet50Config
import omegaconf


cs = ConfigStore.instance()
cs.store(name='model_config', node=SSDResNet50Config)
OmegaConf.register_new_resolver(
    "abspath", lambda relative_path: os.path.abspath(relative_path)
)


os.environ['HYDRA_FULL_ERROR'] = '1'
@hydra.main(version_base=None, config_path='../configs', config_name='config.yaml')
def test_dataset(cfg: SSDResNet50Config):
    cfg = OmegaConf.create(cfg)
    print(cfg)
    #print(cfg.keys())
    #print(cfg.values())
    print(cfg['dataset'].destination)


@hydra.main(version_base=None, config_path='../configs', config_name='config.yaml')
def test_model(cfg: SSDResNet50Config):
    cfg = OmegaConf.create(cfg)
    print(cfg.model.keys())
    print(cfg['model'].dir_out)

"""
@hydra.main(version_base=None, config_path='../../configs', config_name='config.yaml')
@hydra_with_ray_remote
def test_decorator(cfg: SSDResNet50Config):
    print(cfg)
    print("Succeeded")
"""

def test_compose():
    with initialize(version_base=None, config_path='../configs'):
        cfg = compose(config_name='config.yaml')
        cfg = OmegaConf.create(cfg)
        print(cfg)


if __name__ == '__main__':
    test_dataset()
    #test_model()
    #test_decorator()
    test_compose()