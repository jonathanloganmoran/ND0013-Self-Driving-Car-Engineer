from functools import wraps
from omegaconf import DictConfig
import ray
from typing import Callable

### Using a custom decorator to wrap `@ray.remote` into `@hydra.main` ###
### Note: this can be replaced with the Hydra Compose API ###
# For information on the Hydra Compose API route:
# See: https://hydra.cc/docs/advanced/compose_api/
# See: https://github.com/facebookresearch/hydra/blob/main/examples/advanced/ray_example/ray_compose_example.py
# For information on the custom decorator route:
# See: https://hydra.cc/docs/advanced/decorating_main/
# See: https://stackoverflow.com/questions/52517273/merging-python-decorators-with-arguments-into-a-single-one


def hydra_with_ray_remote(func: Callable) -> Callable:
    @functools.wraps(func)
    def inner_decorator(cfg: DictConfig):
        ray.remote(func)
        return func(cfg)
    return inner_decorator

