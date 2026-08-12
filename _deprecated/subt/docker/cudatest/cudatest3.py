
import torch

if torch.cuda.is_available():
    print("torch ok")
else:
    print("torch failed")

import tensorflow as tf

if len(tf.config.list_physical_devices('GPU')) >= 1:
    print("tf ok")
else:
    print("tf failed")
