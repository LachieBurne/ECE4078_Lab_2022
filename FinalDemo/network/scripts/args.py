import os
import argparse

parser = argparse.ArgumentParser()

# n_classes specifies target types, no changes needed
parser.add_argument("--n_classes", metavar='', type=int, default=5)

# initial learning rate (can be tuned):
# the starting point for the learning rate scheduler
# a learning rate too high can lead to exploding or fluctuating loss
# a learning rate too low can lead to slow learning or overfitting
parser.add_argument("--lr", metavar='', type=float, default=1.0e-3)

# total number of epochs (can be tuned):
# when to stop the training
# a higher number of epochs can lead to overfitting
parser.add_argument("--epochs", metavar='', type=int, default=40)

# batch size (can be tuned):
# a smaller batch size means the parameters are updated using a smaller subset of the data
# thus the parameter update will be more noisy, but the model may have better generalizability
parser.add_argument("--batch_size", metavar='', type=int, default=64)

# weight decay (can be tuned):
# adding a penalty to the cost which can lead to smaller model weights
parser.add_argument("--weight_decay", metavar='', type=float, default=1.0e-4)

# learning rate scheduler (can be tuned): 
# controls how fast the learning rate decreases
# here, the learning rate is designed to be its 50% every 5 epoch.
parser.add_argument("--scheduler_step", metavar='', type=int, default=5,
        help='the learning rate is reduced to <?>')
parser.add_argument("--scheduler_gamma", metavar='', type=float, default=0.5,
        help='the learning rate is reduced to <?> every <?> epochs')

# Trainer Configuration, no changes neccessary
parser.add_argument("--model_dir", metavar='', type=str, default="",
                    help="folder to save the experiment")
parser.add_argument("--load_best", metavar='', type=int, default=0, 
    help=' load the best checkpoint')
parser.add_argument("--log_freq", metavar='', type=int, default=20,
                    help=" record a training log every <?> mini-batches")
parser.add_argument('--dataset_dir', metavar='', type=str, default="")

args, _ = parser.parse_known_args()

try: 
    args.model_dir=os.environ['SM_MODEL_DIR']
    args.dataset_dir = os.environ['SM_CHANNEL_TRAIN']
except:
    pass

