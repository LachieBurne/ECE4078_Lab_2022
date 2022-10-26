import argparse
import os

import cmd_printer
from imdb import imdb_loader
from res18_skip import Resnet18Skip
from trainer import Trainer
from args import args

if __name__ == '__main__':
    # print args
    cmd_printer.divider(text="Hyper-parameters", line_max=60)
    for arg in vars(args):
        print(f"   {arg}: {getattr(args, arg)}")
    cmd_printer.divider(line_max=60)

    train_loader, eval_loader = imdb_loader(args)
    model = Resnet18Skip(args)
    trainer = Trainer(args)
    trainer.fit(model, train_loader, eval_loader)
