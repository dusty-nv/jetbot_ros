#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

from datetime import datetime
from navigation_model import NavigationModel


parser = argparse.ArgumentParser()

parser.add_argument('--model', default='resnet18', type=str)
parser.add_argument('--type', default='regression', type=str)
parser.add_argument('--data', default='', type=str, required=True)
parser.add_argument('--workers', default=1, type=int)
parser.add_argument('--epochs', default=10, type=int)
parser.add_argument('--batch-size', default=1, type=int)
parser.add_argument('--learning-rate', default=0.01, type=float)
parser.add_argument('--scheduler', default='StepLR_75', type=str)
parser.add_argument('--train-split', default=0.8, type=float)
parser.add_argument('--save', default=f"data/models/{datetime.now().strftime('%Y%m%d%H%M')}", type=str)

args = parser.parse_args()
print(args)

# create/load model
model = NavigationModel(args.model, type=args.type)

# train the model
model.train(args.dataset, epochs=args.epochs, batch_size=args.batch_size, learning_rate=args.learning_rate,
            scheduler=args.scheduler, workers=args.workers, train_split=args.train_split, save=args.save)
