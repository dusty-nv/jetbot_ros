#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import numpy as np

import torch
import torch.nn as nn

import torchvision.transforms as transforms
import torchvision.datasets as datasets
import torchvision.models as models

from torch.optim.lr_scheduler import StepLR, ReduceLROnPlateau
from datetime import datetime

from reshape_model import reshape_model
from xy_dataset import XYDataset

model_names = sorted(name for name in models.__dict__
    if name.islower() and not name.startswith("__")
    and callable(models.__dict__[name]))

torch.manual_seed(1)  # reproducibility


class NavigationModel:
    """
    Model for navigation
    """
    def __init__(self, model, type='classification', resolution=224):
        """
        Create or load a model.
        """
        if type != 'classification' and type != 'regression':
            raise ValueError("type must be 'classification' or 'regression' (was '{type}')")
            
        self.type = type
        self.resolution = resolution
        
        self.data_transforms = transforms.Compose([
                transforms.Resize((self.resolution, self.resolution)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                     std=[0.229, 0.224, 0.225]),
            ])
            
        # load model
        if len(os.path.splitext(model)[1]) > 0:
            print(f"=> loading model '{model}'")
            checkpoint = torch.load(model)
            
            if self.type != checkpoint['type']:
                raise ValueError(f"'{model}' is a {checkpoint['type']} model, but expected a {self.type} model")
                
            print(f"     - arch           {checkpoint['arch']}")
            print(f"     - type           {checkpoint['type']}")
            print(f"     - outputs        {checkpoint['num_outputs']}")
            print(f"     - train loss     {checkpoint['train_loss']:.8f}")
            print(f"     - val loss       {checkpoint['val_loss']:.8f}")
            
            if self.classification:
                print(f"     - train accuracy {checkpoint['train_accuracy']:.8f}")
                print(f"     - val accuracy   {checkpoint['val_accuracy']:.8f}")

            self.model_arch = checkpoint['arch']
            self.num_outputs = checkpoint['num_outputs']
            
            self.model = models.__dict__[self.model_arch](pretrained=True)
            self.model = reshape_model(self.model, self.model_arch, self.num_outputs)
            
            self.model.load_state_dict(checkpoint['state_dict'])
            self.model.cuda()
        else:
            print(f"=> creating model '{model}'")
            self.model = models.__dict__[model](pretrained=True)
            self.model_arch = model
            self.num_outputs = 1000    # default classes for torchvision models
    
    @property
    def classification(self):
        return self.type == 'classification'
      
    @property
    def regression(self):
        return self.type == 'regression'
        
    def infer(self, image):
        image = self.data_transforms(image).unsqueeze(0).cuda()

        self.model.eval()
        
        with torch.no_grad():
            output = self.model(image)
            
            if self.classification:
                output = nn.functional.softmax(output)
                prob, cls = torch.max(output, 1)
                return cls.item(), prob.item()
            else:
                return output.detach().squeeze().cpu().numpy() if output.requires_grad else output.squeeze().cpu().numpy()
        
    def train(self, dataset, epochs=10, batch_size=1, learning_rate=0.01, scheduler='StepLR_75', 
              workers=1, train_split=0.8, print_freq=10, use_class_weights=True, 
              save=f"data/models/{datetime.now().strftime('%Y%m%d%H%M')}"):
        """
        Train the model on a dataset
        """
        train_loader, val_loader, class_weights = self.load_dataset(dataset, batch_size, workers, train_split)
        
        self.model.cuda()
        
        # setup model, loss function, and solver
        if self.classification:
            criterion = nn.CrossEntropyLoss(weight=torch.Tensor(class_weights) if use_class_weights else None).cuda()
        else:
            criterion = nn.MSELoss()
            
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        scheduler = self._create_scheduler(scheduler, optimizer)        
        best_metric = -np.inf if self.classification else np.inf
        
        # train for the specified number of epochs
        for epoch in range(epochs):
            self.model.train()
            
            train_loss = 0.0
            train_accuracy = 0.0
            
            for i, (images, target) in enumerate(train_loader):
                images = images.cuda(non_blocking=True)
                target = target.cuda(non_blocking=True)
                
                # compute model output
                output = self.model(images)
                loss = criterion(output, target)
                
                # compute gradient and do solver step
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
        
                # keep track of accuracy/loss over the batch
                accuracy = self.accuracy(output, target) if self.classification else 0.0
                train_accuracy += accuracy
                train_loss += loss
                
                if i % print_freq == 0:
                    if self.classification:
                        print(f"Epoch {epoch}:  train=[{i}/{len(train_loader)}]  lr={scheduler._last_lr[0]:.2g}  loss={train_loss/(i+1):.8f}  accuracy={train_accuracy/(i+1):.8f}")
                    else:
                        print(f"Epoch {epoch}:  train=[{i}/{len(train_loader)}]  lr={scheduler._last_lr[0]:.2g}  loss={train_loss/(i+1):.8f}")
  
            if isinstance(scheduler, ReduceLROnPlateau):
                scheduler.step(metrics=train_loss)
            else:
                scheduler.step()
                
            train_loss /= len(train_loader)
            train_accuracy /= len(train_loader)
                    
            if val_loader is not None:
                val_loss, val_accuracy = self.validate(val_loader, criterion, epoch, print_freq)
            else:
                val_loss = train_loss
                val_accuracy = train_accuracy
                
            if self.classification:
                print(f"Epoch {epoch}:  train_loss={train_loss:.8f}  train_accuracy={train_accuracy:.8f}")
                print(f"Epoch {epoch}:  val_loss={val_loss:.8f}  val_accuracy={val_accuracy:.8f}")
            else:
                print(f"Epoch {epoch}:  train_loss={train_loss:.8f}")
                print(f"Epoch {epoch}:  val_loss={val_loss:.8f}")
                
            if save:
                checkpoint = {
                    'epoch': epoch,
                    'arch': self.model_arch,
                    'type': self.type,
                    'resolution': self.resolution,
                    'num_outputs': self.num_outputs,
                    'state_dict': self.model.state_dict(),
                    'train_loss': train_loss.item(),
                    'val_loss': val_loss.item(),
                }
                
                if self.classification:
                    checkpoint['train_accuracy'] = train_accuracy
                    checkpoint['val_accuracy'] = val_accuracy
                    
                is_best = val_accuracy > best_metric if self.classification else val_loss < best_metric
                self.save_checkpoint(checkpoint, is_best, save)
            
            if self.classification:
                best_metric = max(val_accuracy, best_metric)
            else:
                best_metric = min(val_loss, best_metric)
                
    def validate(self, val_loader, criterion, epoch, print_freq=10):
        """
        Measure model performance on the val dataset
        """
        self.model.eval()
        
        val_loss = 0.0
        val_accuracy = 0.0
        
        with torch.no_grad():
            for i, (images, target) in enumerate(val_loader):
                images = images.cuda(non_blocking=True)
                target = target.cuda(non_blocking=True)
                
                # compute model output
                output = self.model(images)
                loss = criterion(output, target)
                
                # update accuracy and loss
                accuracy = self.accuracy(output, target) if self.classification else 0.0
                val_accuracy += accuracy
                val_loss += loss
                
                if i % print_freq == 0:
                    if self.classification:
                        print(f"Epoch {epoch}:  val=[{i}/{len(val_loader)}]  loss={val_loss/(i+1):.8f}  accuracy={val_accuracy/(i+1):.8f}")
                    else:
                        print(f"Epoch {epoch}:  val=[{i}/{len(val_loader)}]  loss={val_loss/(i+1):.8f}")
                    
        val_loss /= len(val_loader)
        val_accuracy /= len(val_loader)
        
        return val_loss, val_accuracy
        
    def accuracy(self, output, target):
        """
        Compute the classification accuracy.
        """
        _, preds = torch.max(output, 1)
        return (preds == target).float().mean().cpu().item()            
    
    def save_checkpoint(self, state, is_best, path=None, filename='checkpoint.pth', best_filename='model_best.pth'):
        """
        Save a model checkpoint file, along with the best-performing model if applicable
        """
        if path:
            filename = os.path.join(path, filename)
            best_filename = os.path.join(path, best_filename)
            
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        os.makedirs(os.path.dirname(best_filename), exist_ok=True)
 
        # save the checkpoint
        torch.save(state, filename)

        # earmark the best checkpoint
        if is_best:
            shutil.copyfile(filename, best_filename)
            print("saved best model to:  " + best_filename)
        else:
            print("saved checkpoint to:  " + filename)
        
    def load_dataset(self, dataset, batch_size=2, workers=1, train_split=0.8):
        """
        Load dataset from the specified path
        """
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                         std=[0.229, 0.224, 0.225])
                            
        if self.type == 'classification':
            dataset = datasets.ImageFolder(dataset, self.data_transforms)
        elif self.type == 'regression':
            dataset = XYDataset(dataset, self.data_transforms)
            
        # split into train/val
        if train_split > 0:
            lengths = [int(len(dataset) * train_split)]
            lengths.append(len(dataset) - lengths[0])

            train_dataset, val_dataset = torch.utils.data.random_split(
                dataset, lengths, 
                generator=torch.Generator().manual_seed(1))
                
            val_loader = torch.utils.data.DataLoader(
                val_dataset, batch_size=batch_size, num_workers=workers,
                shuffle=False, pin_memory=True)
        else:
            train_dataset = dataset
            val_dataset = None
            val_loader = None
            
        # create dataloaders
        train_loader = torch.utils.data.DataLoader(
            train_dataset, batch_size=batch_size, num_workers=workers,
            shuffle=True, pin_memory=True)

        print(f'=> train samples:   {len(train_dataset)}')
        print(f'=> val samples:     {len(val_dataset) if val_dataset is not None else 0}')
        
        # reshape model if needed   
        if self.type == 'classification':
            num_outputs = len(dataset.classes)
            print(f'=> dataset classes: {dataset_classes} ({str(dataset.classes)})')
        else:
            num_outputs = 2
            
        if self.num_outputs != num_outputs:
            self.model = reshape_model(self.model, self.model_arch, num_outputs)
            self.num_outputs = num_outputs

        # get class weights
        if self.type == 'classification':
            class_weights, class_counts = self.get_class_weights(dataset)
            
            print('=> class distribution:')
            
            for idx, (weight, count) in enumerate(zip(class_weights, class_counts)):
                print(f'     [{idx}] - {count} samples ({count/sum(class_counts):.4f}), weight {weight:.8f}')
        else:
            class_weights = [1.0] * self.num_outputs
            
        return train_loader, val_loader, class_weights
 
    def get_class_weights(self, dataset):
        counts = [0] * len(dataset.classes) 
        weights = [0.] * len(dataset.classes)   
        
        for item in dataset.imgs:                                                         
            counts[item[1]] += 1                                                     
                                            
        max_count = max(counts)
        
        for i in range(len(dataset.classes)):   
            if counts[i] > 0:
                weights[i] = max_count / counts[i]   
            else:
                weights[i] = 1.0
                
        return weights, counts          

    @staticmethod
    def _create_scheduler(scheduler, optimizer):
        """
        Create a scheduler from a param string like 'StepLR_30'
        """
        if scheduler.startswith('StepLR'):
            return StepLR(optimizer, step_size=NavigationModel._parse_param(scheduler, default=30))
        elif scheduler.startswith('ReduceLROnPlateau'):
            return ReduceLROnPlateau(optimizer, patience=NavigationModel._parse_param(scheduler, default=10))
        else:
            raise ValueError(f"invalid scheduler '{scheduler}'") 
        
    @staticmethod
    def _parse_param(str, default):
        """
        Parse a parameter in a string of the form 'text_value'
        """
        idx = str.find('_')
        
        if idx < 0 or idx == (len(str) - 1):
            return default

        return int(str[idx+1:])
        
if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser()

    parser.add_argument('--model', default='resnet18', type=str)
    parser.add_argument('--type', default='classification', type=str)
    parser.add_argument('--dataset', default='data/dataset', type=str)
    parser.add_argument('--epochs', default=10, type=int)
    parser.add_argument('--batch-size', default=1, type=int)
    parser.add_argument('--learning-rate', default=0.01, type=float)
    parser.add_argument('--scheduler', default='StepLR_75', type=str)
    parser.add_argument('--train-split', default=0.8, type=float)
    
    args = parser.parse_args()
    print(args)
    
    model = NavigationModel(args.model, type=args.type)
    model.train(args.dataset, epochs=args.epochs, batch_size=args.batch_size, train_split=args.train_split)