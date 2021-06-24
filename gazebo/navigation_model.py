#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil

import torch
import torch.nn as nn

import torchvision.transforms as transforms
import torchvision.datasets as datasets
import torchvision.models as models

from datetime import datetime
from reshape_model import reshape_model


model_names = sorted(name for name in models.__dict__
    if name.islower() and not name.startswith("__")
    and callable(models.__dict__[name]))

torch.manual_seed(1)  # reproducibility


class NavigationModel:
    """
    Model for navigation
    """
    def __init__(self, model, resolution=224):
        """
        Create or load a model.
        """
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
            
            print(f"     - arch           {checkpoint['arch']}")
            print(f"     - classes        {checkpoint['num_classes']}")
            print(f"     - train accuracy {checkpoint['train_accuracy']:.8f}")
            print(f"     - val accuracy   {checkpoint['val_accuracy']:.8f}")
            
            self.model_arch = checkpoint['arch']
            self.num_classes = checkpoint['num_classes']
            
            self.model = models.__dict__[self.model_arch](pretrained=True)
            self.model = reshape_model(self.model, self.model_arch, self.num_classes)
            
            self.model.load_state_dict(checkpoint['state_dict'])
            self.model.cuda()
        else:
            print(f"=> creating model '{model}'")
            self.model = models.__dict__[model](pretrained=True)
            self.model_arch = model
            self.num_classes = 1000    # default classes for torchvision models
    
    def infer(self, image):
        image = self.data_transforms(image).unsqueeze(0).cuda()

        self.model.eval()
        
        with torch.no_grad():
            output = self.model(image)
            output = nn.functional.softmax(output)
            prob, cls = torch.max(output, 1)
            
        return cls.item(), prob.item()
        
    def train(self, dataset, epochs=10, batch_size=1, workers=1, learning_rate=0.01, train_split=0.8, print_freq=10, 
              use_class_weights=True, save=f"data/models/{datetime.now().strftime('%Y%m%d%H%M')}"):
        """
        Train the model on a dataset
        """
        train_loader, val_loader, class_weights = self.load_dataset(dataset, batch_size, workers, train_split)
        
        self.model.cuda()
        
        # setup model, loss function, and solver
        criterion = nn.CrossEntropyLoss(weight=torch.Tensor(class_weights) if use_class_weights else None).cuda()
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)   
        
        best_accuracy = -1.0
        
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
                accuracy = self.accuracy(output, target)
                train_accuracy += accuracy
                train_loss += loss
                
                if i % print_freq == 0:
                    print(f"Epoch {epoch}:  train=[{i}/{len(train_loader)}]  loss={train_loss/(i+1):.8f}  accuracy={train_accuracy/(i+1):.8f}")
            
            train_loss /= len(train_loader)
            train_accuracy /= len(train_loader)
            
            if val_loader is not None:
                val_loss, val_accuracy = self.validate(val_loader, criterion, epoch, print_freq)
            else:
                val_loss = train_loss
                val_accuracy = train_accuracy
                
            print(f"Epoch {epoch}:  train_loss={train_loss:.8f}  train_accuracy={train_accuracy:.8f}")
            print(f"Epoch {epoch}:  val_loss={val_loss:.8f}  val_accuracy={val_accuracy:.8f}")
            
            if save:
                self.save_checkpoint({
                    'epoch': epoch,
                    'arch': self.model_arch,
                    'resolution': self.resolution,
                    'num_classes': self.num_classes,
                    'state_dict': self.model.state_dict(),
                    'train_accuracy': train_accuracy,
                    'val_accuracy': val_accuracy,
                }, val_accuracy > best_accuracy, save)
            
            best_accuracy = max(val_accuracy, best_accuracy)
            
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
                accuracy = self.accuracy(output, target)
                val_accuracy += accuracy
                val_loss += loss
                
                if i % print_freq == 0:
                    print(f"Epoch {epoch}:  val=[{i}/{len(val_loader)}]  loss={val_loss/(i+1):.8f}  accuracy={val_accuracy/(i+1):.8f}")
        
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
                                     
        dataset = datasets.ImageFolder(dataset, self.data_transforms)
          
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

        # reshape model if needed     
        dataset_classes = len(dataset.classes)
        
        print(f'=> dataset classes: {dataset_classes} ({str(dataset.classes)})')
        print(f'=> train samples:   {len(train_dataset)}')
        print(f'=> val samples:     {len(val_dataset) if val_dataset is not None else 0}')
        
        if self.num_classes != dataset_classes:
            self.model = reshape_model(self.model, self.model_arch, dataset_classes)
            self.num_classes = dataset_classes
            
        # get class weights
        class_weights, class_counts = self.get_class_weights(dataset)
        
        print('=> class distribution:')
        
        for idx, (weight, count) in enumerate(zip(class_weights, class_counts)):
            print(f'     [{idx}] - {count} samples ({count/sum(class_counts):.4f}), weight {weight:.8f}')
            
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
 
if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser()

    parser.add_argument('--model', default='resnet18', type=str)
    parser.add_argument('--dataset', default='data/dataset', type=str)
    parser.add_argument('--epochs', default=10, type=int)
    parser.add_argument('--batch-size', default=1, type=int)
    parser.add_argument('--train-split', default=0.8, type=float)
    
    args = parser.parse_args()
    print(args)
    
    model = NavigationModel(args.model)
    model.train(args.dataset, epochs=args.epochs, batch_size=args.batch_size, train_split=args.train_split)