#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import PIL.Image
import numpy as np
import torch

import torchvision.transforms as transforms
import torchvision.datasets as datasets


def get_x(path, width):
    """Gets the x value from the image filename"""
    return (float(int(path.split("_")[1])) - width/2) / (width/2)

def get_y(path, height):
    """Gets the y value from the image filename"""
    return (float(int(path.split("_")[2])) - height/2) / (height/2)


class XYDataset(torch.utils.data.Dataset):
    
    def __init__(self, directory, transform=None, random_hflips=False):
        self.directory = directory
        self.transform = transform
        self.image_paths = glob.glob(os.path.join(self.directory, '*.jpg'))
        self.random_hflips = random_hflips
        #self.color_jitter = transforms.ColorJitter(0.3, 0.3, 0.3, 0.3)
    
    def __len__(self):
        return len(self.image_paths)
    
    def __getitem__(self, idx):
        image_path = self.image_paths[idx]
        
        image = PIL.Image.open(image_path)
        width, height = image.size
        
        x = float(get_x(os.path.basename(image_path), width))
        y = float(get_y(os.path.basename(image_path), height))
      
        if self.random_hflips and float(np.random.rand(1)) > 0.5:
            image = transforms.functional.hflip(image)
            x = -x
        
        #image = self.color_jitter(image)
        
        if self.transform is not None:
            image = self.transform(image)
 
        return image, torch.tensor([x, y]).float()
