import os
import time
import copy
import torch
import torchvision
import pandas as pd
import numpy as np
import torch.nn as nn
import torch.optim as optim
from torch.optim import lr_scheduler
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, models, datasets
import torch.nn.functional as F
import matplotlib.pyplot as plt
from PIL import Image


def model_parser(model, fixed_weight=False, dropout_rate=0.0, bayesian=False):
    base_model = None

    if model == 'Googlenet':
        base_model = models.inception_v3(pretrained=True)
        network = GoogleNet(base_model, fixed_weight, dropout_rate)
    elif model == 'Resnet':
        base_model = models.resnet34(pretrained=True)
        network = ResNet(base_model, fixed_weight, dropout_rate, bayesian)
    elif model == 'ResnetSimple':
        base_model = models.resnet34(pretrained=True)
        network = ResNetSimple(base_model, fixed_weight)
    else:
        assert 'Invalid Model'

    return network


class ResNetSimple(nn.Module):
    def __init__(self, base_model, fixed_weight=False, dropout_rate=0.0):
        super(ResNetSimple, self).__init__()
        self.dropout_rate = dropout_rate
        feat_in = base_model.fc.in_features

        self.base_model = nn.Sequential(*list(base_model.children())[:-1])

        if fixed_weight:
            for param in self.base_model.parameters():
                param.requires_grad = False

        self.fc_last = nn.Linear(feat_in, 2048, bias=True)
        self.fc_position = nn.Linear(2048, 3, bias=True)
        self.fc_rotation = nn.Linear(2048, 4, bias=True)

        init_modules = [self.fc_last, self.fc_position, self.fc_rotation]

        for module in init_modules:
            if isinstance(module, nn.Conv2d) or isinstance(module, nn.Linear):
                nn.init.kaiming_normal_(module.weight)
                if module.bias is not None:
                    nn.init.constant_(module.bias, 0)

    def forward(self, x):
        x = self.base_model(x)
        x = x.view(x.size(0), -1)
        x_fully = self.fc_last(x)
        x = F.relu(x_fully)

        dropout_on = self.training
        if self.dropout_rate > 0:
            x = F.dropout(x, p=self.dropout_rate, training=dropout_on)

        position = self.fc_position(x)
        rotation = self.fc_rotation(x)

        return position, rotation, x_fully

