import pickle
import argparse
import numpy as np
import json
import os
from sklearn import tree, ensemble
import random
import pickle

folder = ""

par = argparse.ArgumentParser()
par.add_argument('--folder', default='./',type=str,help='Path to the image folder')
par.add_argument('--model', default='./trained_models/', type = str, help = 'Path to the trained random forest model pickle file')

args = par.parse_args()

#Run MoFA on folder
os.system("python NewDemo.py --img_path " + args.model + " --use_MisfitPrior True --batch_size 20")

#Get encodings from results folder
results = []
image_directory = []
predictors_list = [] 

def deconstructEncodings(file):
    return json.load(file) #array of objects
    



