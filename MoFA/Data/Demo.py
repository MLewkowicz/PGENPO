 #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  3 10:42:21 2021

@author: li0005
"""

import torch
import os
import util.util as util
import util.load_dataset as load_dataset
import cv2
import numpy as np
import argparse
import FOCUS_model.FOCUS_basic as FOCUSModel
from util.get_landmarks import get_landmarks_main
import time
import pickle
import warnings
import json
import csv

torch.set_grad_enabled(False)

warnings.filterwarnings("ignore")

par = argparse.ArgumentParser(description='Test: MoFA+UNet ')
par.add_argument('--pretrained_encnet_path',default='./MoFA_UNet_Save/MoFA_UNet_CelebAHQ/unet_200000.model',type=str,help='Path of the pre-trained model')
par.add_argument('--gpu',default=0,type=int,help='The GPU ID')
par.add_argument('--batch_size',default=8,type=int,help='Batch size')
par.add_argument('--img_path',type=str,help='Root of the images')
par.add_argument('--use_MisfitPrior',type=util.str2bool,help='Root of the images')
torch.set_grad_enabled(False) # make sure to not compute gradients for computational performance

args = par.parse_args()


args.img_path = (args.img_path +'/').replace('//','/')

args.pretrained_encnet_path = args.pretrained_encnet_path.replace('/unet_','/enc_net_')
args.device = torch.device("cuda:{}".format(args.gpu) if torch.cuda.is_available() else "cpu")

#parameters
args.width = args.height = 224

current_path = os.getcwd()  
args.model_path = current_path+'/basel_3DMM/model2017-1_bfm_nomouth.h5'


pretrained_unet_path =args.pretrained_encnet_path.replace('enc_net_','unet_')
if os.path.exists(pretrained_unet_path):
    args.pretrained_unet_path = pretrained_unet_path
    args.where_occmask='UNet'
else:
    print('No UNet available!!!')
print('loading encoder: ' + args.pretrained_encnet_path  )
print('Masks estimated by '+args.where_occmask)


FOCUSModel = FOCUSModel.FOCUSmodel(args)
FOCUSModel.init_for_now_challenge()


def write_lm_txt(filename_save,lms_ringnet_temp):
    anno_file = open( filename_save,"w")
    
    lms  = lms_ringnet_temp.T.astype(np.int)
            
    str_temp = ''
    for i_temp in range(7):
        str_temp +='{} {} {}\n'.format(lms[i_temp,0],lms[i_temp,1],lms[i_temp,2])
    anno_file.write(str_temp)    
    anno_file.close()
    



save_name = time.time()
save_dir = './Results/{}'.format(save_name)
if not os.path.exists(save_dir):
    os.makedirs(save_dir)
save_dir_recon_result = os.path.join(save_dir,'reconstruction_results')
if not os.path.exists(save_dir_recon_result):
    os.makedirs(save_dir_recon_result)

    
    
    
'''------------------
  Prepare Log Files
------------------'''

log_path_train = os.path.join(save_dir,"{}_log.pkl".format(save_name))
dist_log = {}
dist_log['args'] = args
with open(log_path_train, 'wb') as f:
    pickle.dump(dist_log, f)

del dist_log

landmark_filepath = get_landmarks_main(args.img_path,save_dir) 
testset = load_dataset.CelebDataset(args.img_path,False,landmark_file=landmark_filepath)
testloader = torch.utils.data.DataLoader(testset, batch_size=args.batch_size,shuffle=False, num_workers=0)

im_iter=0
    
if args.use_MisfitPrior:
    prior_path = './MisfitPrior/MisfitPrior.pt'
    prior = torch.load(prior_path,map_location=args.device).detach().to('cpu').numpy()
'''---------------------------------------------------
Reconstructed results, masks
---------------------------------------------------'''

encodings_path = os.path.join(save_dir, f"encodings.json")
encodings = [] #['Shape,Expression,Color,Camera,Illumination\n']

landmark_list = list(csv.reader(open(landmark_filepath),delimiter=','))
filenames = [landmark[0] for landmark in landmark_list]

with torch.no_grad():
    for i, data in enumerate(testloader, 0):
        data = FOCUSModel.data_to_device(data)

        batch_encodings = FOCUSModel.encode_only(data)
        shape_param, expression_param, color_param, camera_param, illumination_param = batch_encodings
        print(shape_param.shape)
        for j in range(len(shape_param)):  # Handling batched data
            #row = [param[j].numpy().tolist() if param[j].ndim > 0 else param[j].item() for param in [shape_param, expression_param, color_param, camera_param, illumination_param]]
            encodings.append({
                'filename': filenames[j],
                'shape_param': shape_param.numpy()[j,:].tolist(),
                'expression_param': expression_param.numpy()[j,:].tolist(),
                'color_param': color_param.numpy()[j,:].tolist(),
                'camera_param': camera_param.numpy()[j,:].tolist(),
                'illumination_param': illumination_param.numpy()[j,:].tolist()
            })

# Write the accumulated encodings to a file
with open(encodings_path, 'w') as f:
    json.dump(encodings, f, indent=4)