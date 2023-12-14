import numpy as np
import json
import csv
from sklearn import tree, ensemble
import random
import pickle
import datetime

#How many times should we iterate over our datasets and add scoring noise?
noiseIter = 3
noiseRange = 0.15

# get data (Olivetti)
file = open('encodings.json')
dataVectors = json.load(file) #array of objects
results = []
image_directory = []
predictors_list = [] 

with open("image_scores.csv", "r") as sheet:
    reader = csv.reader(sheet)
    rows = np.array(list(reader))
    
    for index in range(0, len(dataVectors)):
        #Retrieve row by filename
        results.append(float(rows[rows[:, 0] == dataVectors[index]['filename'].split('/')[2]][0, 1]))
        image_directory.append(dataVectors[index]['filename'].split('/')[2])
        predictors_list.append(
            np.concatenate((
            np.array(dataVectors[index]['shape_param']),
            np.array(dataVectors[index]['expression_param']),
            np.array(dataVectors[index]['color_param']),
            np.array(dataVectors[index]['camera_param']),
            np.array(dataVectors[index]['illumination_param'])))
            )

print(results)

#Origial data dataframe
train_predictors = [] + predictors_list
train_results = [] + results

for n in range(0, noiseIter):
    #Add to predictors, results some noise:
    train_predictors += predictors_list
    train_results += [x - noiseRange + 2*random.random()*noiseRange for x in results] #Assume each image can get varying scores


tested_images = []
test_predictors = []
test_results = []

for i in range(10):
    index = int(random.random() * len(predictors_list))
    tested_images.append(image_directory[index])
    test_predictors.append(predictors_list[index])
    test_results.append(results[index])

tested_images.append(image_directory[len(predictors_list) - 1])
test_predictors.append(predictors_list[len(predictors_list) - 1])
test_results.append(results[len(predictors_list) - 1])

#Build RF
dtr = ensemble.RandomForestRegressor(n_estimators = 1000, min_samples_leaf = 5)

#Train RF
regr = dtr.fit(train_predictors, train_results)

#Use RF
pred_y = regr.predict(test_predictors)

filehandler = open("trained_models/rf_model" + datetime.datetime.now().strftime('%H_%M_%S-%Y-%m-%d') + ".obj", "wb")
pickle.dump(regr, filehandler)

print("Testing!\n")
for i in range(len(pred_y)):
    print("Image: " + tested_images[i] + " -> Expected: " + str(test_results[i]))
    print("Predicted: " + str(pred_y[i]))


#Mean Squared Error
mse = np.mean(np.square(test_results-pred_y))

print(mse)