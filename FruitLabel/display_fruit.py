import tkinter as tk 
from PIL import ImageTk, Image
import os 
import sys

pic_map = {} 
WINDOW_X = 1200 
WINDOW_Y = 1200
# IMAGE_SIZE = 200 
curr_folder = None
folders = [] 
f_index = 0
window = tk.Tk() 
window.title("Join") 
window.geometry(str(WINDOW_X) + 'x' + str(WINDOW_Y)) 
window.configure(background = 'grey') 
dir_path = "data" 
rankings = 1 

widgets = [] 

output_file = "rankings.txt" 
set_num =  0 


#On key press from window, re-render 

#On click, store rankings
def click_cb(event): 
    global rankings
    global set_num 

    num = str(event.widget).split(".")[1]
    res_num = "".join([i for i in num if i.isdigit()])
    if res_num == '': 
        num = 0
    else: 
        num = int(res_num) - 1
    
    widget = widgets[num]
    file_path = widget[0] 
    widget[2].config(text = str(rankings)) 
    output_string = file_path + " , " + str(rankings) + "\n"
    f = open(output_file, "a")
    if rankings == 0: 
        f.write("Set " + str(set_num) + "\n")
    if rankings == 10: 
        f.write( "End of Set\n")  
    f.write(output_string)
    f.close()
    rankings += 1





def create_window(name): 
    # for folder in os.listdir(dir_path): 
    #     folders.append(folder) 

    # number = 0
    # curr_folder = folders[f_index] 
    number = 0 
    curr_folder = name 

    for file in os.listdir(dir_path + "/" + curr_folder):
        file_path = dir_path + "/" + curr_folder + "/" + file
        img = Image.open(file_path)
        print("Imiage Size: " + str(img.size)) 
        img = Image.open(file_path).resize((320,180)) 
        img = ImageTk.PhotoImage(img) 
        row_i = number // 3
        col_i = number % 3 
        #Frame holds picture + ranking 
        frame = tk.Frame(master = window, width = 400, height = 400)
        frame.grid(row = row_i, column = col_i) 
        print(row_i) 
        print(col_i) 
        panel = tk.Label(master = frame, image = img) 
        ranking = tk.Label(master = frame, text = "0") 
        widgets.append((file_path,panel, ranking))
        panel.photo = img 
        panel.pack()
        ranking.pack() 
        number += 1 

    

if __name__ == "__main__": 
    if len(sys.argv) != 3: 
        print("Incorrect number of arguments")
        exit()
    # global set_num 
    set_num = sys.argv[2] 
    path = sys.argv[1] 
    create_window(path)
    window.bind("<Button-1>", click_cb) 
    window.mainloop() 


# import json 
# import tkinter as tk 
# from PIL import ImageTk, Image
# import os 
# import sys
# import random 

# widgets = [] 

# window = tk.Tk() 
# rooted_dir = "/home/scrc/PGENPO/MoFA/Data"

# #Takes in a mapping and spits out an encoding 
# def mock_tree(encodings): 
#     return random.random() 

# def display_top(scores): 
#     for i in range(0,3):
#         if i >= len(scores): 
#             break  
#         display_image(scores[i][0], scores[i][1]) 

# def display_image(file_path, ranking): 
#     print(file_path) 
#     file_path = rooted_dir + file_path[1:]
#     img = Image.open(file_path)
#     img = Image.open(file_path).resize((640,360)) 
#     img = ImageTk.PhotoImage(img) 
#     frame = tk.Frame(master = window, width = 400, height = 400)
#     frame.pack() 
#     panel = tk.Label(master = frame, image = img) 
#     ranking = tk.Label(master = frame, text = ranking) 
#     widgets.append((file_path,panel, ranking))
#     panel.photo = img 
#     panel.pack()
#     ranking.pack() 

# if __name__ == "__main__":
#     if len(sys.argv) != 2:
#         print("Missing path argument")
#         exit() 
    
#     folder = sys.argv[1] 
#     file_path = "/home/scrc/PGENPO/MoFA/Data/Results/" + folder + "/encodings.json" 
# # "/home/scrc/PGENPO/MoFA/Data/Results/1701730427.3845797/encodings.json"
#     f = open(file_path) 
#     data = json.load(f) 
#     waypoint_count = len(data) 
#     top_imgs = [] 
#     rankings = [] 
#     scores = {} 
#     #For each encoding, match it against all the available encodings 
#     for i in range(0,waypoint_count): 
#         mappings = data[i] 
#         keys = mappings.keys() 
#         print("Filename: " + mappings['filename']) 
#         score = mock_tree(mappings) 
#         scores[mappings['filename']] = score 
#         # for k in keys: 
#         #     print("Key: " + str(k) + "\t" + str(type(mappings[k])) + "\t" + str(len(mappings[k])))
#     #Get append to file names
#     sorted_scores = sorted(scores.items(), key = lambda k: k[1])
#     print("Sorted Scores: " + str(sorted_scores)) 
#     #Display 
#     display_top(sorted_scores)
#     window.mainloop() 
    


# # import json 
# # import tkinter as tk 
# # from PIL import ImageTk, Image
# # import os 
# # import sys
# # import random 

# # widgets = [] 

# # window = tk.Tk() 
# # rooted_dir = "/home/scrc/PGENPO/MoFA/Data"

# # #Takes in a mapping and spits out an encoding 
# # def mock_tree(encodings): 
# #     return random.random() 

# # def display_top(scores): 
# #     for i in range(0,3):
# #         if i >= len(scores): 
# #             break  
# #         display_image(scores[i][0], scores[i][1]) 

# # def display_image(file_path, ranking): 
# #     print(file_path) 
# #     file_path = rooted_dir + file_path[1:]
# #     img = Image.open(file_path)
# #     img = Image.open(file_path).resize((640,360)) 
# #     img = ImageTk.PhotoImage(img) 
# #     frame = tk.Frame(master = window, width = 400, height = 400)
# #     frame.pack() 
# #     panel = tk.Label(master = frame, image = img) 
# #     ranking = tk.Label(master = frame, text = ranking) 
# #     widgets.append((file_path,panel, ranking))
# #     panel.photo = img 
# #     panel.pack()
# #     ranking.pack() 

# # if __name__ == "__main__":
# #     if len(sys.argv) != 2:
# #         print("Missing path argument")
# #         exit() 
    
# #     folder = sys.argv[1] 
# #     file_path = "/home/scrc/PGENPO/MoFA/Data/Results/" + folder + "/encodings.json" 
# # # "/home/scrc/PGENPO/MoFA/Data/Results/1701730427.3845797/encodings.json"
# #     f = open(file_path) 
# #     data = json.load(f) 
# #     waypoint_count = len(data) 
# #     top_imgs = [] 
# #     rankings = [] 
# #     scores = {} 
# #     #For each encoding, match it against all the available encodings 
# #     for i in range(0,waypoint_count): 
# #         mappings = data[i] 
# #         keys = mappings.keys() 
# #         print("Filename: " + mappings['filename']) 
# #         score = mock_tree(mappings) 
# #         scores[mappings['filename']] = score 
# #         # for k in keys: 
# #         #     print("Key: " + str(k) + "\t" + str(type(mappings[k])) + "\t" + str(len(mappings[k])))
# #     #Get append to file names
# #     sorted_scores = sorted(scores.items(), key = lambda k: k[1])
# #     print("Sorted Scores: " + str(sorted_scores)) 
# #     #Display 
# #     display_top(sorted_scores)
# #     window.mainloop() 
    

