from flask import Flask, send_from_directory, jsonify, request
import os
from datetime import datetime
import csv

app = Flask(__name__)

def get_most_recent_dir(base_path):
    # List all directories in the base path
    dirs = [os.path.join(base_path, d) for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))]
    # Find the most recently modified directory
    most_recent_dir = max(dirs, key=os.path.getmtime)
    return most_recent_dir

@app.route('/images')
def list_images():
    global directory_counter
    global last_processed_directory

    # Use the most recent directory
    images_dir = get_most_recent_dir('/home/scrc/PGENPO/MoFA/Data/image_dir')

    # Check if the directory has changed
    if images_dir != last_processed_directory:
        directory_counter += 1
        last_processed_directory = images_dir
    
    # List all file names in the images directory
    images = list(sorted([f for f in os.listdir(images_dir) if os.path.isfile(os.path.join(images_dir, f)) and "jpg" in f], key=lambda k: int(os.path.basename(k).split('_')[1].split('.')[0]) ))

    print(directory_counter)

    return jsonify(images)

directory_counter = 0
last_processed_directory = None

@app.route('/rankings')
def list_rankings():
    global directory_counter

    # Use the most recent directory
    images_dir = get_most_recent_dir('/home/scrc/PGENPO/MoFA/Data/image_dir')

    # Display the rankings
    with open(os.path.join(images_dir, ''), 'r') as f:
        return jsonify(list(map(int,f.read().split('\n')[(directory_counter - 2) % 2].split(','))))

@app.route('/image/<filename>')
def serve_image(filename):
    # Use the most recent directory
    images_dir = get_most_recent_dir('/home/scrc/PGENPO/MoFA/Data/image_dir')
    image_path = os.path.join(images_dir, filename)
    if os.path.isfile(image_path):
        return send_from_directory(images_dir, filename)
    else:
        return "Image not found", 404

@app.route('/save-rankings', methods=['POST'])
def save_rankings():
    data = request.json
    file_path = './user_rankings.csv'
    
    images_dir = get_most_recent_dir('/home/scrc/PGENPO/MoFA/Data/image_dir')

    with open(file_path, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=['path', 'favorite', 'timestamp'])
        #writer.writeheader()
        for row in data:
            row['path'] = os.path.join(images_dir, row['path'])
            writer.writerow(row)

    return 'Rankings saved', 200

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=5000, debug=True)
