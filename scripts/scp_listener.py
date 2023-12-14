#!/usr/bin/env python
import os
import time
import paramiko
import stat

# Configuration
remote_host = '192.168.1.145'
remote_port = 22  # SSH port (usually 22)
remote_username = 'recycling'
remote_password = 'keepondancing'
remote_source_folder = '/home/scazlab/catkin_ws/src/pgenpo/bags'
local_destination_folder = '/home/scrc/PGENPO/MoFA/Data/image_dir'
interval_seconds = 1  # Check for new files every 60 seconds

# Create SSH client
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

def get_most_recent_dir(sftp, path):
    """Return the most recently modified directory within the given path."""
    most_recent = None
    highest_ts = 0
    for item in sftp.listdir_attr(path):
        if stat.S_ISDIR(item.st_mode):
            if item.st_mtime > highest_ts:
                highest_ts = item.st_mtime
                most_recent = item.filename
    if most_recent:
        return os.path.join(path, most_recent)
    else:
        return None

def copy_structure(sftp, remote_path, local_path):
    """Recursively copy files and directories from remote to local."""
    for item in sftp.listdir(remote_path):
        remote_item_path = os.path.join(remote_path, item)
        local_item_path = os.path.join(local_path, item)

        file_info = sftp.stat(remote_item_path)
        if stat.S_ISDIR(file_info.st_mode):
            if not os.path.exists(local_item_path):
                os.makedirs(local_item_path)  # Ensure the local directory exists
            copy_structure(sftp, remote_item_path, local_item_path)
        else:
            if not os.path.exists(os.path.dirname(local_item_path)):
                os.makedirs(os.path.dirname(local_item_path))  # Create parent directory for the file
            sftp.get(remote_item_path, local_item_path)

try:
    ssh.connect(remote_host, port=remote_port, username=remote_username, password=remote_password)
    while True:
        with ssh.open_sftp() as sftp:
            if not os.path.exists(local_destination_folder):
                os.makedirs(local_destination_folder)

            most_recent_remote_dir = get_most_recent_dir(sftp, remote_source_folder)
            if most_recent_remote_dir:
                local_most_recent_dir = os.path.join(local_destination_folder, os.path.basename(most_recent_remote_dir))
                copy_structure(sftp, most_recent_remote_dir, local_most_recent_dir)

            print(f"Completed copying from '{most_recent_remote_dir}' to '{local_most_recent_dir}'")
        time.sleep(interval_seconds)

except paramiko.AuthenticationException:
    print("Authentication failed. Please check your credentials.")
except paramiko.SSHException as ssh_exception:
    print(f"SSH error: {ssh_exception}")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    ssh.close()
