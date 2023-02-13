import os
import shutil

root_dir = '/home/simenallum/Desktop/Compressed/train'

for subdir, dirs, files in os.walk(root_dir):
    for file in files:
        if file.endswith('.jpg') or file.endswith('.jpeg') or file.endswith('.png'):
            shutil.move(os.path.join(subdir, file), os.path.join(root_dir, file))
