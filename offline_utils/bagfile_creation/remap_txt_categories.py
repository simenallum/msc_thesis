import os

# Define the mapping from old values to new values
mapping = {
          0: 0, 
          1: 0, 
          2: 1, 
          3: None,
          4: None,
          5: None
        }

# Define the path to the folder with the text files
folder_path = '/home/msccomputer/Desktop/SeaDroneSea/val_yolo/new_dir/labels'

# Loop through each file in the folder
for filename in os.listdir(folder_path):
    # Only process .txt files
    if filename.endswith('.txt'):
        file_path = os.path.join(folder_path, filename)
        with open(file_path, 'r') as file:
            # Read the lines of the file
            lines = file.readlines()
        with open(file_path, 'w') as file:
            # Write the remapped lines to the file
            for line in lines:
                parts = line.split()
                old_value = int(parts[0])
                new_value = mapping.get(old_value, None)
                if new_value is not None:
                    file.write('{} {}\n'.format(new_value, ' '.join(parts[1:])))