import os
import json

def split_and_rename_images(image_folder, json_file):
    # Load the json data from the file
    with open(json_file, "r") as f:
        data = json.load(f)

    # Loop over each entry in the json data
    for entry in data["images"]:
        # Get the relevant information from the entry
        folder_name = entry["source"]["folder_name"]
        frame_no = str(entry["frame_index"])
        file_name = entry["file_name"].replace(".png", ".jpg")

        # Create the destination folder if it does not exist
        destination_folder = os.path.join(image_folder, folder_name)
        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)

        # Move the file to the destination folder and rename it
        source_path = os.path.join(image_folder, file_name)
        destination_path = os.path.join(destination_folder, frame_no + ".jpg")
        os.rename(source_path, destination_path)

def main():
    # Path to the folder containing all the images
    image_folder = "/home/simenallum/Desktop/Compressed/train"

    # Path to the json file
    json_file = "/home/simenallum/Downloads/instances_train_objects_in_water.json"

    split_and_rename_images(image_folder, json_file)

if __name__ == "__main__":
    main()
