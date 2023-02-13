import os
import shutil

def check_first_char_in_txt_files(folder_path):
    for filename in os.listdir(folder_path):
        if filename.endswith(".txt"):
            file_path = os.path.join(folder_path, filename)
            with open(file_path) as f:
                for line in f:
                    if line.startswith("5"):
                        print(f"{filename} starts with '5'")
                        break

def main():
  folder_path = "/home/msccomputer/Desktop/SeaDroneSea/val_yolo/new_dir/labels"

  check_first_char_in_txt_files(folder_path)

if __name__ == '__main__':
  main()