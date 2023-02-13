import json

def load_and_print(json_file):
    with open(json_file, 'r') as file:
        data_list = json.load(file)

    folder_names = {}
    for data in data_list["images"]:
        folder_name = data["source"]["folder_name"]
        if folder_name in folder_names:
            folder_names[folder_name] += 1
        else:
            folder_names[folder_name] = 1
            print(folder_name, data["source"]["drone"])

    print("\n")
    for folder_name, count in folder_names.items():
        print(f"{folder_name}: {count}")

    print("\n")

    drone_names = {}
    for data in data_list["images"]:
        d = data["source"]["drone"]
        if d in drone_names:
            drone_names[d] += 1
        else:
            drone_names[d] = 1
    for d, count in drone_names.items():
        print(f"{d}: {count}")

    print("\n")

    heights = {}
    for data in data_list["images"]:
        height = data["height"]
        if height in heights:
            heights[height] += 1
        else:
            heights[height] = 1
    for height, count in heights.items():
        print(f"{height}: {count}")

    widths = {}
    for data in data_list["images"]:
        w = data["width"]
        if w in widths:
            widths[w] += 1
        else:
            widths[w] = 1
    for w, count in widths.items():
        print(f"{w}: {count}")

def pretty(d, indent=0):
   for key, value in d.items():
      print('\t' * indent + str(key))
      if isinstance(value, dict):
         pretty(value, indent+1)
      else:
         print('\t' * (indent+1) + str(value))

def main():
    # Path to the json file
    json_file = "/home/simenallum/SeaDroneSea/instances_test_objects_in_water.json"

    load_and_print(json_file)

if __name__ == "__main__":
    main()