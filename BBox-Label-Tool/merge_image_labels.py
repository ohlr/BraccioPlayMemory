import glob, os, shutil
from os import path
from shutil import copyfile
from pathlib import Path

def main():
    directory = path.join(os.getcwd(), 'Labels_merged')

    if os.path.exists(directory):
        shutil.rmtree(directory)
    os.makedirs(directory)

    for image in glob.glob('./Images/*/*.JPEG'):
        image_paths = image.split(os.sep)

        image_src = image_paths[len(image_paths)-1]
        image_dest = path.join(directory, image_src).replace(".JPEG", ".jpg")
        image_exists = Path(image_dest).is_file()

        category_id = image_paths[len(image_paths) - 2].replace(' ', '')
        labeled_file_src = path.join(os.getcwd(), 'Labels_out', category_id, image_src.replace('.JPEG', '.txt'))
        labeled_file_dst = path.join(directory, image_src.replace('.JPEG', '.txt'))

        if image_exists:
            print("Duplicate in " + labeled_file_src)
            # just append the txt code
            with open(labeled_file_dst, "a") as dst_file:
                with open(labeled_file_src, 'r') as src_file:
                    dst_file.write(src_file.read().replace("\n", "") + "\n")
        else:
            copyfile(image, image_dest)
            copyfile(labeled_file_src, labeled_file_dst)

if __name__ == "__main__":
    main()
