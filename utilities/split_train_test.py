import random
import os
from os.path import exists
import subprocess
import sys

def split_data_set(image_dir):

    f_val = open("data_test.txt", 'w')
    f_train = open("data_train.txt", 'w')
    
    path, dirs, files = next(os.walk(image_dir))
    data_size = len(files)

    ind = 0
    data_test_size = int(0.1 * data_size)
    test_array = random.sample(range(data_size), k=data_test_size)
    
    for f in os.listdir(image_dir):
        try:
            if(f.split(".")[2] == "png"):
                if exists(image_dir+'/'+f.split(".png")[0]+'.txt'):
                    ind += 1
                    if ind in test_array:
                        f_val.write(image_dir+'/'+f+'\n')
                    else:
                        f_train.write(image_dir+'/'+f+'\n')
                else:
                    print(f"Warning: No label file for image {f}")
        except:
            continue
            



split_data_set(sys.argv[1])
