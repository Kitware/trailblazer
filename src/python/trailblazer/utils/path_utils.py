# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import ntpath
import shutil
import pathlib

def setup_paths(input, output):
  input,output = fixup_paths(input, output)
  clean_directory(output)
  return input,output

def sub_path(type, id, density, run):
  sub_dir = '/' + type + str(id) + '/' + density + '_density/run' + str(run) + '/'
  return sub_dir

def fixup_paths(root, relative):
  if root.startswith('.'):
    root = os.getcwd() + root[1:]
    root += '/'
  if relative.startswith('.'):
    relative = root + '/' + relative[1:]
    relative += '/'
  return root,relative

def clean_directory(path):
    if os.path.exists(path):
        for the_file in os.listdir(path):
          file_path = os.path.join(path, the_file)
          try:
            if os.path.isfile(file_path):
              os.unlink(file_path)
            elif os.path.isdir(file_path):
              shutil.rmtree(file_path)
          except Exception as e:
            print(e)
    else:
      pathlib.Path(path).mkdir(parents=True, exist_ok=True)

def base_layer_files(path, base_layer_file_list):
    # If we only have 1 tif and one metadata.txt file in our file_list, use those
    tifs = [x for x in base_layer_file_list if '.tif' in x ]
    metas = [x for x in base_layer_file_list if 'metadata.txt' in x]
    if len(tifs) == 1 and len(metas) == 1:
        base_layer_fname = tifs[0]
        base_layer_meta_fname = metas[0]
        return base_layer_fname,base_layer_meta_fname
    # Based on the path provided, find the base layer files to use
    gsd = None
    occlusion_level = None
    folder = ntpath.split(path)[1]
    # This should the sensor description
    settings = folder.split('_')
    for i in range(0, len(settings)):
        if settings[i] == 'oc':
            occlusion_level = settings[i + 1]
        if settings[i] == 'gsd':
            gsd = settings[i + 1]
    if gsd is None or occlusion_level is None:
        print("Path does not have gsd and/or occlusion level encoded in it " + path)
        return None,None
    # Make sure the base_layer files are there
    base_layer_fname = "base_layer_gsd=" + gsd + "_occ=" + occlusion_level + ".tif"
    base_layer_meta_fname = "base_layer_gsd=" + gsd + "_occ=" + occlusion_level + "_metadata.txt"
    if base_layer_fname not in base_layer_file_list or base_layer_meta_fname not in base_layer_file_list:
        print("Unable to find base layer associated with the gsd and occlusion level for " + path)
        return None,None
    return base_layer_fname,base_layer_meta_fname