# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import argparse
import subprocess
import sumolib
from trailblazer.utils.path_utils import fixup_paths
from trailblazer.utils.path_utils import sub_path

# Class to kick off sumo in gui mode or command line mode given an input
# Set the working directory to your root data directory

class SUMO_exec(object):

  def get_additional_simulation_files(self, network_dir, routes_dir):
    routes = ''
    if (not os.path.isdir(routes_dir)):
      print('Could not find routes_dir : '+routes_dir)
      return routes

    for the_file in os.listdir(routes_dir):
      if the_file.endswith(".rou.xml"):
        routes += os.path.join(routes_dir, the_file) + ','

    if routes.endswith(','):
      routes = routes[:-1]
    return routes

  def start_sim(self, network_dir, routes_dir, timestep, duration):
    SUMO = sumolib.checkBinary('sumo')
    sumocfg = network_dir + '/osm.sumocfg'
    routes = self.get_additional_simulation_files(network_dir, routes_dir)
    args = [SUMO, '-c', sumocfg, '--additional-files', routes, '--step-length', str(timestep),
            '--fcd-output', routes_dir + '/osm.fcd.xml']
    if not duration is None:
      args = [SUMO, '-c', sumocfg, '--additional-files', routes, '--step-length', str(timestep),
              '--fcd-output', routes_dir + '/osm.fcd.xml', '-e', str(duration)]
    print("calling ", " ".join(args))
    subprocess.call(args)

  def start_gui(self, network_dir, routes_dir, timestep):
    SUMO = sumolib.checkBinary('sumo-gui')
    sumocfg = network_dir + '/osm.sumocfg'
    routes = self.get_additional_simulation_files(network_dir, routes_dir)
    args = [SUMO, '-c', sumocfg, '--additional-files', routes, '--step-length', str(timestep)]
    print("calling ", " ".join(args))
    subprocess.call(args)

def main():
  # THIS ASSUMES YOU HAVE A SUMO_HOME ENVIRONMENT VARIABLE SET TO THE ROOT OF THE SUMO DOWNLOAD
  parser = argparse.ArgumentParser(description='Start sumo with a network populated with a directory of route files.')
  parser.add_argument('--net_directory',
                      help='Directory path containing .net.xml file.',
                      type=str,
                      default='./net_quantico')
  parser.add_argument('--routes_directory',
                      help='Directory of ' \
                           'route files (e.g. *.rou.xml).',
                      type=str,
                      default="./routes")
  parser.add_argument('--timestep',
                      help='Timestep for simulation.',
                      type=float,
                      default=0.066666667)
  parser.add_argument('--density',
                      help='Traffic density [low,medium,high].',
                      type=str,
                      default='high')
  parser.add_argument('--run',
                      help='Run number.',
                      type=int,
                      default=0)

  args = parser.parse_args()
  network_dir, routes_dir = fixup_paths(args.net_directory, args.routes_directory)
  routes_dir = routes_dir + sub_path("traffic",args.run,args.density,args.run)
  starter = SUMO_exec()
  starter.start_gui(network_dir, routes_dir, args.timestep)

if __name__ == "__main__":
  main()