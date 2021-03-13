# Distributed under the Apache License, Version 2.0.
# See accompanying NOTICE file for details.

import os
import argparse

import randomTrips  # noqa
import sumolib
from trailblazer.utils.path_utils import fixup_paths

# This code is based on SUMO/tools/osmWebWizard.py
class RouteBuilder(object):

    vehicleParameters = {
        "passenger":  ["--vehicle-class", "passenger",  "--prefix", "veh",   "--min-distance",
                       "300",  "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"],
        "truck":      ["--vehicle-class", "truck", "--prefix", "truck", "--min-distance",
                       "600",  "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"],
        "bus":        ["--vehicle-class", "bus",   "--prefix", "bus",   "--min-distance",
                       "600",  "--trip-attributes",                'departLane="best"', "--validate"],
        "delivery":   ["--vehicle-class", "delivery",  "--prefix", "dlv",   "--min-distance",
                       "500",  "--trip-attributes",                'departLane="best"', "--validate"],
        "motorcycle": ["--vehicle-class", "motorcycle", "--vclass", "motorcycle", "--prefix", "moto",  "--max-distance",
                       "1200", "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"],
        "bicycle":    ["--vehicle-class", "bicycle",    "--vclass", "bicycle",    "--prefix", "bike",  "--max-distance",
                       "8000", "--trip-attributes", 'speedDev="0.1" departLane="best"', "--validate"],
        "tram":       ["--vehicle-class", "tram",       "--vclass", "tram",       "--prefix", "tram",  "--min-distance",
                       "1200", "--trip-attributes",                'departLane="best"', "--validate"],
        "rail_urban": ["--vehicle-class", "rail_urban", "--vclass", "rail_urban", "--prefix", "urban", "--min-distance",
                       "1800", "--trip-attributes",                'departLane="best"', "--validate"],
        "rail":       ["--vehicle-class", "rail",       "--vclass", "rail",       "--prefix", "rail",  "--min-distance",
                       "2400", "--trip-attributes",                'departLane="best"', "--validate"],
        "ship":       ["--vehicle-class", "ship",       "--vclass", "ship",       "--prefix", "ship", "--validate"],
        "pedestrian": ["--vehicle-class", "pedestrian", "--pedestrians", "--prefix", "ped",   "--max-distance",
                       "2000", "--trip-attributes", 'speedDev="0.1"', ],
        "persontrips": ["--vehicle-class", "pedestrian", "--persontrips", "--prefix", "ped",
                        "--trip-attributes", 'speedDev="0.1" modes="public"', ],
    }

    # All lengths and widths in meters (http://sumo.dlr.de/wiki/Vehicle_Type_Parameter_Defaults)
    vehicle_specs = [ {'type':'passenger','length': 4.3, 'width':1.8},
                      {'type':'delivery','length': 6.5, 'width':2.16},
                      {'type':'truck','length': 7.1, 'width':2.4},
                      {'type':'bus','length': 12, 'width':2.5}]
    vehicle_densities = {
        'low': {'passenger': 7, 'truck': 3, 'delivery': 3, 'bus': 1},
        'medium': {'passenger': 9, 'truck': 5, 'delivery': 5, 'bus': 3},
        'high': {'passenger': 21, 'truck': 12, 'delivery': 11, 'bus': 9},
    }



    def __init__(self):
        self.files = {}

    def build_routes_from_densities(self, sumo_net, net_xml,
                                    routes_dir, densities, duration):
      self.additionalFiles = []
      self.routenames = []
      publicTransport = False;

      vehicles = {type_: {'count': count, 'fringeFactor': 15}
                  for type_, count in densities.items()}

      # This code is extracted from SUMO/tools/osmWebWizard.py lin 226
      # Note the pedestrian and publicTransport code is present but not supported/tested
      self.edges = sumo_net.getEdges()

      for vehicle, options in vehicles.items():
        print("Processing vehicle %s" % vehicle)

        self.files["route"] = "%sosm.%s.rou.xml" % (routes_dir, vehicle)
        self.files["trips"] = "%sosm.%s.trips.xml" % (routes_dir, vehicle)
        self.files["detour"] = "%sdetour.xml" % (routes_dir)

        try:
            options = self.parseTripOpts(net_xml, vehicle, options, duration, publicTransport)
        except ZeroDivisionError:
            continue

        #if vehicle == "pedestrian" and publicTransport:
        #    options += ["--additional-files", ",".join([self.files["stops"], self.files["ptroutes"]])]
        if os.path.isfile(self.files["detour"]):
            options += ["--additional-files", ",".join([self.files["detour"]])]
        randomTrips.main(randomTrips.get_options(options))

        # --validate is not called for pedestrians
        if vehicle == "pedestrian":
            self.routenames.append(self.files["route"])
        else:
            self.routenames.append(self.files["trips"])

    def build_routes(self, sumo_net, net_xml, routes_dir, density, duration):
        try:
            densities = self.vehicle_densities[density]
        except KeyError:
            print('Please provide a density of : \'low\' \'medium\' or \'high\'')
            return
        self.build_routes_from_densities(sumo_net, net_xml, routes_dir,
                                         densities, duration)

    # Copied from SUMO/tools/osmWebWizard.py
    def parseTripOpts(self, net_xml, vehicle, options, duration, publicTransport):
        "Return an option list for randomTrips.py for a given vehicle"
        print("Parsing trip options.")

        # calculate the total length of the available lanes
        length = 0.
        for edge in self.edges:
            if edge.allows(vehicle):
                length += edge.getLaneNumber() * edge.getLength()

        # converting the count into cars per hour per lane-meter
        period = 3600 / (length / 1000) / options["count"]

        opts = ["-n", net_xml,
                "--trip-attributes", "reroute=\"true\"",
                "--fringe-factor", options["fringeFactor"],
                "-p", period,
                "-r", self.files["route"],
                "-o", self.files["trips"],
                "-e", duration]
        if vehicle == "pedestrian" and publicTransport:
            opts += self.vehicleParameters["persontrips"]
        else:
            opts += self.vehicleParameters[vehicle]
        return opts

def main():
  parser = argparse.ArgumentParser(description='Generate Random routes for a SUMO network.')
  parser.add_argument('--net_directory',
                      help='Directory path containing .net.xml file.',
                      type=str,
                      default='./net_quantico/')
  parser.add_argument('--routes_directory',
                      help='Directory to generate ' \
                           'route files (e.g. *.rou.xml) to.',
                      type=str,
                      default="./routes/")
  parser.add_argument('--density',
                      help='Generate \'low\', \'medium\', or \'high\' traffic density',
                      type=str,
                      default='medium')
  parser.add_argument('--duration',
                      help='Time length of the simulation.',
                      type=float,
                      default=520)

  args = parser.parse_args()
  network_dir, output_dir = fixup_paths(args.net_directory, args.routes_directory)

  net_xml = network_dir + "osm.net.xml"
  sumo_net = sumolib.net.readNet(net_xml)
  if not sumo_net:
    print('Could not open network ' + net_xml)

  builder = RouteBuilder()
  builder.build_routes(sumo_net, net_xml, output_dir, args.density, args.duration)

if __name__ == "__main__":
  main()
