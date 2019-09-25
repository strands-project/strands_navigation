#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 12 20:21:38 2019

@author: gpdas

"""

import utils
import yaml
import sys


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print ("Usage: add_node_tags.py <in_file> <out_file> <config_file>")
    else:
        in_file = sys.argv[1]
        out_file = sys.argv[2]
        cfg_file = sys.argv[3]

        topo_map = utils.load_data_from_yaml(in_file)

        tags = utils.load_data_from_yaml(cfg_file)
        node_tag = {}
        for tag in tags:
            for node in tags[tag]:
                node_tag[node] = tag

        go_ahead = False
        while True:
            user_input = raw_input("This will add new tags as per the config file. Do you want to continue? Y/N: ")
            if user_input.lower() == "y" or user_input.lower() == "yes":
                go_ahead = True
                break
            elif user_input.lower() == "n" or user_input.lower() == "no":
                break
            else:
                print("Wrong input!")

        if go_ahead:
            for node in topo_map:
                if node["meta"]["node"] in node_tag:
                    if "tag" in node["meta"]:
                        if node_tag[node["meta"]["node"]] not in node["meta"]["tag"]:
                            node["meta"]["tag"].append(node_tag[node["meta"]["node"]])
                    else:
                        node["meta"]["tag"] = [node_tag[node["meta"]["node"]]]

            with open(out_file, 'w') as f_out:
                yaml.dump(topo_map, f_out, default_flow_style=False)
            print("Successfully created the new file with node tags")
