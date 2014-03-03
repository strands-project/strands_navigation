#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import subprocess
import sys
from optparse import OptionParser
from time import sleep
import waypoint_patroller.log_util
import datetime
from dateutil import parser
import json

def create_sumary_file(datacentre_host, datacentre_port, jsonfile, start_time=None, end_time=None):
    """ create a json summary. if start_time and end_time are set, then all episodes in that
    window are sumarised, otherwise only the latest"""
    gen = waypoint_patroller.log_util.StatGenerator(datacentre_host, datacentre_port)
    if start_time is None:
        summary = gen.get_episode(gen.get_latest_run_name()).get_json_summary()
        with  open(jsonfile, "w") as f:
            f.write(summary)
    else:
        episodes = gen.get_episode_names_since(start_time, end_time)
        summaries = [gen.get_episode(i).get_summary() for i in episodes]
        with  open(jsonfile, "w") as f:
            f.write(json.dumps(summaries))

def upload_summary_scp(upload_path, server, user, password, jsonfile):
    call = subprocess.call(["sshpass", "-p", password, "scp", jsonfile, "%s@%s:%s"%(user,server,upload_path) ])
    if call != 0:
        raise Exception("Failed to upload summary. Bad call to sshpass...scp")
    
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("-s","--server",dest="hostname",
                      help="the ssh host to copy the file to")
    parser.add_option("-u","--username",dest="username",
                      help="username for account on ssh server")
    parser.add_option("-p","--password",dest="password", default="nopass",
                      help="the users password")
    parser.add_option("-j","--jsonfile",dest="jsonfile", default="/tmp/patrol_run.json",
                      help="the path of the temporary json file")
    parser.add_option("-f","--filepath",dest="path",
                      help="the location to place the file on the server.")
    parser.add_option("-t","--time-between",dest="timeout", type="int", default=300,
                      help="the time in seconds to pause between uploads to server. default = 300 seconds")

    parser.add_option("-d","--datacentre-host",dest="datacentre", default="localhost",
                      help="the machine that the datacentre(mongodb) is on")

    parser.add_option("-k","--datacentre-port",dest="datacentre_port",  type="int", default="62345",
                      help="the port that the datacentre(mongodb) is on")

    
    (options,args) = parser.parse_args()

    while True:
        try:
            create_sumary_file(options.datacentre, options.datacentre_port,options.jsonfile,
                               start_time=datetime.datetime.strptime('Nov 25 2013  09:40AM', '%b %d %Y %I:%M%p'))

            if options.hostname != None:
                upload_summary_scp(options.path, options.hostname, options.username, options.password,options.jsonfile)
            print "File done."
        except Exception, e:
            print "Upload failed: ", e
        sleep(options.timeout)
