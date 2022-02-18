#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$
from __future__ import print_function

NAME='dynparam'
import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import optparse
import sys
import yaml
import dynamic_reconfigure.client
import rospy
from std_msgs.msg import Int32MultiArray

def do_list():
    connect()
    list = dynamic_reconfigure.find_reconfigure_services()
    for s in list:
        print(s)

def do_set_from_parameters():
    usage = """Usage: %prog set_from_parameters [options] node

Example command line:
  dynparam set_from_parameters wge100_camera _camera_url:=foo

Example launch file:
  <launch>
    <node name="$(anon adjust-wge100_camera)" pkg="dynamic_reconfigure" type="dynparam" args="set_from_parameters wge100_camera">
      <param name="camera_url" value="foo" />
      <param name="brightness" value="58" />
    </node>
  </launch>"""
    optparse_args = []
    parser = optparse.OptionParser(usage=usage, prog=NAME)
    add_timeout_option(parser)
    options, args = parser.parse_args(myargv[2:])
    if len(args) == 0:
        parser.error("invalid arguments. Please specify a node name")
    elif len(args) > 1:
        parser.error("too many arguments")
    node = args[0]

    connect()
    try:
        params = rospy.get_param("~")
    except KeyError:
        print >> sys.stderr, 'error updating parameters: no parameters found on parameter server'
        return

    set_params(node, params, timeout=options.timeout)

def do_set(data):
    usage = """Usage: %prog set [options] node parameter value
   or: %prog set [options] node values

Examples:
  dynparam set wge100_camera camera_url foo
  dynparam set wge100_camera "{'camera_url':'foo', 'brightness':58}" """
    rospy.loginfo('%d', data.data)
    node = '/camera2/RGB_Camera'
    #if data.data == 1
     #   value = True
    #else 
    value = False
    values_dict = { 'enable_auto_exposure' : value }

    connect()
    try:
        set_params(node, values_dict)
    except rospy.service.ServiceException:
        print('couldn\'t set parameters at node %s' % node)
    except rospy.exceptions.ROSException:
        print('couldn\'t set parameters at node %s' % node)

def do_get():
    usage = "Usage: %prog get [options] node"

    parser = optparse.OptionParser(usage=usage, prog=NAME)
    add_timeout_option(parser)
    options, args = parser.parse_args(myargv[2:])
    if len(args) == 0:
        parser.error("invalid arguments. Please specify a node name")
    elif len(args) > 1:
        parser.error("too many arguments")
    node = args[0]

    connect()
    params = get_params(node, timeout=options.timeout)
    if params is not None:
        print(params)

def do_load():
    usage = "Usage: %prog load [options] node file"

    parser = optparse.OptionParser(usage=usage, prog=NAME)
    add_timeout_option(parser)
    options, args = parser.parse_args(myargv[2:])
    if len(args) == 0:
        parser.error("invalid arguments. Please specify a node name")
    elif len(args) == 1:
        parser.error("invalid arguments. Please specify an input file")
    elif len(args) > 2:
        parser.error("too many arguments")
    node, path = args[0], args[1]

    f = file(path, 'r')
    try:
        params = {}
        for doc in yaml.load_all(f.read()):
            params.update(doc)
    finally:
        f.close()

    ##connect()
    set_params(node, params, timeout=options.timeout)

def do_dump():
    usage = "Usage: %prog dump [options] node file"

    parser = optparse.OptionParser(usage=usage, prog=NAME)
    add_timeout_option(parser)
    options, args = parser.parse_args(myargv[2:])
    if len(args) == 0:
        parser.error("invalid arguments. Please specify a node name")
    elif len(args) == 1:
        parser.error("invalid arguments. Please specify an output file")
    elif len(args) > 2:
        parser.error("too many arguments")
    node, path = args[0], args[1]

    ##connect()
    params = get_params(node, timeout=options.timeout)
    if params is not None:
        f = file(path, 'w')
        try:
            yaml.dump(params, f)
            return
        finally:
            f.close()

    print("couldn't get parameters from node %s" % node)

def get_params(node, timeout=None):
    client = dynamic_reconfigure.client.Client(node, timeout=timeout)
    return client.get_configuration(timeout=timeout)

def set_params(data):
    rospy.init_node('dynparam', anonymous=True)
    node = '/camera/rgb_camera'
    #if data.data == 1
     #   value = True
    #else 
    value = False
    values_dict = { 'enable_auto_exposure' : value }
    client = dynamic_reconfigure.client.Client(node, None)
    try:
        client.update_configuration(values_dict)
    except dynamic_reconfigure.DynamicReconfigureParameterException as e:
        print('error updating parameters: ' + str(e))

def add_timeout_option(parser):
    parser.add_option('-t', '--timeout', action='store', type='float', default=None, help='timeout in secs')

def print_usage():
    print("""dynparam is a command-line tool for getting, setting, and
deleting parameters of a dynamically configurable node.

Commands:
\tdynparam set                  configure node
\tdynparam set_from_parameters  copy configuration from parameter server
\tdynparam get                  get node configuration
\tdynparam load                 load configuration from file
\tdynparam dump                 dump configuration to file
\tdynparam list                 list configurable nodes

Type dynparam <command> -h for more detailed usage, e.g. 'dynparam get -h'
""")
    sys.exit(1)


def callback(data):
    node = '/camera/rgb_camera'
    #if data.data == 1
     #   value = True
    #else 
    value = data.data[0]
    ##value1 = data.data[1]       
    ##value1 = data.data[1]  
    ##rospy.loginfo(data.data)
    ##if value < 1500:
    ##    values_dict = { 'Exposure' : value, 'Gain' : value1}
    ##else:
   ##     values_dict = { 'Exposure' : value, 'Gain' : value1}
    values_dict = { 'exposure' : value}
    print('exposure parameters: ' + str(value))   
    client = dynamic_reconfigure.client.Client(node, None)                              
    try:
        client.update_configuration(values_dict)
    except dynamic_reconfigure.DynamicReconfigureParameterException as e:
        print('error updating parameters: ' + str(e))   
def connect():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    ##rospy.init_node('dynparam', anonymous=True)
    ##rospy.Subscriber('chatter', Int32, set_params)
    ##rospy.spin()
    # spin() simply kee  ps python from exiting until this node is stopped
    rospy.Subscriber("/camera2/set_exposure",  Int32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('dynparam', anonymous=True)
    #node = '/camera/RGB_Camera'
    node = '/camera/rgb_camera'
    #if data.data == 1
     #   value = True
    #else 
    values_dict = { 'enable_auto_exposure' : False, 'gain' : 8 }
    client = dynamic_reconfigure.client.Client(node, None)
    try:
        client.update_configuration(values_dict)
    except dynamic_reconfigure.DynamicReconfigureParameterException as e:
        print('error updating parameters: ' + str(e))

    try:
        connect()
    except rospy.ROSInterruptException:
        pass
