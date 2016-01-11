#!/usr/bin/env python 

#      Author: Yiming Yang
# 
# Copyright (c) 2016, University Of Edinburgh 
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met: 
# 
# # Redistributions of source code must retain the above copyright notice, 
#    this list of conditions and the following disclaimer. 
# # Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the distribution. 
# # Neither the name of  nor the names of its contributors may be used to 
#    endorse or promote products derived from this software without specific 
#    prior written permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE. 

from lxml import etree
import sys
import os.path

def Help():
  print 'EXOTicise.py <package_path> <xml_config_path> reconfigures an OMPL MoveIt package into EXOTica package (use escape characters where necessary)'
  print 'e.g.\n rosrun exotica_moveit EXOTicise.py my_package "\$(find my_package)../resources/experiment.xml"'
  
def MoveGroup(file):
  xmlData = etree.parse(file)
  incl=xmlData.findall('include')
  mgroup=False
  for ii in incl:
    if ii.keys().count('ns')==1:
      if ii.attrib['ns']=='move_group':
        for c in ii.getchildren():
          if c.attrib['name']=='pipeline':
            c.set('value','exotica')
            mgroup=True
  if mgroup:
    xmlData.write(file)
    print 'Reconfigured "%s"' % (file)
    
def Pipeline(file,conf):
  outFile=file[0:-33]+'exotica_planning_pipeline.launch.xml'
  xmlData = etree.parse(file)
  arg=xmlData.findall('arg')
  pipe=0
  for a in arg:
    if a.attrib['name']=='planning_plugin':
      a.set('value','exotica_interface/EXOTicaPlanner')
      pipe=1
  param=xmlData.find('rosparam')
  if param.attrib['command']=='load':
    if param.attrib['file'][-21:]=='exotica_planning.yaml':
      pipe=pipe+1
    else: 
      param.set('file',param.attrib['file'][0:-18]+'exotica_planning.yaml')
      pipe=pipe+1
  par=xmlData.findall('param')
  found=False
  for p in par:
    if p.attrib['name']=='exotica_config':
      found=true
      pipe=pipe+1
      break
  if not found:
    r=etree.Element('param')
    r.set('name','exotica_config')
    r.set('value',conf)
    par[len(par)-1].addnext(r)
    pipe=pipe+1      
  if pipe==3:
    xmlData.write(outFile)
    print 'Reconfigured "%s", output written into "%s"' % (file, outFile)
  else:
    print 'Invalid xml structure (%s)' % file

def Package(file):
  xmlData = etree.parse(file)
  run=xmlData.findall('run_depend')
  found=False
  for r in run:
    if r.text=='exotica_moveit':
      found=True
      break
  if found:
    print 'Already reconfigured "%s"' % (file)
  else:
    r=etree.Element('run_depend')
    r.text='exotica_moveit'
    run[len(run)-1].addnext(r)
    xmlData.write(file)
    print 'Reconfigured "%s"' % (file)

def Config(file,conf):
  outFile=file[0:-18]+'exotica_planning.yaml'
  o=''
  # No move group configs or otherwise at the moment
  #with open(file) as f:
  #  content = f.readlines()  
  #for l in content:
  #  if l[0]!=' ' and l[0]!='\t' and l[0:15]!='planner_configs':
  #    o=o+l+'  exotica_config: '+conf+'\n'
  with open(outFile, 'w') as ff:
    ff.write(o)
  print 'Reconfigured "%s", output written into "%s"' % (file, outFile)

good=False
if len(sys.argv)!=3:
  print 'Invalid aguments'
  Help()  
else:
  inPackage=sys.argv[1]
  conf=sys.argv[2]
  files=[inPackage+'/launch/move_group.launch',inPackage+'/launch/ompl_planning_pipeline.launch.xml',inPackage+'/config/ompl_planning.yaml',inPackage+'/package.xml']
  good=True
  for f in files:
    if not os.path.isfile(f):
      good=False
      print 'File not found: "%s"' % f
      break
    
if good:
  MoveGroup(inPackage+'/launch/move_group.launch')
  Pipeline(inPackage+'/launch/ompl_planning_pipeline.launch.xml',conf)
  Package(inPackage+'/package.xml')
  Config(inPackage+'/config/ompl_planning.yaml',conf)
  print 'Good'

  
