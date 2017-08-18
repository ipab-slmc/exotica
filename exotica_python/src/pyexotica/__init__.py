from _pyexotica import *

def setModel(prob, urdf, srdf):
    prob[1]['PlanningScene'][0][1]['URDF'] = urdf
    prob[1]['PlanningScene'][0][1]['SRDF'] = srdf
