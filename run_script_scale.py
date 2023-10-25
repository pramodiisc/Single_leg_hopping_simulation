import argparse
import time
import os 
import xml.etree.ElementTree
import train_ga_library
import train_scale
parser = argparse.ArgumentParser()


parser.add_argument("-a", "--algorithm", help = "Type in algorithm to train on") # This will either be "cma" or "ga"
# parser.add_argument("-kp","--spring_foot",help="Type in the foot spring kp here") # Any numerical value

args = parser.parse_args()

if args.algorithm:
	algo=str(args.algorithm) 
else :
    algo=str("cma")

# if args.spring_foot:
#     kp=str(args.spring_foot)
# else :
#     kp=str("6000")

# os.chdir('./env/assets/')
# et = xml.etree.ElementTree.parse("single_leg.xml")

# new_tag = et.getroot()

# new_tag[5][3].attrib['kp']=kp 


# et.write("single_leg.xml")

# os.chdir('../../')
if algo=="ga":
    train_ga_library.run()
else :
    train_scale.run()


   
    

