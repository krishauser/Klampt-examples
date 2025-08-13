from klampt.io import download_ycb
from klampt import vis 
objs = download_ycb.object_list()
print("Available YCB objects:", objs)
opts = download_ycb.options()
print("Available YCB options:", opts)
geom = download_ycb.load(objs[0])  #object [0] doesn't have a Google 16k model
vis.debug(geom)

geom = download_ycb.load(objs[1])  #object [1] has a Google 16k model
vis.debug(geom)

geom = download_ycb.load(objs[15])  #object [51] has a Google 16k model
vis.debug(geom)
