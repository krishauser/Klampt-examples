from klampt.math.autodiff.pytorch import *
import klampt.math.autodiff.ad as ad
import torch,numpy as np

#torch to ad
vx = ad.var('x')
vy = ad.var('y')
vz = ad.var('z')
f = torch_to_ad(torch.nn.Sigmoid(),[vx+vy])/vz

vxi = np.array([3.0,4.0])/100
vyi = np.array([5.0,6.0])/100
vzi = np.array([7.0,8.0])/100
context = {'x':vxi,'y':vyi,'z':vzi}
ad.check_derivatives(f,context)

vzi = np.array([3.0,4.0])/100
vyi = np.array([5.0,6.0])/100
vxi = np.array([7.0,8.0])/100
context = {'x':vxi,'y':vyi,'z':vzi}
ad.check_derivatives(f,context)
print("torch-to-ad: vector-valued derivative check passed")

vxi = 3./100
vyi = 4./100
vzi = 5./100
context = {'x':vxi,'y':vyi,'z':vzi}
ad.check_derivatives(f,context)
    
vzi = 3./100
vyi = 4./100
vxi = 5./100
context = {'x':vxi,'y':vyi,'z':vzi}
ad.check_derivatives(f,context)
print("torch-to-ad: scalar-valued derivative check passed")

#ad to torch
f=(vx+vy)/vz
m = ad_to_torch(f,[vx,vy,vz])
m.check_derivatives_torch(f,[vx,vy,vz])
print("ad-to-torch: derivative check passed")