#!/usr/bin/env python

from pylab import *
import sys
from subprocess import run
from os.path import dirname

this_dir = dirname(__file__)

eps = 0.05 

high = 2.5
low = eps
pool = 20.
out = 34. 
stl = this_dir + '/base_air.stl'

if '-b' in sys.argv:
    stl = 'base_water.stl'
    high = 0.1
    low = -7.
    pool += eps
    out -= eps
dae = stl[:-3] + 'dae'
ive = stl[:-3] + 'ive'

pool /= 2
out /= 2
thick = out-pool
corner = -out + sqrt(2*out**2 - 4*out*pool + 4*sqrt(2)*thick*pool + 6*pool**2)

# generate overall surface
out_p = []
pool_p = []

for x,y in ((1,1),(1,-1),(-1,-1),(-1,1)):
    pool_p.append([x*pool, y*pool])
    out_p.append([x*out, y*corner])
    out_p.append([x*corner,y*out])
    
pool_p.sort(key = lambda p: arctan2(p[1],p[0]))
out_p.sort(key = lambda p: arctan2(p[1],p[0]))

# triangles
surf_t = []

for i in range(4):
    p1 = pool_p[i]
    p2 = pool_p[(i+1) % 4]
    o1 = out_p[2*i]
    o2 = out_p[2*i+1]
    o3 = out_p[(2*i+2) % 8]
    
    surf_t.append([p1,o1,o2])
    surf_t.append([p1,o2,o3])
    surf_t.append([p1,o3,p2])
    
def coords(p):
    if 'float' in str(type(p)):
        return f'{p:.5f}'.rstrip('0').rstrip('.')
    return f'{coords(p[0])} {coords(p[1])} {coords(p[2])}'
    
class Face:
    def __init__(self, p1, p2=None, p3=None):
        if p2 is None:
            p1,p2,p3 = p1
        self.p1 = array(p1)
        self.p2 = array(p2)
        self.p3 = array(p3)
        
    def meanZ(self):
        return (self.p1[2] + self.p2[2] + self.p2[2])/3
        
    def normal(self):
        n = cross(self.p2-self.p1, self.p3-self.p1)
        return n/norm(n)
    
    def stl(self):
        out = [f'facet normal {coords(self.normal())}','outer loop']
        for p in self.p1,self.p2,self.p3:
            out.append(f'vertex {coords(p)}')
        return out + ['endloop','endfacet']
    
    def coords(self, idx):
        return coords([self.p1,self.p2,self.p3][idx])
    
def sides_from(p):
    
    faces = []
    
    for i in range(len(p)):
        p1 = p[i]
        p2 = p[(i+1)% len(p)]          
        faces.append(Face(p1+[high],p2+[high],p1+[low]))
        faces.append(Face(p1+[low],p2+[high],p2+[low]))

    return faces

out_p.reverse()
faces = sides_from(pool_p) + sides_from(out_p)

for t in surf_t:
    faces.append(Face([p+[high] for p in t]))
    faces.append(Face([p+[low] for p in reversed(t)]))
    
out = ['solid base']
    
xyz = []
for face in faces:
    out += face.stl()
    for i in range(3):
        c = face.coords(i)
        if c not in xyz:
            xyz.append(c)

out.append('endsolid base')

with open(stl,'w') as f:
    f.write('\n'.join(out))
    
    
# update dae file
with open(dae) as f:
    dae_xml = f.read()
begin = dae_xml.find('count="72"')+11
end = begin + dae_xml[begin:].find('<')
with open(dae, 'w') as f:
    f.write(dae_xml[:begin] + ' '.join(xyz) + dae_xml[end:])

# update ive
run(['osgconv',dae,ive])
