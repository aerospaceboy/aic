import json, math
from pathlib import Path
boxes=json.loads(Path('/tmp/audit_geometry.json').read_text())
paths={
 'blue5':[(9.26,-8.99),(9.65,-6.0),(7.2,-6.0),(4.6,-6.5),(3.1,-6.75),(2.55,-6.25),(-1.4,-4.5),(-1.4,-2.4),(-2.1,0),(-2.1,1.8),(-9.6,2)],
 'blue4':[(4.72,-11),(4.8,-11.65),(3.3,-11.65),(2.5,-11.1),(.8,-10.4),(-.1,-8.8),(0,-6),(-1.4,-4.5),(-1.4,-2.4),(-2.1,0),(-2.1,1.8),(-9.6,2)]}
def distance(x,y,b):
 p=b['pose']; a=p[5] if len(p)>5 else 0
 dx=x-p[0];dy=y-p[1];sx,sy=b['size'][:2]
 return math.hypot(max(abs(math.cos(a)*dx+math.sin(a)*dy)-sx/2,0),max(abs(-math.sin(a)*dx+math.cos(a)*dy)-sy/2,0))
for name,pts in paths.items():
 print(name)
 for a,b in zip(pts,pts[1:]):
  best=(100,None,None)
  n=max(1,int(math.dist(a,b)/.02))
  for k in range(n+1):
   x=a[0]+(b[0]-a[0])*k/n;y=a[1]+(b[1]-a[1])*k/n
   for box in boxes:
    d=distance(x,y,box)
    if d<best[0]:best=(d,box['name'],(round(x,2),round(y,2)))
  print(a,b,round(best[0],3),best[1:])
