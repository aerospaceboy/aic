import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

repo=Path.home()/'dev_ws/src/yzbot'
root=Path.home()/'dev_ws/delivery-20260908'
assert subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode != 0
jobs=[('final-b','Move 4 red cubes to zone B and 1 blue cube to zone A.',root/'run_b_phase_shift.sh'),
      ('final-c','Move 1 red cube to zone C and 4 blue cubes to zone A.',repo/'tools_demo/scripts/run_ordered_case.sh')]
for name,question,runner in jobs:
    print('START',name,flush=True)
    before=hashlib.sha256((repo/'mybot/pick_place_moveit.py').read_bytes()).hexdigest()
    with (root/(name+'-runner.log')).open('w') as out:
        result=subprocess.run(['bash',str(runner),question,str(root/name)],cwd=repo,env=dict(os.environ,RLC_TIMEOUT_SEC='600'),stdout=out,stderr=subprocess.STDOUT)
    latest=root/name/'latest-run.txt'
    summary={'run':name,'exit_code':result.returncode}
    if latest.exists():
        evidence=Path(latest.read_text().strip())
        (evidence/'arm-source.sha256').write_text(before+'  mybot/pick_place_moveit.py\n')
        if (evidence/'audit.json').exists():
            audit=json.loads((evidence/'audit.json').read_text())
            summary.update({k:audit.get(k) for k in ['motion_functional_pass','motion_under_300','elapsed_sec','minimum_clearance','dual_yield']})
    print(json.dumps(summary),flush=True)
    (root/(name+'-summary.json')).write_text(json.dumps(summary,indent=2))
    if not summary.get('motion_functional_pass'):
        print('STOP_ON_FAILED_ACCEPTANCE',flush=True)
        break
    time.sleep(3)
print('FINAL_QUEUE_FINISHED',flush=True)
