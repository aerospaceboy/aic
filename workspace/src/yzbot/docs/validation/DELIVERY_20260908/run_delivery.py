import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

repo=Path.home()/'dev_ws/src/yzbot'
root=Path.home()/'dev_ws/delivery-20260908'
deadline=time.monotonic()+360
while not (root/'final-queue.log').exists() or 'FINAL_QUEUE_FINISHED' not in (root/'final-queue.log').read_text():
    assert time.monotonic()<deadline, 'Prior run did not finish'
    time.sleep(3)
assert subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode != 0
rejected=root/'rejected-anchor';rejected.mkdir(exist_ok=True)
for name in ['mybot/pick_place_moveit.py','mybot/test/test_pick_place_math.py']:
    shutil.copy2(repo/name,rejected/Path(name).name)
(rejected/'candidate.patch').write_bytes(subprocess.check_output(['git','diff','--','mybot/pick_place_moveit.py','mybot/test/test_pick_place_math.py'],cwd=repo))
shutil.copy2(root/'pick_place_moveit.before_anchor.py',repo/'mybot/pick_place_moveit.py')
test=repo/'mybot/test/test_pick_place_math.py'
source=test.read_text();marker='\n\ndef test_attachment_rebuilds_stable_but_shifted_joint'
assert marker in source
test.write_text(source.split(marker,1)[0]+'\n')
# The original test file already ended with a newline.
baseline=subprocess.check_output(['git','show','HEAD:mybot/test/test_pick_place_math.py'],cwd=repo)
assert test.read_bytes().rstrip()==baseline.rstrip()
test.write_bytes(baseline)
assert not subprocess.check_output(['git','diff','--','mybot'],cwd=repo).strip()
with (root/'delivery-build.log').open('w') as out:
    result=subprocess.run(['bash','-c','source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select mybot tools_demo'],cwd=Path.home()/'dev_ws',stdout=out,stderr=subprocess.STDOUT)
assert result.returncode==0
print('ANCHOR_CANDIDATE_REVERTED_AND_REBUILT',flush=True)
for name,question in [('delivery-b','Move 4 red cubes to zone B and 1 blue cube to zone A.'),('delivery-c','Move 1 red cube to zone C and 4 blue cubes to zone A.')]:
    print('START',name,flush=True)
    digest=hashlib.sha256((repo/'mybot/pick_place_moveit.py').read_bytes()).hexdigest()
    with (root/(name+'-runner.log')).open('w') as out:
        result=subprocess.run(['bash',str(repo/'tools_demo/scripts/run_ordered_case.sh'),question,str(root/name)],cwd=repo,env=dict(os.environ,RLC_TIMEOUT_SEC='600'),stdout=out,stderr=subprocess.STDOUT)
    latest=root/name/'latest-run.txt'
    summary={'run':name,'exit_code':result.returncode}
    if latest.exists():
        evidence=Path(latest.read_text().strip())
        (evidence/'arm-source.sha256').write_text(digest+'  mybot/pick_place_moveit.py\n')
        if (evidence/'audit.json').exists():
            audit=json.loads((evidence/'audit.json').read_text())
            summary.update({k:audit.get(k) for k in ['motion_functional_pass','motion_under_300','elapsed_sec','minimum_clearance','dual_yield']})
    print(json.dumps(summary),flush=True)
    (root/(name+'-summary.json')).write_text(json.dumps(summary,indent=2))
    time.sleep(3)
print('DELIVERY_QUEUE_FINISHED',flush=True)
