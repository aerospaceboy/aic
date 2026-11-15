"""Explicit experimental scene selection; original layout remains recoverable."""
import json,os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def dense_enabled():
    value=os.environ.get('YZBOT_DENSE_SCENE','1').strip().lower()
    if value not in ('0','1','true','false','on','off'):
        raise ValueError('YZBOT_DENSE_SCENE must be 0/1, true/false or on/off')
    return value in ('1','true','on')


def load_dense_scene():
    p=Path(get_package_share_directory('bot_navigation'))/'config/dense_transport_scene.json'
    return json.loads(p.read_text())


def dense_topics():
    return ['/competition/obstacles/'+a['name'] for a in load_dense_scene()['actors']]
