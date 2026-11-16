"""The HTTP submission must preserve official seed metadata through ROS input."""
from functools import partial
from http.server import ThreadingHTTPServer
import json
from threading import Thread
from types import SimpleNamespace
from urllib.request import Request, urlopen

from tools_demo.competition_dashboard import DashboardRequestHandler
from tools_demo.dashboard_state import DashboardState


def test_http_submission_forwards_seed_without_changing_question_or_rules():
    messages = []
    handler = partial(DashboardRequestHandler, state=DashboardState(), page=b'',
                      problem_publisher=SimpleNamespace(publish=messages.append))
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        body = {'task_id': 'seed-round', 'question': '完整题目',
                'mapping_rules': '完整规则', 'seed': '00123'}
        request = Request(f'http://127.0.0.1:{server.server_port}/api/task',
                          data=json.dumps(body).encode(), method='POST')
        with urlopen(request, timeout=3) as response:
            assert response.status == 200
        forwarded = json.loads(messages[0].data)
        assert forwarded['question_seed'] == '00123'
        for key in ('task_id', 'question', 'mapping_rules'):
            assert forwarded[key] == body[key]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
