"""Real-transport integration: the client against a local HTTP server.

V11 API checklist item: exercise the requests layer itself (headers,
timeouts, status handling) against a localhost test double instead of a
paid provider. The server answers /v1/chat/completions only.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer

import pytest

from tools_demo.llm_client import LlmClient
from tools_demo.llm_client import LlmClientConfig
from tools_demo.llm_client import LlmClientError
from tools_demo.llm_client import normalize_endpoint


class Handler(BaseHTTPRequestHandler):
    """Minimal OpenAI-compatible double with scripted statuses."""

    script = []  # list of (status, body-dict-or-None)
    seen_auth = []

    def do_POST(self):
        if self.path != "/v1/chat/completions":
            self.send_response(404)
            self.end_headers()
            return
        Handler.seen_auth.append(self.headers.get("Authorization"))
        if not Handler.script:
            self.send_response(500)
            self.end_headers()
            return
        status, body = Handler.script.pop(0)
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        if body is not None:
            self.wfile.write(json.dumps(body).encode("utf-8"))

    def log_message(self, *_args):
        pass


@pytest.fixture()
def llm_server():
    server = HTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    Handler.script = []
    Handler.seen_auth = []
    url = "http://127.0.0.1:%d" % server.server_port
    yield url
    server.shutdown()


def client_for(url):
    return LlmClient(
        LlmClientConfig(
            base_url=url,
            model="test-model",
            connect_timeout_sec=2.0,
            read_timeout_sec=5.0,
            solve_budget_sec=10.0,
            max_retries=0,
        )
    )


def test_round_trip_against_real_http(llm_server):
    Handler.script = [(200, {"choices": [{"message": {"content": "plan"}}]})]
    content = client_for(llm_server).complete({"model": "test-model"})
    assert content == "plan"
    assert Handler.seen_auth == [None]


def test_http_404_unknown_endpoint(llm_server):
    client = client_for(normalize_endpoint(llm_server) + "_wrong")
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "config"


def test_http_503_without_retry_budget(llm_server):
    Handler.script = [(503, None), (503, None)]
    with pytest.raises(LlmClientError) as error:
        client_for(llm_server).complete({})
    assert error.value.kind == "http"
