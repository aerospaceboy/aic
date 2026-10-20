"""Tests for the production LLM HTTP client (mocked transport)."""

import os

import pytest

from tools_demo.llm_client import LlmClient
from tools_demo.llm_client import LlmClientConfig
from tools_demo.llm_client import LlmClientError
from tools_demo.llm_client import normalize_endpoint


class FakeResponse:
    def __init__(self, status, body=None):
        self.status_code = status
        self._body = body

    def json(self):
        if self._body is None:
            raise ValueError("no body")
        return self._body


def completion_body(content="ok"):
    return {"choices": [{"message": {"content": content}}]}


def capture_client(responses, **overrides):
    calls = []

    def poster(url, json=None, headers=None, timeout=None):
        calls.append({"url": url, "json": json, "headers": headers,
                      "timeout": timeout})
        return responses.pop(0)

    poster_override = overrides.pop("poster", None)
    config = LlmClientConfig(
        base_url=overrides.pop("base_url", "http://llm:8081"),
        model=overrides.pop("model", "test-model"),
        **overrides,
    )
    client = LlmClient(
        config, poster=poster_override or poster, sleeper=lambda _s: None)
    return client, calls


def test_normalize_endpoint_accepts_all_provider_base_forms():
    assert normalize_endpoint(
        "http://llm:8081") == "http://llm:8081/v1/chat/completions"
    assert normalize_endpoint(
        "http://llm:8081/") == "http://llm:8081/v1/chat/completions"
    assert normalize_endpoint(
        "https://api.provider.com/v1"
    ) == "https://api.provider.com/v1/chat/completions"
    assert normalize_endpoint(
        "https://api.provider.com/v1/chat/completions"
    ) == "https://api.provider.com/v1/chat/completions"


def test_missing_key_env_fails_before_any_http_call():
    os.environ.pop("TEST_LLM_KEY_MISSING", None)
    client, calls = capture_client(
        [], api_key_env="TEST_LLM_KEY_MISSING")
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "auth"
    assert not calls


def test_auth_key_is_sent_and_never_logged_in_errors():
    os.environ["TEST_LLM_KEY"] = "supersecret"
    client, calls = capture_client([FakeResponse(200, completion_body())],
                                   api_key_env="TEST_LLM_KEY")
    content = client.complete({"model": "test-model"})
    assert content == "ok"
    assert calls[0]["headers"]["Authorization"] == "Bearer supersecret"
    del os.environ["TEST_LLM_KEY"]


def test_auth_rejection_fails_immediately_without_retry():
    client, calls = capture_client([FakeResponse(401)])
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "auth"
    assert len(calls) == 1


def test_server_error_retries_within_bound_then_fails():
    client, calls = capture_client(
        [FakeResponse(503), FakeResponse(503)], max_retries=1)
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "http"
    assert len(calls) == 2  # one retry, no unbounded hammering


def test_server_error_recovers_on_retry():
    client, calls = capture_client(
        [FakeResponse(500), FakeResponse(200, completion_body("plan"))],
        max_retries=1)
    assert client.complete({}) == "plan"
    assert len(calls) == 2


def test_malformed_body_is_a_protocol_error():
    client, _ = capture_client([FakeResponse(200, {"unexpected": True})])
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "protocol"


def test_connect_timeout_is_reported_as_timeout():
    import requests as requests_module

    def poster(url, json=None, headers=None, timeout=None):
        raise requests_module.Timeout("timed out")

    client, _ = capture_client([], poster=poster)
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "timeout"


def test_zero_budget_fails_before_any_http_call():
    client, calls = capture_client([], solve_budget_sec=0.0)
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "budget"
    assert not calls


def test_slow_transport_cannot_overrun_solve_deadline():
    import threading
    import time
    release = threading.Event()

    def poster(*args, **kwargs):
        release.wait(1.0)
        return FakeResponse(200, completion_body())

    client, _ = capture_client([], poster=poster, solve_budget_sec=0.08)
    start = time.monotonic()
    try:
        with pytest.raises(LlmClientError) as error:
            client.complete({})
        assert error.value.kind == "budget"
        assert time.monotonic() - start < 0.5
        with pytest.raises(LlmClientError) as busy:
            client.complete({})
        assert busy.value.kind == "busy"
    finally:
        release.set()


def test_transport_exception_text_cannot_disclose_credentials():
    import requests

    def poster(*args, **kwargs):
        raise requests.ConnectionError("secret-key-in-url")

    client, _ = capture_client([], poster=poster)
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert "secret-key" not in str(error.value)


@pytest.mark.parametrize("content", [None, {}, "", "   "])
def test_non_text_or_empty_model_content_is_rejected(content):
    client, _ = capture_client([FakeResponse(200, completion_body(content))])
    with pytest.raises(LlmClientError):
        client.complete({})


def test_truncated_completion_is_a_retryable_protocol_error():
    """finish_reason=length must be distinguishable from a broken response."""
    client, _ = capture_client([FakeResponse(200, {"choices": [
        {"message": {"content": "{\"items\":"}, "finish_reason": "length"}]})])
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "protocol"
    assert "truncated" in str(error.value)


def test_empty_completion_is_a_protocol_error():
    client, _ = capture_client([FakeResponse(200, {"choices": [
        {"message": {"content": "   "}, "finish_reason": "stop"}]})])
    with pytest.raises(LlmClientError) as error:
        client.complete({})
    assert error.value.kind == "protocol"


def test_shared_budget_shrinks_the_request_timeouts():
    """A caller can bound several samples with one phase budget."""
    client, calls = capture_client([FakeResponse(200, completion_body())],
                                   connect_timeout_sec=5.0,
                                   read_timeout_sec=30.0,
                                   solve_budget_sec=40.0)
    assert client.complete({}, budget_sec=5.0) == "ok"
    connect, read = calls[0]["timeout"]
    # Half of the shared 5 s budget each, not the 5 s/30 s config defaults.
    assert 2.0 < connect <= 2.5
    assert 2.0 < read <= 2.5


def test_zero_shared_budget_fails_before_any_http_call():
    client, calls = capture_client([], solve_budget_sec=40.0)
    with pytest.raises(LlmClientError) as error:
        client.complete({}, budget_sec=0.0)
    assert error.value.kind == "budget"
    assert calls == []


def test_busy_transport_is_retried_within_budget():
    """A timed-out predecessor must not fail the next click immediately."""
    client, _ = capture_client([], solve_budget_sec=5.0, max_retries=1,
                               retry_backoff_sec=0.01)
    calls = {"n": 0}

    def flaky(deadline, payload, headers, timeout):
        calls["n"] += 1
        if calls["n"] == 1:
            raise LlmClientError("llm transport still busy", kind="busy")
        return FakeResponse(200, completion_body())

    client._post_until = flaky
    assert client.complete({}) == "ok"
    assert calls["n"] == 2
