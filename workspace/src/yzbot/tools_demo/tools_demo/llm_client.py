"""Production HTTP client for OpenAI-compatible task-solving providers.

Pure module (no ROS imports) so the HTTP contract is unit-testable with an
injected poster. Responsibilities:

- endpoint normalisation: accept provider roots with or without ``/v1``
  (the historical config shipped ``http://host:8081`` and appended
  ``/v1/chat/completions`` itself; hosted providers commonly include
  ``/v1`` in the base URL, which produced ``/v1/v1/...``);
- authentication from an environment variable whose NAME is configured -
  the key value never enters config files, logs, or llm_status;
- separate connect/read timeouts bounded by one monotonic solve budget so
  retries can never multiply past the phase budget;
- bounded retries only for 429/5xx; 401/403/404 and protocol errors fail
  immediately with an explicit, secret-free reason.
"""

import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import requests

RETRYABLE_STATUS = frozenset({429, 500, 502, 503, 504})
AUTH_FAILURE_STATUS = frozenset({401, 403})


def normalize_endpoint(base_url: str) -> str:
    """Return the full chat-completions URL for any provider base form."""
    base = (base_url or "").strip().rstrip("/")
    if not base:
        raise LlmClientError("empty llm base_url", kind="config")
    if base.endswith("/chat/completions"):
        return base
    if base.endswith("/v1"):
        return base + "/chat/completions"
    return base + "/v1/chat/completions"


class LlmClientError(RuntimeError):
    """Explicit, secret-free LLM HTTP failure."""

    def __init__(self, message: str, kind: str = "protocol"):
        super().__init__(message)
        self.kind = kind


@dataclass
class LlmClientConfig:
    """Provider connection settings (never carries the key itself)."""

    base_url: str
    model: str
    api_key_env: str = ""
    connect_timeout_sec: float = 5.0
    read_timeout_sec: float = 30.0
    solve_budget_sec: float = 60.0
    max_retries: int = 1
    retry_backoff_sec: float = 1.0


class LlmClient:
    """One-shot chat-completions caller with budgeted retries."""

    def __init__(
        self,
        config: LlmClientConfig,
        poster: Optional[Callable] = None,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.endpoint = normalize_endpoint(config.base_url)
        self._post = poster or requests.post
        self._sleep = sleeper
        self._transport_slot = threading.Lock()

    def _post_until(self, deadline, payload, headers, timeout):
        """Bound caller wall time even if DNS or a trickling body stalls.

        A timed-out transport may finish in its daemon thread, but its late
        response is discarded. One slot prevents unbounded orphan requests.
        """
        if not self._transport_slot.acquire(blocking=False):
            raise LlmClientError("llm transport still busy", kind="busy")
        results = queue.Queue(maxsize=1)

        def send():
            try:
                response = self._post(
                    self.endpoint, json=payload, headers=headers,
                    timeout=timeout)
                if time.monotonic() >= deadline:
                    close = getattr(response, "close", None)
                    if close:
                        close()
                results.put((response, None))
            except Exception as exception:
                results.put((None, exception))
            finally:
                self._transport_slot.release()

        threading.Thread(target=send, daemon=True).start()
        try:
            response, error = results.get(
                timeout=max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            raise LlmClientError("llm solve deadline exceeded", kind="budget") from None
        if time.monotonic() >= deadline:
            raise LlmClientError("llm response arrived after deadline", kind="budget")
        if error is not None:
            # Transport exception strings may contain URLs or credentials.
            if isinstance(error, requests.Timeout):
                raise LlmClientError("llm request timed out", kind="timeout") from None
            raise LlmClientError("llm transport failed", kind="connection") from None
        return response

    def _auth_headers(self) -> Dict[str, str]:
        headers = {"Content-Type": "application/json"}
        env_name = self.config.api_key_env.strip()
        if env_name:
            key = os.environ.get(env_name, "")
            if not key:
                raise LlmClientError(
                    f"api key environment variable {env_name!r} is not set",
                    kind="auth",
                )
            headers["Authorization"] = f"Bearer {key}"
        return headers

    def complete(self, payload: Dict, budget_sec: Optional[float] = None) -> str:
        """POST one completion and return choices[0].message.content.

        budget_sec lets a caller share one phase budget across several samples
        (semantic retries) so they can never multiply past that phase.
        """
        started = time.monotonic()
        budget = self.config.solve_budget_sec
        if budget_sec is not None:
            budget = min(float(budget_sec), budget)
        deadline = started + max(0.0, budget)
        headers = self._auth_headers()
        attempts = 0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise LlmClientError(
                    "llm solve budget exhausted before a response",
                    kind="budget",
                )
            connect_timeout = min(self.config.connect_timeout_sec, remaining / 2.0)
            timeout = (connect_timeout,
                       min(self.config.read_timeout_sec, remaining - connect_timeout))
            attempts += 1
            try:
                response = self._post_until(deadline, payload, headers, timeout)
            except LlmClientError as exception:
                # A timed-out predecessor may still hold the single transport
                # slot. Wait one bounded backoff inside the budget instead of
                # failing the round immediately.
                if (
                    exception.kind == "busy"
                    and attempts <= self.config.max_retries
                    and time.monotonic() + self.config.retry_backoff_sec
                    < deadline
                ):
                    self._sleep(self.config.retry_backoff_sec)
                    continue
                raise
            except requests.Timeout as exception:
                raise LlmClientError(
                    "llm request timed out", kind="timeout"
                ) from None
            except requests.ConnectionError as exception:
                raise LlmClientError(
                    "llm endpoint unreachable",
                    kind="connection",
                ) from None
            status = response.status_code
            if status in AUTH_FAILURE_STATUS:
                raise LlmClientError(
                    f"llm provider rejected credentials (http {status})",
                    kind="auth",
                )
            if status == 404:
                raise LlmClientError(
                    "llm endpoint or model not found (http 404)",
                    kind="config",
                )
            if status in RETRYABLE_STATUS and attempts <= (
                self.config.max_retries
            ):
                if time.monotonic() + self.config.retry_backoff_sec \
                        >= deadline:
                    raise LlmClientError(
                        f"llm retryable http {status} but budget exhausted",
                        kind="budget",
                    )
                self._sleep(self.config.retry_backoff_sec)
                continue
            if status != 200:
                raise LlmClientError(
                    f"llm http {status}", kind="http"
                )
            break
        try:
            body = response.json()
        except ValueError as exception:
            raise LlmClientError(
                "llm http response body is not JSON", kind="protocol"
            ) from exception
        try:
            choice = body["choices"][0]
            content = choice["message"]["content"]
        except (KeyError, IndexError, TypeError) as exception:
            raise LlmClientError(
                "llm http response omitted choices[0].message.content",
                kind="protocol",
            ) from exception
        if choice.get("finish_reason") == "length":
            # Truncated by max_tokens; another sample can still be complete.
            raise LlmClientError(
                "llm completion truncated by max_tokens", kind="protocol"
            )
        if not isinstance(content, str) or not content.strip():
            raise LlmClientError(
                "llm completion content is empty", kind="protocol"
            )
        return content
