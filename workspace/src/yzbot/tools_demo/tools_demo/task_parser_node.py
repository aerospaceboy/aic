"""Parse competition questions with LLM validation and rule fallback."""

import base64
import binascii
import json
import time
import uuid
from typing import Optional

import rclpy
from rclpy.node import Node
from tools_demo.llm_client import LlmClient
from tools_demo.llm_client import LlmClientConfig
from tools_demo.llm_client import LlmClientError
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from std_msgs.msg import UInt32MultiArray

from tools_demo.task_protocol import parse_llm_response
from tools_demo.task_protocol import parse_task_question
from tools_demo.task_protocol import TaskPlan
from tools_demo.task_protocol import TaskProtocolError
from tools_demo.problem_solver import is_generator_problem
from tools_demo.problem_solver import problem_to_plan
from tools_demo.problem_solver import validate_problem_calculation
from tools_demo.task_question_node import task_qos
from tools_demo.task_question_node import input_chunk_qos


SYSTEM_PROMPT = """你是移动机器人分拣任务解析器。
只返回一个 JSON 对象，不要 Markdown、解释或额外字段。
格式：
{"schema_version":"1.0","items":[
  {"color":"red|blue","count":1,"target_zone":"A|B|C"}
]}
items 必须严格保持题目执行顺序，数字和区域必须来自题目。
"""


SYSTEM_PROMPT = """You extract ordered robot tasks from one question.
Return only one JSON object. No Markdown, explanation, or extra fields.
Map 红色 to "red" and 蓝色 to "blue". Color is one exact word.
Count is an integer. Zone is one exact uppercase letter A, B, or C.
The first spoken clause MUST be items[0]; the clause after 先/then MUST
remain before the clause after 再/then. Never sort or reverse the items.

Example input:
请先将3个蓝色方块送到A区，再将2个红色方块送到B区。
Example output:
{"schema_version":"1.0","items":[
{"color":"blue","count":3,"target_zone":"A"},
{"color":"red","count":2,"target_zone":"B"}]}

Example input:
任务：先搬运3个红色方块到C区，然后搬运2个蓝色方块到A区。
Example output:
{"schema_version":"1.0","items":[
{"color":"red","count":3,"target_zone":"C"},
{"color":"blue","count":2,"target_zone":"A"}]}

Now extract the user's question with exactly the same JSON shape.
Never emit placeholders such as "red|blue" or "A|B|C".
"""

PROBLEM_SYSTEM_PROMPT = """你是比赛出题程序的题目解题器。
先根据题目算出 x、y 两个变量的数值，再按照给定的映射规则生成抓取任务。
只返回一个 JSON 对象，不要 Markdown 或 JSON 之外的解释。
格式：
{"schema_version":"1.0",
 "calculation":{"people":[{"name":"姓名","x_goods":整数,"y_goods":整数}],
 "capacity":整数,"x_total":整数,"y_total":整数,"x":整数,"y":整数},
 "items":[
  {"color":"red|blue","count":N,"target_zone":"A|B|C"}
]}
items 必须保持 x 对应的颜色在前、y 对应的颜色在后；某颜色数量为 0 则省略该项。
count 必须是整数，来自你算出的 x、y。target_zone 必须是 A、B、C 之一。
解题规则：逐人累加每类物品需求；“甲、乙都+动作+N”表示两人各需要N，必须乘人数。
动作词可能变化（如想要、想玩、准备拿、希望得到），人物后的数量和物品才是需求依据。
不能因为动作词陌生而忽略该人物；双胞胎明确要求双倍时乘二。
“甲和乙需求一样，但少一件某物”是复制乙的两类需求后，仅减少指定物品。
先完整输出calculation.people，每个独立人物都要有两类需求，没有该类填0。
相同需求人物不能遗漏另一类物品；双倍仅作用于该人物和题目指定物品。
然后分别求x_total、y_total，x=ceil(x_total/capacity)，y同理；不能先忽略引用人物。
每个容器只能放一种物品，容器数量=该类总需求除以容量后向上取整。
先独立算出x、y，再严格应用原文的颜色和区域规则，最后按x、y顺序输出。
不得默认x是红色或y是蓝色。省赛三类规则必须区分：
1. x代表红色、y代表蓝色：颜色按变量，区域按各变量数值的门槛判断。
2. 数值>=3代表红色、否则蓝色，x去B、y去A：颜色按数值，区域按变量。
   x的项无论红蓝都去B，y的项无论红蓝都去A，不能套用按数值分区域的规则。
3. 较大数值代表蓝色、较小代表红色，蓝色去B、红色去C：先比较x和y再映射。
   x、y是物品变量，不预设哪个较大；先使用独立算出的数值进行比较。
   若x>y：x项为blue、count=x、target_zone=B；y项为red、count=y、target_zone=C。
   若x<y：x项为red、count=x、target_zone=C；y项为blue、count=y、target_zone=B。
   相等且规则未定义时不可猜测。禁止为符合某个颜色而交换x、y已经算出的数值。
映射只给每个变量指定颜色和区域，不改变该变量的数值；输出前逐项核对count。
不要把物品种类数当成容器数量，也不要直接猜测2和3；输出前核对容量是否足够。
"""

# Sent only when a previous sample failed validation. It names the checks that
# actually rejected answers on real rounds, without revealing the local
# deterministic result (the model must still solve independently).
PROBLEM_RETRY_PROMPT = """上一次输出未通过校验，请重新独立求解并重新输出完整 JSON。
提交前必须逐项自查：
1. calculation.people 覆盖题目中每一个出现需求的人，且每人都有 x_goods、y_goods 两项（没有的填 0）；
2. “甲、乙都+动作+N”是两人各 N，不能因动作词变化而漏人；“双胞胎…双倍”只作用于该人物和指定物品；
3. x_total 必须等于所有 x_goods 之和，y_total 必须等于所有 y_goods 之和（先求和再写总数）；
4. x = ceil(x_total/capacity)，y = ceil(y_total/capacity)，items 里的 count 必须与 x、y 完全一致；
5. color 必须来自原文映射规则，target_zone 必须是 A、B、C；数量为 0 的颜色省略该项。"""

# A chunk set that stays incomplete this long is reported to the operator.
PROBLEM_CHUNK_TTL_SEC = 30.0


def llm_error_is_retryable(exception: Exception) -> bool:
    """Only answer-quality failures are worth resampling.

    Transport, auth, config and budget failures would repeat unchanged, so they
    fail immediately (the client keeps its own bounded 429/5xx retries).
    """
    if isinstance(exception, LlmClientError):
        return exception.kind == "protocol"
    return True


class TaskParserNode(Node):
    """Convert natural-language questions into validated ROS task topics."""

    def __init__(self) -> None:
        super().__init__("competition_task_parser")
        self.declare_parameter("use_llm", True)
        self.declare_parameter("require_llm", True)
        self.declare_parameter(
            "base_url",
            "http://localhost:8081",
        )
        self.declare_parameter("model", "qwen2.5-coder")
        self.declare_parameter("llm_enable_thinking", False)
        self.declare_parameter("llm_max_tokens", 256)
        self.declare_parameter("timeout_sec", 45.0)
        # Hosted-provider settings: the key VALUE lives only in the
        # environment; config carries the variable name. LlmClientError
        # messages never contain secrets.
        self.declare_parameter("api_key_env", "")
        self.declare_parameter("connect_timeout_sec", 5.0)
        self.declare_parameter("read_timeout_sec", 30.0)
        self.declare_parameter("solve_budget_sec", 60.0)
        self.declare_parameter("llm_max_retries", 1)
        # Resamples of the same problem when the model answer fails validation
        # (self-inconsistent table, swapped plan). Transport retries stay in
        # llm_max_retries; this counts semantic attempts and is bounded by
        # solve_budget_sec inside the client.
        self.declare_parameter("llm_validate_attempts", 3)
        # Attempt 1 stays greedy (temperature 0) so the verified behaviour and
        # reproducibility are unchanged; a resample needs some diversity or a
        # deterministic wrong answer would simply be repeated.
        self.declare_parameter("llm_retry_temperature", 0.3)
        self.declare_parameter(
            "question_topic",
            "/competition/task/question",
        )
        self.declare_parameter(
            "plan_topic",
            "/competition/task/plan",
        )
        self.declare_parameter(
            "counts_topic",
            "/competition/task/counts",
        )
        self.declare_parameter(
            "order_topic",
            "/competition/task/order",
        )
        self.declare_parameter(
            "llm_status_topic",
            "/competition/task/llm_status",
        )
        self.declare_parameter(
            "problem_topic",
            "/competition/task/problem",
        )
        self.declare_parameter(
            "problem_chunk_topic",
            "/competition/task/problem_chunk",
        )

        self.use_llm = bool(self.get_parameter("use_llm").value)
        self.require_llm = bool(self.get_parameter("require_llm").value)
        self.base_url = str(self.get_parameter("base_url").value).rstrip("/")
        self.model = str(self.get_parameter("model").value)
        self.llm_enable_thinking = bool(
            self.get_parameter("llm_enable_thinking").value)
        self.llm_max_tokens = int(self.get_parameter("llm_max_tokens").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.llm_validate_attempts = max(
            1, int(self.get_parameter("llm_validate_attempts").value)
        )
        self.llm_retry_temperature = float(
            self.get_parameter("llm_retry_temperature").value
        )
        self.solve_budget_sec = float(
            self.get_parameter("solve_budget_sec").value
        )
        self.llm_client = LlmClient(
            LlmClientConfig(
                base_url=self.base_url,
                model=self.model,
                api_key_env=str(
                    self.get_parameter("api_key_env").value
                ),
                connect_timeout_sec=float(
                    self.get_parameter("connect_timeout_sec").value
                ),
                read_timeout_sec=float(
                    self.get_parameter("read_timeout_sec").value
                ),
                solve_budget_sec=float(
                    self.get_parameter("solve_budget_sec").value
                ),
                max_retries=int(self.get_parameter("llm_max_retries").value),
            )
        )
        self.last_question: Optional[str] = None
        self.get_logger().info(
            f"parser config use_llm={self.use_llm} "
            f"require_llm={self.require_llm} base_url={self.base_url}"
        )

        self.plan_publisher = self.create_publisher(
            String,
            str(self.get_parameter("plan_topic").value),
            task_qos(),
        )
        self.parse_result_publisher = self.create_publisher(
            String, "/competition/task/parse_result", task_qos())
        self.display_request = {}
        self.counts_publisher = self.create_publisher(
            UInt32MultiArray,
            str(self.get_parameter("counts_topic").value),
            task_qos(),
        )
        self.order_publisher = self.create_publisher(
            String,
            str(self.get_parameter("order_topic").value),
            task_qos(),
        )
        self.llm_status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("llm_status_topic").value),
            task_qos(),
        )
        self.problem_publisher = self.create_publisher(
            String,
            str(self.get_parameter("problem_topic").value),
            task_qos(),
        )
        self.subscription = self.create_subscription(
            String,
            str(self.get_parameter("question_topic").value),
            self.question_callback,
            task_qos(),
        )
        self.problem_subscription = self.create_subscription(
            String,
            str(self.get_parameter("problem_topic").value),
            self.problem_callback,
            task_qos(),
        )
        self.problem_chunk_subscription = self.create_subscription(
            String,
            str(self.get_parameter("problem_chunk_topic").value),
            self.problem_chunk_callback,
            input_chunk_qos(),
        )
        self.last_problem: Optional[str] = None
        self.problem_submissions = {}
        self._problem_chunks = {}
        self._last_successful_parse_at = 0.0
        # A lost chunk would otherwise leave the submission hanging with no
        # message to the operator; this timer reports the incomplete set.
        self.create_timer(2.0, self._expire_problem_chunks)

    def publish_llm_status(
        self,
        *,
        available: bool,
        source: str,
        latency_sec: float,
        detail: str,
    ) -> None:
        """Publish judge-visible proof of the model parsing attempt."""
        message = String()
        message.data = json.dumps(
            {
                "schema_version": "1.0",
                "required": self.require_llm,
                "available": bool(available),
                "source": source,
                "model": self.model,
                "base_url": self.base_url,
                "latency_sec": round(float(latency_sec), 3),
                "detail": detail,
            },
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.llm_status_publisher.publish(message)

    def request_llm(self, question: str) -> str:
        """Request one non-streaming OpenAI-compatible completion."""
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": question},
            ],
            "max_tokens": self.llm_max_tokens,
            "enable_thinking": self.llm_enable_thinking,
            "temperature": 0.0,
            "stream": False,
            "response_format": {"type": "json_object"},
        }
        return self.llm_client.complete(payload)

    def parse_question(self, question: str) -> TaskPlan:
        """Prefer a validated LLM plan and log every fallback reason."""
        started_at = time.monotonic()
        if self.use_llm:
            try:
                content = self.request_llm(question)
                plan = parse_llm_response(content, question)
                self.publish_llm_status(
                    available=True,
                    source="llm",
                    latency_sec=time.monotonic() - started_at,
                    detail="validated_model_plan",
                )
                return plan
            except (
                LlmClientError,
                json.JSONDecodeError,
                TaskProtocolError,
            ) as exception:
                self.publish_llm_status(
                    available=False,
                    source="error" if self.require_llm else "rules",
                    latency_sec=time.monotonic() - started_at,
                    detail=str(exception),
                )
                if self.require_llm:
                    raise TaskProtocolError(
                        "Required LLM parsing failed: "
                        f"{exception}"
                    ) from exception
                self.get_logger().warn(
                    "LLM parse unavailable or invalid; "
                    f"using rules: {exception}"
                )
        elif self.require_llm:
            self.publish_llm_status(
                available=False,
                source="error",
                latency_sec=0.0,
                detail="use_llm is false while require_llm is true",
            )
            raise TaskProtocolError(
                "Competition mode requires use_llm:=True"
            )
        else:
            self.publish_llm_status(
                available=False,
                source="rules",
                latency_sec=0.0,
                detail="model parsing explicitly disabled",
            )
        return parse_task_question(question, source="rules")

    def publish_plan(self, plan: TaskPlan) -> None:
        """Publish JSON, red-blue counts, and human-readable order."""
        plan_message = String()
        plan_message.data = plan.to_json()
        self.plan_publisher.publish(plan_message)

        counts_message = UInt32MultiArray()
        counts_message.layout.dim = [
            MultiArrayDimension(
                label="colors:red,blue",
                size=2,
                stride=2,
            )
        ]
        counts_message.data = [
            plan.counts["red"],
            plan.counts["blue"],
        ]
        self.counts_publisher.publish(counts_message)

        order_message = String()
        order_message.data = " | ".join(
            f"{index}:{item.color}x{item.count}->{item.target_zone}"
            for index, item in enumerate(plan.items, start=1)
        )
        self.order_publisher.publish(order_message)
        self.publish_parse_result("published", plan=plan)
        self.get_logger().info(
            f"task_id={plan.task_id} source={plan.source} "
            f"order={order_message.data}"
        )

    def begin_parse(self, question, mapping_rules="", task_id=None):
        """Associate display evidence with this exact submitted request."""
        self.display_request = {
            "request_id": task_id or uuid.uuid4().hex,
            "question": question, "mapping_rules": mapping_rules,
        }
        self.publish_parse_result("parsing")

    def publish_parse_result(self, state, *, plan=None, error=""):
        """Publish one atomic display snapshot after the command is sent."""
        publisher = getattr(self, "parse_result_publisher", None)
        if publisher is None:
            return
        payload = {
            **getattr(self, "display_request", {}),
            "schema_version": "1.0", "state": state,
            "model": self.model, "published": state == "published",
            "task_id": plan.task_id if plan else "",
            "source": plan.source if plan else "",
            "plan": plan.to_dict() if plan else None,
            "error": error,
        }
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def question_callback(self, message: String) -> None:
        """Parse and publish one previously unseen task question."""
        question = message.data.strip()
        if not question or question == self.last_question:
            return
        if is_generator_problem(question):
            # Generator word problems need mapping rules and are handled by
            # the /competition/task/problem flow; do not misparse them here.
            self.get_logger().info(
                "Detected generator word problem; awaiting mapping rules"
            )
            return
        self.begin_parse(question)
        try:
            self.publish_plan(self.parse_question(question))
            self.last_question = question
        except TaskProtocolError as exception:
            self.publish_parse_result("failed", error=str(exception))
            self.get_logger().error(f"Task question rejected: {exception}")

    def request_llm_problem(
        self,
        question: str,
        mapping_rules: str,
        deterministic: TaskPlan,
        *,
        attempt: int = 1,
        budget_sec: Optional[float] = None,
    ) -> str:
        """Ask the model to solve independently; validate its answer locally."""
        # Keep the candidate argument for the validation call contract, but
        # never transmit its computed counts or zones as the model's answer.
        messages = [
            {"role": "system", "content": PROBLEM_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": (
                    "映射规则：\n" + mapping_rules
                    + "\n\n题目：\n" + question
                ),
            },
        ]
        temperature = 0.0
        if attempt > 1:
            # Resample with a corrective instruction and some diversity: at
            # temperature 0 a retry could repeat the rejected answer verbatim.
            messages.append({"role": "user", "content": PROBLEM_RETRY_PROMPT})
            temperature = max(0.0, float(self.llm_retry_temperature))
        payload = {
            "model": self.model,
            "messages": messages,
            "max_tokens": self.llm_max_tokens,
            "enable_thinking": self.llm_enable_thinking,
            "temperature": temperature,
            "stream": False,
            "response_format": {"type": "json_object"},
        }
        return self.llm_client.complete(payload, budget_sec=budget_sec)

    def parse_problem(
        self,
        question: str,
        mapping_rules: str,
        task_id: Optional[str] = None,
        *,
        require_llm: Optional[bool] = None,
    ) -> TaskPlan:
        """
        Prefer a validated LLM solution, otherwise solve deterministically.

        The deterministic solver is the source of truth for the two official
        generator problems, so the LLM plan must agree with it exactly. The
        web problem flow respects require_llm, just like direct questions.
        Deterministic fallback is available only in explicit practice mode.
        """
        deterministic = problem_to_plan(
            question,
            mapping_rules,
            source="rules",
            task_id=task_id,
        )
        mandatory = self.require_llm if require_llm is None else require_llm
        started_at = time.monotonic()
        if self.use_llm:
            attempts = max(1, int(getattr(self, "llm_validate_attempts", 3)))
            phase_budget = float(getattr(self, "solve_budget_sec", 40.0))
            deadline = started_at + max(0.0, phase_budget)
            last_error: Optional[Exception] = None
            for attempt in range(1, attempts + 1):
                remaining = deadline - time.monotonic()
                if attempt > 1 and remaining <= 1.0:
                    last_error = TaskProtocolError(
                        "LLM parse budget exhausted before resampling"
                    )
                    break
                try:
                    content = self.request_llm_problem(
                        question,
                        mapping_rules,
                        deterministic,
                        attempt=attempt,
                        budget_sec=remaining,
                    )
                    plan = self._validate_problem_llm(
                        content,
                        question,
                        mapping_rules,
                        deterministic,
                    )
                except (
                    LlmClientError,
                    json.JSONDecodeError,
                    TaskProtocolError,
                ) as exception:
                    # A single bad model sample must not cost a competition
                    # round: an inconsistent model table or a swapped plan was
                    # observed on real rounds, so resample within one shared
                    # phase budget. Every accepted answer still has to match
                    # the deterministic solution exactly. Transport, auth,
                    # config and budget failures are not resampled (they would
                    # repeat unchanged); the client keeps its own 429/5xx
                    # retries.
                    last_error = exception
                    if (
                        llm_error_is_retryable(exception)
                        and attempt < attempts
                    ):
                        self.get_logger().warn(
                            "LLM problem parse attempt "
                            f"{attempt}/{attempts} rejected: {exception}; "
                            "resampling"
                        )
                        continue
                    break
                self.publish_llm_status(
                    available=True,
                    source="llm",
                    latency_sec=time.monotonic() - started_at,
                    detail="validated_model_problem_solution",
                )
                return plan
            self.publish_llm_status(
                available=False,
                source="error" if mandatory else "rules",
                latency_sec=time.monotonic() - started_at,
                detail=str(last_error),
            )
            if mandatory:
                raise TaskProtocolError(
                    "Required LLM problem parsing failed: "
                    f"{last_error}"
                ) from last_error
            self.get_logger().warn(
                "LLM problem parse unavailable or invalid; "
                f"using deterministic solver: {last_error}"
            )
        elif mandatory:
            self.publish_llm_status(
                available=False,
                source="error",
                latency_sec=0.0,
                detail="use_llm is false while require_llm is true",
            )
            raise TaskProtocolError(
                "Competition mode requires use_llm:=True"
            )
        else:
            self.publish_llm_status(
                available=False,
                source="rules",
                latency_sec=0.0,
                detail="deterministic problem solver",
            )
        return deterministic

    def _validate_problem_llm(
        self,
        content: str,
        question: str,
        mapping_rules: str,
        deterministic: TaskPlan,
    ) -> TaskPlan:
        """Validate the LLM solution against the deterministic plan."""
        from tools_demo.task_protocol import parse_llm_response
        from tools_demo.task_protocol import (  # noqa: PLC0415
            TaskProtocolError as ProtocolError,
        )

        # Arithmetic trace is only permitted on this word-problem path.
        # Direct command parsing keeps its original strict root schema.
        try:
            payload = json.loads(content)
        except json.JSONDecodeError:
            payload = None
        if isinstance(payload, dict) and "calculation" in payload:
            validate_problem_calculation(question, payload.pop("calculation"))
            content = json.dumps(payload)
        llm_plan = parse_llm_response(content, question)

        def item_key(item):
            return item.color, item.count, item.target_zone

        if sorted(llm_plan.items, key=item_key) != sorted(
            deterministic.items, key=item_key
        ):
            raise ProtocolError(
                "LLM problem plan disagrees with the deterministic solution: "
                f"expected={sorted(item_key(i) for i in deterministic.items)} "
                f"received={sorted(item_key(i) for i in llm_plan.items)}"
            )
        return TaskPlan(
            task_id=deterministic.task_id,
            source="llm",
            # Current scoring does not require model output color order.
            # Preserve every verified model item, only canonicalize ordering.
            items=tuple(sorted(
                llm_plan.items,
                key=lambda item: deterministic.items.index(item),
            )),
        )

    def problem_callback(self, message: String) -> None:
        """Solve one generator problem submitted from the web dashboard."""
        payload = None
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.begin_parse("")
            self.publish_parse_result("failed", error="题目消息不是合法JSON")
            self.get_logger().error("Problem payload is not valid JSON")
            return
        if not isinstance(payload, dict):
            self.begin_parse("")
            self.publish_parse_result("failed", error="题目消息必须为JSON对象")
            self.get_logger().error("Problem payload must be a JSON object")
            return
        question = str(payload.get("question", "")).strip()
        mapping_rules = str(payload.get("mapping_rules", "")).strip()
        task_id = payload.get("task_id")
        if not question:
            self.begin_parse("", mapping_rules, task_id)
            self.publish_parse_result("failed", error="题目不能为空")
            self.get_logger().error("Problem payload lacks a question")
            return
        # One confirm-click ID is one submission, including a failed parse.
        # A duplicate transport frame must not retry after the clock has
        # frozen as failed and then unexpectedly publish a recovered plan.
        # An explicit new click supplies a new ID and remains retryable.
        dedupe = (question, mapping_rules, task_id)
        submissions = getattr(self, "problem_submissions", {})
        if isinstance(task_id, str) and task_id and len(task_id) <= 128:
            if task_id in submissions:
                return
            submissions[task_id] = None
            if len(submissions) > 128:
                del submissions[next(iter(submissions))]
            self.problem_submissions = submissions
        if dedupe == self.last_problem:
            return
        self.begin_parse(question, mapping_rules, task_id)
        self.get_logger().info(
            f"problem received: {question} | rules: {mapping_rules}"
        )
        try:
            plan = self.parse_problem(
                question,
                mapping_rules,
                task_id,
            )
        except TaskProtocolError as exception:
            self.publish_parse_result("failed", error=str(exception))
            self.get_logger().error(
                f"Problem rejected: {exception}"
            )
            return
        self.publish_plan(plan)
        self._last_successful_parse_at = time.monotonic()
        self.last_problem = dedupe
        self.get_logger().info(
            f"problem task_id={plan.task_id} source={plan.source} "
            f"counts={plan.counts} order={plan.execution_order}"
        )

    def _expire_problem_chunks(self) -> None:
        """Report a submission whose chunks never completed.

        coStudio sends the problem in up to 128 chunks. If one is lost the set
        stays incomplete and no further callback runs, so without this timer
        the round would hang silently instead of telling the operator to
        resubmit.
        """
        now = time.monotonic()
        expired = [
            chunk_id
            for chunk_id, state in getattr(self, "_problem_chunks", {}).items()
            if now - state["received_at"] > PROBLEM_CHUNK_TTL_SEC
        ]
        for chunk_id in expired:
            state = self._problem_chunks.pop(chunk_id, None)
            if state is None:
                continue
            missing = sorted(set(range(state["total"])) - set(state["parts"]))
            self.get_logger().error(
                f"problem_chunks_incomplete id={chunk_id} "
                f"received={len(state['parts'])}/{state['total']} "
                f"missing={missing}"
            )
            # Never overwrite a state produced by a later successful parse.
            if state["received_at"] >= getattr(
                self, "_last_successful_parse_at", 0.0
            ):
                self.publish_parse_result(
                    "failed",
                    error=(
                        "题目分片不完整（"
                        f"{len(state['parts'])}/{state['total']}），请重新提交"
                    ),
                )

    def problem_chunk_callback(self, message: String) -> None:
        """Restore one bounded, UTF-8 coStudio manual-problem submission."""
        now = time.monotonic()
        self._problem_chunks = {
            chunk_id: state
            for chunk_id, state in self._problem_chunks.items()
            if now - state["received_at"] <= PROBLEM_CHUNK_TTL_SEC
        }
        try:
            envelope = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().error("Problem chunk is not valid JSON")
            return
        if not isinstance(envelope, dict) or envelope.get("v") != 1:
            self.get_logger().error("Problem chunk has an invalid envelope")
            return
        chunk_id = envelope.get("id")
        index = envelope.get("i")
        total = envelope.get("n")
        data = envelope.get("d")
        if (
            not isinstance(chunk_id, str)
            or not chunk_id
            or len(chunk_id) > 64
            or not isinstance(index, int)
            or not isinstance(total, int)
            or not isinstance(data, str)
            or index < 0
            or total < 1
            or total > 128
            or index >= total
            or len(data) > 96
        ):
            self.get_logger().error("Problem chunk fields are invalid")
            return
        state = self._problem_chunks.get(chunk_id)
        if state is None or state["total"] != total:
            state = {"total": total, "parts": {}, "received_at": now}
            self._problem_chunks[chunk_id] = state
        state["parts"][index] = data
        state["received_at"] = now
        if len(state["parts"]) != total:
            return
        encoded = "".join(state["parts"][part] for part in range(total))
        del self._problem_chunks[chunk_id]
        try:
            decoded = base64.b64decode(encoded, validate=True).decode("utf-8")
        except (binascii.Error, UnicodeDecodeError):
            self.get_logger().error("Problem chunks have invalid UTF-8 data")
            return
        if len(decoded) > 16_384:
            self.get_logger().error("Problem payload exceeds the size limit")
            return
        normalized = String(data=decoded)
        self.get_logger().info(
            f"problem_chunks_reassembled id={chunk_id} "
            f"chunks={total} bytes={len(decoded.encode('utf-8'))}"
        )
        self.problem_publisher.publish(normalized)
        self.problem_callback(normalized)


def main(args=None) -> None:
    """Run the task parser node."""
    rclpy.init(args=args)
    node = TaskParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
