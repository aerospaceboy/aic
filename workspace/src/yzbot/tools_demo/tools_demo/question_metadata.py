"""Retain supplied question identity without inventing an official seed."""

import re


def supplied_question_seed(request, original=None):
    """Return (seed, provenance), accepting only current-request evidence.

    A parser may omit auxiliary metadata. Recover it from the original input
    only when both its request ID and exact question match. A task ID, a hash,
    and our development generator's RNG configuration are not official seeds.
    """
    candidates = [request]
    original = original or {}
    identity = request.get('request_id') or request.get('task_id')
    if (identity and identity == (original.get('request_id') or original.get('task_id'))
            and request.get('question') == original.get('question')):
        candidates.append(original)
    for candidate in candidates:
        for key in ('question_seed', 'seed'):
            value = candidate.get(key)
            if isinstance(value, bool):
                continue
            if isinstance(value, int) and value >= 0:
                return str(value), 'supplied_metadata'
            if isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', value):
                if not value.startswith('-'):
                    return value, 'supplied_metadata'
    match = re.search(
        r'(?:题目种子|随机种子|种子|\bseed)\s*[:：=]\s*([A-Za-z0-9_.-]{1,128})',
        str(request.get('question', '')), re.IGNORECASE)
    if match and not match[1].startswith('-'):
        return match[1], 'question_text'
    return '', 'not_supplied'
