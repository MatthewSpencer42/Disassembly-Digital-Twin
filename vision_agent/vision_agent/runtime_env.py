#!/usr/bin/env python3
import glob
import os
import sys


def _find_repo_root(current_path, target_name="agentic_disassembly"):
    curr = os.path.abspath(current_path)
    while curr != os.path.dirname(curr):
        if os.path.basename(curr) == target_name:
            return curr
        candidate = os.path.join(curr, target_name)
        if os.path.exists(candidate):
            return candidate
        curr = os.path.dirname(curr)
    return None


def _repo_has_vision_venv(path):
    return bool(
        path
        and os.path.isdir(path)
        and glob.glob(
            os.path.join(
                path,
                "vision_training",
                ".venv",
                "lib",
                "python*",
                "site-packages",
            )
        )
    )


def _candidate_repo_roots(current_path):
    env_root = os.environ.get("AGENTIC_DISASSEMBLY_ROOT", "").strip()
    if env_root:
        yield os.path.abspath(env_root)

    found = _find_repo_root(current_path)
    if found:
        yield found

    path = os.path.abspath(current_path)
    while path != os.path.dirname(path):
        if os.path.basename(path) == "disassembly_ws":
            yield os.path.join(path, "src", "agentic_disassembly")
        path = os.path.dirname(path)

    yield "/home/adip/workspace/disassembly_ws/src/agentic_disassembly"


def setup_python_env(current_path):
    sys.path = [p for p in sys.path if "/.local/lib/python" not in p]

    repo_root = None
    for candidate in _candidate_repo_roots(current_path):
        if _repo_has_vision_venv(candidate):
            repo_root = candidate
            break
    if repo_root is None:
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    venv_candidates = []
    for rel_path in (
        os.path.join("vision_training", ".venv"),
        os.path.join("vision_training", "train_vision_model", ".venv"),
    ):
        base = os.path.join(repo_root, rel_path, "lib")
        venv_candidates.extend(
            sorted(glob.glob(os.path.join(base, "python*", "site-packages")))
        )

    for path in venv_candidates:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            break

    os.environ["PYTHONNOUSERSITE"] = "1"
    os.environ["NO_ALBUMENTATIONS_UPDATE"] = "1"
    os.environ["TRANSFORMERS_VERBOSITY"] = "error"
    os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

    return repo_root
