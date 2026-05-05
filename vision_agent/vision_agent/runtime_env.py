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


def setup_python_env(current_path):
    sys.path = [p for p in sys.path if "/.local/lib/python" not in p]

    repo_root = _find_repo_root(current_path)
    if repo_root is None:
        repo_root = "/home/adip/workspace/disassembly_ws/src/agentic_disassembly"

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

    os.environ["NO_ALBUMENTATIONS_UPDATE"] = "1"
    os.environ["TRANSFORMERS_VERBOSITY"] = "error"
    os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

    return repo_root
