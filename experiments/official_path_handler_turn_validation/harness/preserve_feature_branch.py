#!/usr/bin/env python3
"""Archive this isolated experiment without touching the canonical checkout/index."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
repo = Path(sys.argv[1]).resolve()
ref = "refs/heads/feature/official-path-handler-turn-validation"
prefix = "experiments/official_path_handler_turn_validation/"
receipt = root / "evidence/feature_branch_receipt.json"

def git(*args, data=None, env=None):
    return subprocess.check_output(["git", "-C", str(repo), *args], input=data,
                                   env=env).decode().strip()

before_head = git("rev-parse", "HEAD")
before_branch = git("symbolic-ref", "HEAD")
before_status = git("status", "--porcelain=v1", "--untracked-files=normal")
index = Path(git("rev-parse", "--path-format=absolute", "--git-path", "index"))
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
before_index = digest(index)
try:
    old = git("rev-parse", "--verify", ref)
except subprocess.CalledProcessError:
    old = ""
previous = json.loads(receipt.read_text()) if receipt.exists() else {}
if old and old != previous.get("commit"):
    raise SystemExit("Feature branch changed independently; refusing to overwrite it")
base = previous.get("base_commit", before_head)
files = []
for path in sorted(root.rglob("*")):
    if not path.is_file() or path == receipt:
        continue
    rel = path.relative_to(root)
    if rel.parts[0] in {"build", "runtime_home", "runtime_logs"} or "__pycache__" in rel.parts:
        continue
    if path.is_symlink():
        raise SystemExit(f"Unexpected symlink: {rel}")
    files.append((path, rel))

with tempfile.TemporaryDirectory(prefix="feature_snapshot_", dir=root / "runtime_home") as temp:
    env = os.environ.copy()
    env["GIT_INDEX_FILE"] = str(Path(temp) / "index")
    git("read-tree", base, env=env)
    additions = bytearray()
    for path, rel in files:
        blob = git("hash-object", "-w", "--stdin", data=path.read_bytes(), env=env)
        mode = "100755" if path.stat().st_mode & 0o111 else "100644"
        additions.extend(f"{mode} {blob}\t{prefix}{rel.as_posix()}\0".encode())
    git("update-index", "-z", "--index-info", data=bytes(additions), env=env)
    tree = git("write-tree", env=env)
    message = ("experiment: preserve isolated official path-interface validation\n\n"
               "Archive compatibility adapters, frozen upstream sources, rotation helper patch, "
               "local fixtures and evidence. Negative corridor and raw goal-yaw results are "
               "retained. This is not production integration, deployment or hardware acceptance.\n")
    commit = git("commit-tree", tree, "-p", old or base, data=message.encode(), env=env)
    git("update-ref", ref, commit, old or "0" * 40)

after_head = git("rev-parse", "HEAD")
after_branch = git("symbolic-ref", "HEAD")
after_index = digest(index)
report = {"branch": ref.removeprefix("refs/heads/"), "base_commit": base,
          "previous_snapshot": old or None, "commit": commit, "tree": tree,
          "archived_files": len(files), "repository": str(repo), "prefix": prefix,
          "canonical_head_before": before_head, "canonical_head_after": after_head,
          "canonical_branch_before": before_branch, "canonical_branch_after": after_branch,
          "canonical_index_sha256_before": before_index,
          "canonical_index_sha256_after": after_index,
          "canonical_head_and_index_unchanged":
              before_head == after_head and before_branch == after_branch and before_index == after_index,
          "status_before": before_status,
          "status_after": git("status", "--porcelain=v1", "--untracked-files=normal"),
          "pushed": False, "board_touched": False}
receipt.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: v for k, v in report.items() if not k.startswith("status_")}))
if not report["canonical_head_and_index_unchanged"]:
    raise SystemExit("Canonical state changed concurrently; snapshot retained, inspect receipt")
