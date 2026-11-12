#!/usr/bin/env python3
"""Terra workflow observations only; Astra's mission client owns mission state.

This ledger does not create, reserve, send, cancel or resolve action UUIDs. It
does not determine visited waypoints or authorize execution. Client snapshots
and receipts are accepted only as hash-bound references; freshness and all
control decisions remain the client’s responsibility.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import sys
import time
import uuid


APP_ID = int.from_bytes(b"TRWF", "big")
SCHEMA = f"""
PRAGMA application_id={APP_ID};
PRAGMA user_version=2;
CREATE TABLE workflow (
  run_id TEXT PRIMARY KEY, created_ns INTEGER NOT NULL
);
CREATE TABLE events (
  sequence INTEGER PRIMARY KEY AUTOINCREMENT,
  event_id TEXT NOT NULL UNIQUE,
  run_id TEXT NOT NULL REFERENCES workflow(run_id),
  recorded_ns INTEGER NOT NULL,
  stage TEXT NOT NULL,
  status TEXT NOT NULL,
  payload_sha256 TEXT NOT NULL,
  payload_json TEXT NOT NULL
);
"""
NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")
STATE_FIELDS = {
    "mission_id", "revision", "identity_epoch_id", "variant", "lifecycle", "hold_reason",
    "identity", "route", "route_size", "policy", "progress", "current_target", "attempt", "attempts",
    "candidates", "budgets", "stop", "result_flags", "updated", "evidence",
}
OPERATION_FIELDS = {
    "operation", "request_id", "workflow_run_id", "operation_outcome", "exit_code",
    "mission_id", "revision", "attempt_id", "parent_action_uuid", "state_path",
    "state_sha256", "result_path", "result_sha256", "reason", "result_flags",
}
RESULT_FIELDS = {
    "mission_id", "revision", "lifecycle", "identity", "progress", "result_flags",
    "attempts", "budgets", "current_target", "last_attempt", "stop", "reason",
}


def name(value):
    if not isinstance(value, str) or not NAME.fullmatch(value):
        raise ValueError(f"invalid identifier: {value}")
    return value


def canonical_uuid(value):
    if not isinstance(value, str):
        raise ValueError("UUID must be a string")
    return str(uuid.UUID(value))


def revision(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**64:
        raise ValueError("client revision must be uint64")
    return value


def require_fields(obj, schema, required):
    if not isinstance(obj, dict) or obj.get("schema") != schema:
        raise ValueError(f"expected schema {schema}")
    missing = required - obj.keys()
    if missing:
        raise ValueError(f"{schema} missing fields: {sorted(missing)}")


def read_stable(path, *, include_data=False):
    path = Path(path).resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"evidence is not a file: {path}")
    before_path = path.stat()
    digest = hashlib.sha256()
    chunks = [] if include_data else None
    with path.open("rb") as stream:
        before_fd = os.fstat(stream.fileno())
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
            if chunks is not None:
                chunks.append(block)
        after_fd = os.fstat(stream.fileno())
    after_path = path.stat()
    identity = lambda st: (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns)
    if len({identity(before_path), identity(before_fd), identity(after_fd), identity(after_path)}) != 1:
        raise ValueError(f"evidence changed during read: {path}")
    return str(path), digest.hexdigest(), b"".join(chunks) if chunks is not None else None


def json_reference(path, schema, fields):
    filename, digest, data = read_stable(path, include_data=True)
    obj = json.loads(data)
    require_fields(obj, schema, fields)
    return {"path": filename, "sha256": digest}, obj


def check_matching(left, right, field, *, uuid_field=False):
    a, b = left.get(field), right.get(field)
    if a is not None and b is not None:
        if uuid_field:
            a, b = canonical_uuid(a), canonical_uuid(b)
        if a != b:
            raise ValueError(f"client {field} mismatch between references")


def client_refs(args, run_id):
    refs = {}
    state = operation = result = None
    if args.client_state:
        refs["state"], state = json_reference(args.client_state, "matrix.mission.state.v1", STATE_FIELDS)
        canonical_uuid(state["mission_id"])
        revision(state["revision"])
        if state["attempt"] is not None and not isinstance(state["attempt"], dict):
            raise ValueError("client state attempt must be an object or null")
        if state["attempt"] and state["attempt"].get("parent_action_uuid") is not None:
            canonical_uuid(state["attempt"]["parent_action_uuid"])
    if args.client_operation:
        refs["operation"], operation = json_reference(args.client_operation, "matrix.mission.operation.v1", OPERATION_FIELDS)
        if operation["workflow_run_id"] is not None and operation["workflow_run_id"] != run_id:
            raise ValueError("client operation belongs to a different workflow run")
        if operation["mission_id"] is not None:
            canonical_uuid(operation["mission_id"])
        if operation["revision"] is not None:
            revision(operation["revision"])
        if (operation["mission_id"] is None) != (operation["revision"] is None):
            raise ValueError("client mission ID and revision must both be null or present")
        if operation["parent_action_uuid"] is not None:
            canonical_uuid(operation["parent_action_uuid"])
        for key in ("state", "result"):
            path, digest = operation[key + "_path"], operation[key + "_sha256"]
            if (path is None) != (digest is None):
                raise ValueError(f"client {key} path and hash must both be null or present")
            if path is not None and (not isinstance(path, str) or not path):
                raise ValueError(f"client {key} path must be a nonempty string")
            if digest is not None and (not isinstance(digest, str) or not SHA256.fullmatch(digest)):
                raise ValueError(f"invalid client {key} SHA-256")
            if digest is not None and key not in refs and not (key == "result" and args.client_result):
                raise ValueError(f"client operation requires a hash-bound {key} copy")
        if state is not None:
            if operation["state_sha256"] != refs["state"]["sha256"]:
                raise ValueError("client state hash differs from operation")
            if operation["mission_id"] is None or operation["revision"] is None:
                raise ValueError("a hash-bound client state requires mission ID and revision")
            check_matching(state, operation, "mission_id", uuid_field=True)
            check_matching(state, operation, "revision")
            if state["attempt"] is not None:
                check_matching(state["attempt"], operation, "attempt_id")
                check_matching(state["attempt"], operation, "parent_action_uuid", uuid_field=True)
    if args.client_result:
        if operation is None:
            raise ValueError("result reference requires the operation that bound its hash")
        refs["result"], result = json_reference(args.client_result, "matrix.mission.result.v1", RESULT_FIELDS)
        if operation["result_sha256"] != refs["result"]["sha256"]:
            raise ValueError("client result hash differs from operation")
        if operation["mission_id"] is None or operation["revision"] is None:
            raise ValueError("a hash-bound client result requires mission ID and revision")
        canonical_uuid(result["mission_id"])
        revision(result["revision"])
        check_matching(result, operation, "mission_id", uuid_field=True)
        if result["last_attempt"] is not None:
            if not isinstance(result["last_attempt"], dict):
                raise ValueError("client result last_attempt must be an object or null")
            check_matching(result["last_attempt"], operation, "attempt_id")
            check_matching(result["last_attempt"], operation, "parent_action_uuid", uuid_field=True)
    source = state if state is not None else operation
    if source is not None:
        refs["mission_id"] = source.get("mission_id")
        refs["revision"] = source.get("revision")
        attempt = state.get("attempt") if state else None
        refs["attempt_id"] = (operation.get("attempt_id") if operation else None) or (attempt.get("attempt_id") if attempt else None)
        refs["parent_action_uuid"] = (operation.get("parent_action_uuid") if operation else None) or (attempt.get("parent_action_uuid") if attempt else None)
    return refs


def connect(path):
    if not Path(path).is_file():
        raise ValueError("workflow database does not exist")
    con = sqlite3.connect(path, timeout=10, isolation_level=None)
    con.row_factory = sqlite3.Row
    con.execute("PRAGMA foreign_keys=ON")
    con.execute("PRAGMA synchronous=FULL")
    if con.execute("PRAGMA application_id").fetchone()[0] != APP_ID or con.execute("PRAGMA user_version").fetchone()[0] != 2:
        con.close()
        raise ValueError("not a Terra workflow v2 database; refusing to write")
    return con


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    init = sub.add_parser("init"); init.add_argument("db"); init.add_argument("--workflow-run-id", required=True)
    event = sub.add_parser("event"); event.add_argument("db"); event.add_argument("--event-id", required=True); event.add_argument("--stage", required=True); event.add_argument("--status", choices=["STARTED", "OBSERVED", "COMPLETED", "BLOCKED", "CLOSED"], required=True); event.add_argument("--evidence", required=True); event.add_argument("--metadata-json", default="{}"); event.add_argument("--client-state"); event.add_argument("--client-operation"); event.add_argument("--client-result")
    show = sub.add_parser("show"); show.add_argument("db")
    args = parser.parse_args()
    if args.action == "init":
        run_id = name(args.workflow_run_id)
        fd = os.open(args.db, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        os.close(fd)
        con = sqlite3.connect(args.db)
        try:
            con.execute("PRAGMA synchronous=FULL")
            con.executescript(SCHEMA)
            con.execute("INSERT INTO workflow VALUES (?,?)", (run_id, time.time_ns()))
            con.commit()
        finally:
            con.close()
        print(json.dumps({"workflow_run_id": run_id, "initialized": str(Path(args.db).resolve()), "dispatch_authority": False}))
        return
    con = connect(args.db)
    if args.action == "show":
        try:
            out = {"workflow": dict(con.execute("SELECT * FROM workflow").fetchone()), "events": [dict(x) for x in con.execute("SELECT * FROM events ORDER BY sequence")], "dispatch_authority": False, "note": "References are historical observations, never permission to dispatch or proof of current revision."}
        finally:
            con.close()
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return
    event_id = canonical_uuid(args.event_id)
    stage = name(args.stage)
    metadata = json.loads(args.metadata_json)
    if not isinstance(metadata, dict):
        raise ValueError("metadata must be a JSON object")
    evidence_path, evidence_hash, _ = read_stable(args.evidence)
    run_id = con.execute("SELECT run_id FROM workflow").fetchone()[0]
    refs = client_refs(args, run_id)
    payload = {"stage": stage, "status": args.status, "evidence": {"path": evidence_path, "sha256": evidence_hash}, "metadata": metadata, "client_references": refs}
    payload_json = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
    payload_hash = hashlib.sha256(payload_json.encode()).hexdigest()
    con.execute("BEGIN IMMEDIATE")
    try:
        existing = con.execute("SELECT payload_sha256,sequence FROM events WHERE event_id=?", (event_id,)).fetchone()
        if existing:
            if existing["payload_sha256"] != payload_hash:
                raise ValueError("event ID reused with different content")
            sequence = existing["sequence"]
            duplicate = True
        else:
            if refs.get("mission_id") is not None and refs.get("revision") is not None:
                current_mission = canonical_uuid(refs["mission_id"])
                previous = [json.loads(row[0])["client_references"] for row in con.execute("SELECT payload_json FROM events")]
                revisions = [r["revision"] for r in previous if r.get("mission_id") is not None and canonical_uuid(r["mission_id"]) == current_mission and r.get("revision") is not None]
                if revisions and refs["revision"] < max(revisions):
                    raise ValueError("client revision regressed; record old evidence separately, never as current state")
            cursor = con.execute("INSERT INTO events(event_id,run_id,recorded_ns,stage,status,payload_sha256,payload_json) VALUES (?,?,?,?,?,?,?)", (event_id, run_id, time.time_ns(), stage, args.status, payload_hash, payload_json))
            sequence = cursor.lastrowid
            duplicate = False
        con.execute("COMMIT")
    except Exception:
        con.execute("ROLLBACK")
        raise
    finally:
        con.close()
    print(json.dumps({"sequence": sequence, "duplicate_event": duplicate, "client_references": refs, "dispatch_authority": False, "freshness_proven": False}, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, sqlite3.Error, json.JSONDecodeError) as exc:
        print(f"HOLD: {exc}", file=sys.stderr)
        sys.exit(1)
