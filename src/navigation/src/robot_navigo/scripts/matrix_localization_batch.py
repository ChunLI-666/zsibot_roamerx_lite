#!/usr/bin/env python3
"""Manifest-driven Matrix mapping and Lightning localization regression runner."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml


SCHEMA = "robot_navigo.matrix_localization_suite"
SCHEMA_VERSION = 1
STAGES = ("map", "init", "tracking", "report")
FINAL_STATUSES = {
    "PASS",
    "METRIC_FAIL",
    "INFRA_FAIL",
    "TIMEOUT",
    "INVALID_INPUT",
    "SKIPPED",
}
REUSABLE_STATUSES = {"PASS", "SKIPPED"}


class ManifestError(ValueError):
    """The suite manifest cannot be executed safely."""


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    elapsed_sec: float
    timed_out: bool = False


PLACEHOLDER = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")
CASE_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def atomic_write_json(path: Path, data: Mapping[str, Any]) -> None:
    atomic_write_text(path, json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n")


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ManifestError(f"cannot read suite {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise ManifestError("suite root must be a mapping")
    return loaded


def parse_overrides(values: Sequence[str]) -> dict[str, str]:
    overrides: dict[str, str] = {}
    for item in values:
        if "=" not in item:
            raise ManifestError(f"--set expects KEY=VALUE, got {item!r}")
        key, value = item.split("=", 1)
        if not key:
            raise ManifestError("--set key must not be empty")
        overrides[key] = value
    return overrides


def resolve_variables(raw: Mapping[str, Any], overrides: Mapping[str, str]) -> dict[str, str]:
    variables = {str(key): str(value) for key, value in raw.items()}
    variables.update(overrides)
    for _ in range(10):
        changed = False
        for key, value in list(variables.items()):
            expanded = os.path.expandvars(value)
            expanded = PLACEHOLDER.sub(
                lambda match: variables.get(match.group(1), match.group(0)), expanded
            )
            if expanded != value:
                variables[key] = expanded
                changed = True
        if not changed:
            break
    return variables


def format_value(value: Any, context: Mapping[str, Any]) -> Any:
    if isinstance(value, str):
        expanded = os.path.expandvars(value)
        unknown = [match.group(1) for match in PLACEHOLDER.finditer(expanded) if match.group(1) not in context]
        if unknown:
            raise ManifestError(f"unknown placeholder {unknown[0]!r} in {value!r}")
        return PLACEHOLDER.sub(lambda match: str(context[match.group(1)]), expanded)
    if isinstance(value, list):
        return [format_value(item, context) for item in value]
    if isinstance(value, dict):
        return {key: format_value(item, context) for key, item in value.items()}
    return value


def case_context(
    context: Mapping[str, Any], raw_case: Mapping[str, Any], case_index: int
) -> dict[str, Any]:
    name = raw_case.get("name")
    if not isinstance(name, str) or not CASE_NAME.fullmatch(name):
        raise ManifestError(
            f"case name must match {CASE_NAME.pattern!r}, got {name!r}"
        )
    resolved = dict(context)
    resolved.update({
        "case": name,
        "case_index": case_index,
        "case_output": str(Path(str(context["stage_output"])) / "cases" / name),
    })
    variables = raw_case.get("variables", {})
    if not isinstance(variables, dict):
        raise ManifestError(f"case {name!r} variables must be a mapping")
    for key, value in variables.items():
        resolved[str(key)] = format_value(value, resolved)
    return resolved


def format_stage(stage: Mapping[str, Any], context: Mapping[str, Any]) -> dict[str, Any]:
    """Format a stage while giving every case its own deterministic context."""
    without_cases = {key: value for key, value in stage.items() if key != "cases"}
    formatted = format_value(without_cases, context)
    if "cases" in stage:
        formatted["cases"] = [
            format_value(raw_case, case_context(context, raw_case, index))
            for index, raw_case in enumerate(stage["cases"])
        ]
    return formatted


def validate_manifest(manifest: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("schema") != SCHEMA:
        errors.append(f"schema must be {SCHEMA!r}")
    if manifest.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version must be {SCHEMA_VERSION}")
    if not isinstance(manifest.get("name"), str) or not manifest.get("name"):
        errors.append("name must be a non-empty string")
    scenes = manifest.get("scenes")
    if not isinstance(scenes, list) or not scenes:
        errors.append("scenes must be a non-empty list")
        return errors

    seen_names: set[str] = set()
    seen_ids: set[int] = set()
    for index, scene in enumerate(scenes):
        prefix = f"scenes[{index}]"
        if not isinstance(scene, dict):
            errors.append(f"{prefix} must be a mapping")
            continue
        name = scene.get("name")
        scene_id = scene.get("scene_id")
        if not isinstance(name, str) or not name:
            errors.append(f"{prefix}.name must be a non-empty string")
        elif name in seen_names:
            errors.append(f"duplicate scene name: {name}")
        else:
            seen_names.add(name)
        if not isinstance(scene_id, int) or scene_id < 0:
            errors.append(f"{prefix}.scene_id must be a non-negative integer")
        elif scene_id in seen_ids:
            errors.append(f"duplicate scene_id: {scene_id}")
        else:
            seen_ids.add(scene_id)

        available = scene.get("available")
        enabled = scene.get("enabled")
        if not isinstance(available, bool) or not isinstance(enabled, bool):
            errors.append(f"{prefix}.available and enabled must be booleans")
            continue
        if enabled and not available:
            errors.append(f"{prefix} cannot be enabled while unavailable")
        if not available:
            if not isinstance(scene.get("reason"), str) or not scene.get("reason"):
                errors.append(f"{prefix}.reason is required for unavailable scenes")
            continue

        stages = scene.get("stages")
        if not isinstance(stages, dict):
            errors.append(f"{prefix}.stages must be a mapping")
            continue
        if enabled:
            missing = [stage for stage in STAGES if stage not in stages]
            if missing:
                errors.append(f"{prefix}.stages is missing {', '.join(missing)}")
        for stage_name, stage in stages.items():
            if stage_name not in STAGES:
                errors.append(f"{prefix}.stages has unsupported stage {stage_name!r}")
                continue
            if not isinstance(stage, dict):
                errors.append(f"{prefix}.stages.{stage_name} must be a mapping")
                continue
            timeout = stage.get("timeout_sec", manifest.get("defaults", {}).get("timeout_sec", 600))
            if not isinstance(timeout, (int, float)) or timeout <= 0:
                errors.append(f"{prefix}.stages.{stage_name}.timeout_sec must be positive")
            for policy in ("continue_on_failure", "always_run"):
                if policy in stage and not isinstance(stage[policy], bool):
                    errors.append(f"{prefix}.stages.{stage_name}.{policy} must be boolean")
            commands = stage.get("commands", [])
            cases = stage.get("cases", [])
            if commands and cases:
                errors.append(
                    f"{prefix}.stages.{stage_name} cannot define both commands and cases"
                )
            if stage_name != "report" and not commands and not cases:
                errors.append(
                    f"{prefix}.stages.{stage_name} requires non-empty commands or cases"
                )
            definitions: list[tuple[str, Any]] = [("", commands)]
            if cases:
                if not isinstance(cases, list):
                    errors.append(f"{prefix}.stages.{stage_name}.cases must be a list")
                    cases = []
                elif stage_name == "init" and len(cases) < 3:
                    errors.append(
                        f"{prefix}.stages.init.cases must contain at least three offsets"
                    )
                case_names: set[str] = set()
                definitions = []
                for case_index, case in enumerate(cases):
                    case_prefix = f".cases[{case_index}]"
                    if not isinstance(case, dict):
                        errors.append(f"{prefix}.stages.{stage_name}{case_prefix} must be a mapping")
                        continue
                    case_name = case.get("name")
                    if not isinstance(case_name, str) or not CASE_NAME.fullmatch(case_name):
                        errors.append(
                            f"{prefix}.stages.{stage_name}{case_prefix}.name is not path-safe"
                        )
                    elif case_name in case_names:
                        errors.append(
                            f"{prefix}.stages.{stage_name} has duplicate case {case_name!r}"
                        )
                    else:
                        case_names.add(case_name)
                    definitions.append((case_prefix, case.get("commands", [])))
            for definition_prefix, definition_commands in definitions:
                if not isinstance(definition_commands, list) or (
                    stage_name != "report" and not definition_commands
                ):
                    errors.append(
                        f"{prefix}.stages.{stage_name}{definition_prefix}.commands "
                        "must be a non-empty list"
                    )
                    continue
                for command_index, command in enumerate(definition_commands):
                    argv = command.get("argv") if isinstance(command, dict) else None
                    if not isinstance(argv, list) or not argv or not all(isinstance(arg, str) for arg in argv):
                        errors.append(
                            f"{prefix}.stages.{stage_name}{definition_prefix}.commands"
                            f"[{command_index}].argv must be a non-empty string list"
                        )
            dependencies = stage.get("depends_on", [])
            if not isinstance(dependencies, list) or any(item not in STAGES for item in dependencies):
                errors.append(
                    f"{prefix}.stages.{stage_name}.depends_on must contain only supported stages"
                )
            elif stage_name in dependencies:
                errors.append(f"{prefix}.stages.{stage_name} cannot depend on itself")
    return errors


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def path_signature(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise ManifestError(f"signature input does not exist: {path}")
    if path.is_file():
        return {"path": str(path.resolve()), "kind": "file", "sha256": file_digest(path)}
    if not path.is_dir():
        raise ManifestError(f"unsupported signature input: {path}")
    digest = hashlib.sha256()
    count = 0
    for child in sorted(item for item in path.rglob("*") if item.is_file()):
        stat = child.stat()
        relative = child.relative_to(path).as_posix()
        digest.update(f"{relative}\0{stat.st_size}\0".encode())
        digest.update(file_digest(child).encode())
        digest.update(b"\n")
        count += 1
    return {
        "path": str(path.resolve()),
        "kind": "directory",
        "file_count": count,
        "sha256": digest.hexdigest(),
    }


def build_input_signature(
    scene_name: str,
    stage_name: str,
    stage: Mapping[str, Any],
    context: Mapping[str, Any],
) -> tuple[str, list[dict[str, Any]]]:
    formatted_stage = format_stage(stage, context)
    inputs: list[dict[str, Any]] = []
    for raw in formatted_stage.get("signature_inputs", []):
        if isinstance(raw, str):
            path = Path(raw)
        elif isinstance(raw, dict) and isinstance(raw.get("path"), str):
            path = Path(raw["path"])
        else:
            raise ManifestError(f"{scene_name}.{stage_name} has invalid signature input {raw!r}")
        inputs.append(path_signature(path))
    inputs.append({
        "kind": "runtime",
        "hostname": platform.node(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "kernel": platform.release(),
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
    })
    payload = {
        "schema_version": SCHEMA_VERSION,
        "scene": scene_name,
        "stage": stage_name,
        "stage_definition": formatted_stage,
        "inputs": inputs,
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest(), inputs


def json_path(data: Any, path: str) -> Any:
    current = data
    if not path:
        return current
    for component in path.split("."):
        if isinstance(current, dict) and component in current:
            current = current[component]
        elif isinstance(current, list) and component.isdigit() and int(component) < len(current):
            current = current[int(component)]
        else:
            raise KeyError(path)
    return current


def compare(actual: Any, operator: str, expected: Any) -> bool:
    operations = {
        "report": lambda: True,
        "eq": lambda: actual == expected,
        "ne": lambda: actual != expected,
        "lt": lambda: actual < expected,
        "le": lambda: actual <= expected,
        "gt": lambda: actual > expected,
        "ge": lambda: actual >= expected,
        "in": lambda: actual in expected,
        "not_in": lambda: actual not in expected,
    }
    if operator not in operations:
        raise ManifestError(f"unsupported metric operator: {operator}")
    try:
        return bool(operations[operator]())
    except (TypeError, ValueError) as exc:
        raise ManifestError(
            f"cannot compare {actual!r} {operator} {expected!r}: {exc}"
        ) from exc


def evaluate_metrics(metrics: Iterable[Mapping[str, Any]], context: Mapping[str, Any]) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    cache: dict[Path, Any] = {}
    for raw_metric in metrics:
        metric = format_value(dict(raw_metric), context)
        source = Path(metric["file"])
        if not source.is_file():
            raise ManifestError(f"metric file does not exist: {source}")
        if source not in cache:
            try:
                cache[source] = json.loads(source.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ManifestError(f"cannot read metric file {source}: {exc}") from exc
        path = str(metric.get("path", ""))
        try:
            actual = json_path(cache[source], path)
        except KeyError as exc:
            if metric.get("optional", False):
                actual = None
            else:
                raise ManifestError(f"metric path {path!r} is absent from {source}") from exc
        operator = str(metric.get("op", "eq"))
        expected = metric.get("value")
        passed = compare(actual, operator, expected)
        results.append({
            "name": metric.get("name", path or source.name),
            "file": str(source),
            "path": path,
            "operator": operator,
            "expected": expected,
            "actual": actual,
            "passed": passed,
        })
    return results


def run_command(
    argv: Sequence[str],
    cwd: Path,
    environment: Mapping[str, str],
    timeout_sec: float,
    log_path: Path,
) -> CommandResult:
    started = time.monotonic()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write(f"command: {json.dumps(list(argv), ensure_ascii=False)}\n")
        log.write(f"cwd: {cwd}\n")
        log.flush()
        process = subprocess.Popen(
            list(argv),
            cwd=cwd,
            env=dict(environment),
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        try:
            returncode = process.wait(timeout=timeout_sec)
            return CommandResult(returncode, time.monotonic() - started)
        except subprocess.TimeoutExpired:
            terminate_process_group(process)
            log.write(f"\nTIMEOUT after {timeout_sec:.3f}s\n")
            return CommandResult(process.returncode or -signal.SIGTERM, time.monotonic() - started, True)
        except BaseException:
            terminate_process_group(process)
            raise


def terminate_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def status_record(
    scene: Mapping[str, Any],
    stage_name: str,
    status: str,
    reason: str,
    started_at: str,
    elapsed_sec: float,
    signature: str | None = None,
    signature_inputs: Sequence[Mapping[str, Any]] = (),
    commands: Sequence[Mapping[str, Any]] = (),
    metrics: Sequence[Mapping[str, Any]] = (),
    cases: Sequence[Mapping[str, Any]] = (),
    resumed: bool = False,
) -> dict[str, Any]:
    if status not in FINAL_STATUSES:
        raise ValueError(f"invalid final status: {status}")
    return {
        "schema": "robot_navigo.matrix_localization_stage_status",
        "schema_version": 1,
        "scene": scene["name"],
        "scene_id": scene["scene_id"],
        "available": scene.get("available"),
        "enabled": scene.get("enabled"),
        "stage": stage_name,
        "status": status,
        "reason": reason,
        "started_at_utc": started_at,
        "ended_at_utc": utc_now(),
        "elapsed_sec": elapsed_sec,
        "input_signature": signature,
        "signature_inputs": list(signature_inputs),
        "commands": list(commands),
        "metrics": list(metrics),
        "cases": list(cases),
        "resumed": resumed,
    }


def read_status(path: Path) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def archive_stage_output(stage_output: Path) -> Path | None:
    if not stage_output.exists():
        return None
    archive_root = stage_output.parent / ".stale"
    archive_root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    destination = archive_root / f"{stage_output.name}_{timestamp}"
    stage_output.rename(destination)
    return destination


def stage_context(
    variables: Mapping[str, str],
    run_dir: Path,
    scene: Mapping[str, Any],
    stage_name: str,
) -> dict[str, Any]:
    scene_dir = run_dir / str(scene["name"])
    stage_output = scene_dir / stage_name
    context: dict[str, Any] = dict(variables)
    context.update({
        "run_dir": str(run_dir),
        "scene": scene["name"],
        "scene_id": scene["scene_id"],
        "scene_dir": str(scene_dir),
        "stage": stage_name,
        "stage_output": str(stage_output),
        "map_output": str(scene_dir / "map" / "map"),
    })
    inputs = scene.get("inputs", {})
    if isinstance(inputs, dict):
        for key, value in inputs.items():
            context[key] = format_value(value, context)
    return context


def result_status(
    timed_out: bool,
    command_failed: bool,
    metric_results: Sequence[Mapping[str, Any]],
    metric_error: str,
    missing_outputs: Sequence[str],
) -> tuple[str, str]:
    if timed_out:
        return "TIMEOUT", "subprocess exceeded its timeout"
    if metric_results and any(not metric["passed"] for metric in metric_results):
        return "METRIC_FAIL", "one or more metric assertions failed"
    if command_failed:
        return "INFRA_FAIL", "subprocess returned a non-zero exit code"
    if metric_error:
        return "INFRA_FAIL", metric_error
    if missing_outputs:
        return "INFRA_FAIL", f"required outputs are missing: {list(missing_outputs)}"
    return "PASS", "all commands, outputs, and metrics passed"


def execute_definition(
    definition: Mapping[str, Any],
    context: Mapping[str, Any],
    output_dir: Path,
    default_timeout: float,
    timeout_override: float | None,
) -> tuple[str, str, list[dict[str, Any]], list[dict[str, Any]]]:
    environment = os.environ.copy()
    environment.update({str(key): str(value) for key, value in definition.get("environment", {}).items()})
    cwd = Path(definition.get("cwd", str(output_dir)))
    cwd.mkdir(parents=True, exist_ok=True)
    command_records: list[dict[str, Any]] = []
    command_failed = False
    timed_out = False
    timeout_sec = float(timeout_override or definition.get("timeout_sec", default_timeout))

    for index, command in enumerate(definition.get("commands", [])):
        argv = command["argv"]
        name = command.get("name", f"command_{index + 1}")
        result = run_command(
            argv,
            cwd,
            environment,
            float(timeout_override or command.get("timeout_sec", timeout_sec)),
            output_dir / f"{index + 1:02d}_{name}.log",
        )
        command_records.append({
            "name": name,
            "argv": argv,
            "returncode": result.returncode,
            "elapsed_sec": result.elapsed_sec,
            "timed_out": result.timed_out,
        })
        if result.timed_out:
            timed_out = True
            break
        if result.returncode not in command.get("success_exit_codes", [0]):
            command_failed = True
            break

    metric_results: list[dict[str, Any]] = []
    metric_error = ""
    try:
        metric_results = evaluate_metrics(definition.get("metrics", []), context)
    except ManifestError as exc:
        metric_error = str(exc)
    missing_outputs = [
        str(path) for path in definition.get("required_outputs", []) if not Path(path).exists()
    ]
    status, reason = result_status(
        timed_out, command_failed, metric_results, metric_error, missing_outputs
    )
    return status, reason, command_records, metric_results


def case_status_record(
    scene: Mapping[str, Any],
    stage_name: str,
    case_name: str,
    status: str,
    reason: str,
    started_at: str,
    elapsed_sec: float,
    commands: Sequence[Mapping[str, Any]],
    metrics: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    return {
        "schema": "robot_navigo.matrix_localization_case_status",
        "schema_version": 1,
        "scene": scene["name"],
        "scene_id": scene["scene_id"],
        "stage": stage_name,
        "case": case_name,
        "status": status,
        "reason": reason,
        "started_at_utc": started_at,
        "ended_at_utc": utc_now(),
        "elapsed_sec": elapsed_sec,
        "commands": list(commands),
        "metrics": list(metrics),
    }


def aggregate_case_status(cases: Sequence[Mapping[str, Any]]) -> tuple[str, str]:
    severity = ("INVALID_INPUT", "TIMEOUT", "INFRA_FAIL", "METRIC_FAIL")
    for status in severity:
        failed = [str(case["case"]) for case in cases if case.get("status") == status]
        if failed:
            return status, f"{status} cases: {', '.join(failed)}"
    return "PASS", f"all {len(cases)} cases passed"


def run_stage(
    manifest: Mapping[str, Any],
    variables: Mapping[str, str],
    run_dir: Path,
    scene: Mapping[str, Any],
    stage_name: str,
    resume: bool,
    timeout_override: float | None,
) -> dict[str, Any]:
    stage = scene["stages"][stage_name]
    context = stage_context(variables, run_dir, scene, stage_name)
    stage_output = Path(context["stage_output"])
    status_path = stage_output / "stage_status.json"
    started_at = utc_now()
    started = time.monotonic()

    try:
        signature, signature_inputs = build_input_signature(scene["name"], stage_name, stage, context)
    except ManifestError as exc:
        record = status_record(
            scene, stage_name, "INVALID_INPUT", str(exc), started_at, time.monotonic() - started
        )
        atomic_write_json(status_path, record)
        return record

    previous = read_status(status_path) if resume else None
    if (
        previous
        and previous.get("status") in REUSABLE_STATUSES
        and previous.get("input_signature") == signature
    ):
        reused = dict(previous)
        reused["resumed"] = True
        reused["ended_at_utc"] = utc_now()
        atomic_write_json(status_path, reused)
        return reused

    archive_stage_output(stage_output)
    stage_output.mkdir(parents=True, exist_ok=True)
    if stage_name == "report":
        record = status_record(
            scene,
            stage_name,
            "PASS",
            "scene results are available to the aggregate reporter",
            started_at,
            time.monotonic() - started,
            signature,
            signature_inputs,
        )
        atomic_write_json(status_path, record)
        return record

    formatted = format_stage(stage, context)
    timeout_sec = float(
        timeout_override
        or formatted.get("timeout_sec", manifest.get("defaults", {}).get("timeout_sec", 600))
    )
    case_records: list[dict[str, Any]] = []
    command_records: list[dict[str, Any]] = []
    metric_results: list[dict[str, Any]] = []
    if formatted.get("cases"):
        for index, definition in enumerate(formatted["cases"]):
            raw_case = stage["cases"][index]
            current_context = case_context(context, raw_case, index)
            case_output = Path(current_context["case_output"])
            case_output.mkdir(parents=True, exist_ok=True)
            case_started_at = utc_now()
            case_started = time.monotonic()
            status, reason, commands, metrics = execute_definition(
                definition, current_context, case_output, timeout_sec, timeout_override
            )
            case_record = case_status_record(
                scene,
                stage_name,
                str(definition["name"]),
                status,
                reason,
                case_started_at,
                time.monotonic() - case_started,
                commands,
                metrics,
            )
            atomic_write_json(case_output / "case_status.json", case_record)
            case_records.append(case_record)
        status, reason = aggregate_case_status(case_records)
    else:
        status, reason, command_records, metric_results = execute_definition(
            formatted, context, stage_output, timeout_sec, timeout_override
        )

    record = status_record(
        scene,
        stage_name,
        status,
        reason,
        started_at,
        time.monotonic() - started,
        signature,
        signature_inputs,
        command_records,
        metric_results,
        case_records,
    )
    atomic_write_json(status_path, record)
    return record


def skipped_status(
    run_dir: Path,
    scene: Mapping[str, Any],
    stage_name: str,
    reason: str,
) -> dict[str, Any]:
    started = utc_now()
    record = status_record(scene, stage_name, "SKIPPED", reason, started, 0.0)
    atomic_write_json(run_dir / scene["name"] / stage_name / "stage_status.json", record)
    return record


def collect_statuses(run_dir: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(run_dir.glob("*/*/stage_status.json")):
        status = read_status(path)
        if status and status.get("status") in FINAL_STATUSES:
            rows.append(status)
    return rows


def write_aggregate_reports(run_dir: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    counts = {status: 0 for status in sorted(FINAL_STATUSES)}
    for row in rows:
        counts[str(row["status"])] += 1
    summary = {
        "schema": "robot_navigo.matrix_localization_batch_summary",
        "schema_version": 1,
        "generated_at_utc": utc_now(),
        "run_dir": str(run_dir.resolve()),
        "counts": counts,
        "results": list(rows),
    }
    atomic_write_json(run_dir / "batch_summary.json", summary)

    csv_path = run_dir / "batch_summary.csv"
    csv_tmp = csv_path.with_name(f".{csv_path.name}.{os.getpid()}.tmp")
    with csv_tmp.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("scene", "scene_id", "stage", "status", "elapsed_sec", "resumed", "reason", "input_signature"),
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in writer.fieldnames})
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(csv_tmp, csv_path)

    metric_csv_path = run_dir / "batch_metrics.csv"
    metric_csv_tmp = metric_csv_path.with_name(f".{metric_csv_path.name}.{os.getpid()}.tmp")
    with metric_csv_tmp.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("scene", "stage", "case", "metric", "actual", "operator", "expected", "passed", "file", "path"),
        )
        writer.writeheader()
        for row in rows:
            metric_groups = [(None, row.get("metrics", []))]
            metric_groups.extend(
                (case.get("case"), case.get("metrics", [])) for case in row.get("cases", [])
            )
            for case_name, metrics in metric_groups:
                for metric in metrics:
                    writer.writerow({
                        "scene": row.get("scene"),
                        "stage": row.get("stage"),
                        "case": case_name,
                        "metric": metric.get("name"),
                        "actual": metric.get("actual"),
                        "operator": metric.get("operator"),
                        "expected": metric.get("expected"),
                        "passed": metric.get("passed"),
                        "file": metric.get("file"),
                        "path": metric.get("path"),
                    })
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(metric_csv_tmp, metric_csv_path)

    lines = [
        "# Matrix 定位批量评测报告",
        "",
        f"- 生成时间：`{summary['generated_at_utc']}`",
        f"- 运行目录：`{summary['run_dir']}`",
        "",
        "## 状态汇总",
        "",
        "| 状态 | 数量 |",
        "|---|---:|",
    ]
    lines.extend(f"| {status} | {count} |" for status, count in counts.items())
    lines.extend([
        "",
        "## 分阶段结果",
        "",
        "| 场景 | ID | 阶段 | 状态 | 用时(s) | 说明 |",
        "|---|---:|---|---|---:|---|",
    ])
    for row in rows:
        reason = str(row.get("reason", "")).replace("|", "\\|").replace("\n", " ")
        lines.append(
            f"| {row.get('scene')} | {row.get('scene_id')} | {row.get('stage')} | "
            f"{row.get('status')} | {float(row.get('elapsed_sec', 0.0)):.3f} | {reason} |"
        )
    unavailable: dict[tuple[Any, Any], str] = {}
    for row in rows:
        if row.get("status") == "SKIPPED" and row.get("available") is False:
            unavailable.setdefault(
                (row.get("scene"), row.get("scene_id")), str(row.get("reason", ""))
            )
    if unavailable:
        lines.extend([
            "",
            "## 不可用场景",
            "",
            "| 场景 | ID | 原因 |",
            "|---|---:|---|",
        ])
        for (scene, scene_id), reason in sorted(unavailable.items()):
            escaped_reason = reason.replace("|", "\\|").replace("\n", " ")
            lines.append(f"| {scene} | {scene_id} | {escaped_reason} |")
    case_rows = [case for row in rows for case in row.get("cases", [])]
    if case_rows:
        lines.extend([
            "",
            "## 分 Case 结果",
            "",
            "| 场景 | 阶段 | Case | 状态 | 用时(s) | 说明 |",
            "|---|---|---|---|---:|---|",
        ])
        for case in case_rows:
            reason = str(case.get("reason", "")).replace("|", "\\|").replace("\n", " ")
            lines.append(
                f"| {case.get('scene')} | {case.get('stage')} | {case.get('case')} | "
                f"{case.get('status')} | {float(case.get('elapsed_sec', 0.0)):.3f} | {reason} |"
            )
    lines.extend([
        "",
        "## 指标明细",
        "",
        "| 场景 | 阶段 | Case | 指标 | 实际值 | 判定 | 阈值 | 通过 |",
        "|---|---|---|---|---:|---|---:|---|",
    ])
    for row in rows:
        metric_groups = [("-", row.get("metrics", []))]
        metric_groups.extend(
            (str(case.get("case")), case.get("metrics", [])) for case in row.get("cases", [])
        )
        for case_name, metrics in metric_groups:
            for metric in metrics:
                actual = str(metric.get("actual", "")).replace("|", "\\|")
                expected = str(metric.get("expected", "")).replace("|", "\\|")
                lines.append(
                    f"| {row.get('scene')} | {row.get('stage')} | {case_name} | "
                    f"{metric.get('name')} | {actual} | {metric.get('operator')} | "
                    f"{expected} | {metric.get('passed')} |"
                )
    atomic_write_text(run_dir / "LOCALIZATION_REPORT.md", "\n".join(lines) + "\n")


def selected_scenes(manifest: Mapping[str, Any], names: Sequence[str]) -> list[dict[str, Any]]:
    scenes = list(manifest["scenes"])
    if not names:
        return scenes
    selected = [scene for scene in scenes if scene.get("name") in names]
    missing = sorted(set(names) - {scene.get("name") for scene in selected})
    if missing:
        raise ManifestError(f"unknown scenes: {', '.join(missing)}")
    return selected


def execute_suite(
    manifest: Mapping[str, Any],
    variables: Mapping[str, str],
    run_dir: Path,
    stage_names: Sequence[str],
    scene_names: Sequence[str] = (),
    resume: bool = False,
    keep_going: bool = False,
    timeout_override: float | None = None,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    stop = False
    for scene in selected_scenes(manifest, scene_names):
        for stage_name in stage_names:
            if not scene["available"]:
                rows.append(skipped_status(run_dir, scene, stage_name, scene["reason"]))
                continue
            if not scene["enabled"]:
                rows.append(skipped_status(run_dir, scene, stage_name, "scene is disabled"))
                continue
            stage = scene["stages"][stage_name]
            if stop and not stage.get("always_run", False):
                rows.append(skipped_status(run_dir, scene, stage_name, "batch stopped after previous failure"))
                continue
            dependencies = stage.get("depends_on", [])
            failed_dependencies = []
            for dependency in dependencies:
                status = read_status(run_dir / scene["name"] / dependency / "stage_status.json")
                if not status or status.get("status") != "PASS":
                    failed_dependencies.append(dependency)
            if failed_dependencies:
                rows.append(skipped_status(
                    run_dir,
                    scene,
                    stage_name,
                    f"dependencies did not pass: {', '.join(failed_dependencies)}",
                ))
                continue
            row = run_stage(
                manifest, variables, run_dir, scene, stage_name, resume, timeout_override
            )
            rows.append(row)
            if (
                row["status"] not in {"PASS", "SKIPPED"}
                and not keep_going
                and not stage.get("continue_on_failure", False)
            ):
                stop = True
    write_aggregate_reports(run_dir, collect_statuses(run_dir))
    return rows


def parse_stages(value: str) -> list[str]:
    stages = [item.strip() for item in value.split(",") if item.strip()]
    invalid = [stage for stage in stages if stage not in STAGES]
    if not stages or invalid:
        raise argparse.ArgumentTypeError(f"stages must be a comma-separated subset of {','.join(STAGES)}")
    return stages


def default_run_dir(manifest: Mapping[str, Any], variables: Mapping[str, str]) -> Path:
    output_root = format_value(manifest.get("output_root", "artifacts/matrix_localization"), variables)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(output_root) / str(manifest["name"]) / timestamp


def exit_code(rows: Sequence[Mapping[str, Any]]) -> int:
    statuses = {row.get("status") for row in rows}
    if statuses & {"INFRA_FAIL", "TIMEOUT", "INVALID_INPUT"}:
        return 2
    if "METRIC_FAIL" in statuses:
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    def common(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument("--suite", type=Path, required=True)
        subparser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")

    validate = subparsers.add_parser("validate", help="validate suite structure and inputs")
    common(validate)

    run = subparsers.add_parser("run", help="execute selected scenes and stages")
    common(run)
    run.add_argument("--run-dir", type=Path)
    run.add_argument("--scene", action="append", default=[])
    run.add_argument("--stages", type=parse_stages, default=list(STAGES))
    run.add_argument("--resume", action="store_true")
    run.add_argument("--keep-going", action="store_true")
    run.add_argument("--timeout", type=float, help="override every subprocess timeout")

    report = subparsers.add_parser("report", help="regenerate aggregate reports")
    common(report)
    report.add_argument("--run-dir", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        manifest = load_manifest(args.suite)
        errors = validate_manifest(manifest)
        if errors:
            raise ManifestError("; ".join(errors))
        variables = resolve_variables(manifest.get("variables", {}), parse_overrides(args.set))

        if args.command == "validate":
            # Formatting catches unresolved manifest placeholders without launching anything.
            for scene in manifest["scenes"]:
                if scene["available"] and scene["enabled"]:
                    for stage_name in STAGES:
                        context = stage_context(variables, Path("/tmp/matrix_suite_validation"), scene, stage_name)
                        format_stage(scene["stages"][stage_name], context)
            print(f"PASS: {args.suite} ({len(manifest['scenes'])} scenes)")
            return 0

        if args.command == "report":
            rows = collect_statuses(args.run_dir)
            if not rows:
                raise ManifestError(f"no stage status files found under {args.run_dir}")
            write_aggregate_reports(args.run_dir, rows)
            print(args.run_dir / "LOCALIZATION_REPORT.md")
            return exit_code(rows)

        if args.timeout is not None and args.timeout <= 0:
            raise ManifestError("--timeout must be positive")
        run_dir = args.run_dir or default_run_dir(manifest, variables)
        run_dir.mkdir(parents=True, exist_ok=True)
        atomic_write_json(run_dir / "suite_snapshot.json", dict(manifest))
        rows = execute_suite(
            manifest,
            variables,
            run_dir,
            args.stages,
            args.scene,
            args.resume,
            args.keep_going,
            args.timeout,
        )
        print(run_dir / "LOCALIZATION_REPORT.md")
        return exit_code(rows)
    except ManifestError as exc:
        print(f"INVALID_INPUT: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
