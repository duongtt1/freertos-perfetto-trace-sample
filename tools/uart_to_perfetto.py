#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def parse_trace_lines(lines):
    in_trace = False
    events = []
    tasks = {}
    isrs = {}
    markers = {}
    stats = {}

    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue

        if line == "#UART_TRACE_BEGIN":
            in_trace = True
            continue

        if line == "#UART_TRACE_END":
            break

        if not in_trace and not line.startswith("#TRACE"):
            continue

        if line.startswith("#TRACE"):
            in_trace = True
            continue

        if line.startswith("#TASKS"):
            continue

        if line.startswith("#TASK "):
            payload = line[len("#TASK "):]
            task_id, name, prio, total_active_us = payload.split(",", 3)
            tasks[name] = {
                "id": int(task_id),
                "prio": int(prio),
                "total_active_us": int(total_active_us),
            }
            continue

        if line.startswith("#ISR "):
            payload = line[len("#ISR "):]
            isr_id, name = payload.split(",", 1)
            isrs[name] = {"id": int(isr_id)}
            continue

        if line.startswith("#MARKER "):
            payload = line[len("#MARKER "):]
            marker_id, name = payload.split(",", 1)
            markers[name] = {"id": int(marker_id)}
            continue

        if line.startswith("#STATS "):
            for kv in line[len("#STATS "):].split():
                key, value = kv.split("=", 1)
                stats[key] = int(value)
            continue

        if line.startswith("#END"):
            break

        parts = line.split(",", 3)
        if len(parts) != 4:
            continue

        ts_us, name, event_type, prio = parts
        events.append(
            {
                "ts_us": int(ts_us),
                "name": name,
                "type": event_type,
                "prio": int(prio),
            }
        )

    return events, tasks, isrs, markers, stats


def thread_id(name, known_ids):
    if name not in known_ids:
        known_ids[name] = len(known_ids) + 1
    return known_ids[name]


def build_perfetto_trace(events, tasks, isrs, markers, stats):
    trace_events = []
    tids = {}
    open_task_spans = {}
    open_isr_spans = {}
    process_id = 1

    def add_thread_metadata(name):
        tid = thread_id(name, tids)
        trace_events.append(
            {
                "ph": "M",
                "pid": process_id,
                "tid": tid,
                "name": "thread_name",
                "args": {"name": name},
            }
        )
        return tid

    trace_events.append(
        {
            "ph": "M",
            "pid": process_id,
            "name": "process_name",
            "args": {"name": "FreeRTOS Trace"},
        }
    )

    queue_tid = add_thread_metadata("Queues")
    marker_tid = add_thread_metadata("Markers")
    irq_tid = add_thread_metadata("IRQs")

    for event in events:
        name = event["name"]
        event_type = event["type"]
        ts_us = event["ts_us"]
        prio = event["prio"]

        if event_type == "IN":
            open_task_spans[name] = ts_us
            if name not in tids:
                add_thread_metadata(name)
            continue

        if event_type == "OUT":
            start_us = open_task_spans.pop(name, None)
            if start_us is None:
                continue
            trace_events.append(
                {
                    "name": name,
                    "ph": "X",
                    "pid": process_id,
                    "tid": thread_id(name, tids),
                    "ts": start_us,
                    "dur": max(ts_us - start_us, 0),
                    "args": {
                        "priority": prio,
                        "trace_id": tasks.get(name, {}).get("id"),
                    },
                }
            )
            continue

        if event_type in ("QSEND", "QRECV"):
            trace_events.append(
                {
                    "name": f"{event_type}:{name}",
                    "ph": "i",
                    "s": "t",
                    "pid": process_id,
                    "tid": queue_tid,
                    "ts": ts_us,
                    "args": {"priority": prio},
                }
            )
            continue

        if event_type == "MARK":
            trace_events.append(
                {
                    "name": name,
                    "ph": "i",
                    "s": "t",
                    "pid": process_id,
                    "tid": marker_tid,
                    "ts": ts_us,
                    "args": {"marker_id": markers.get(name, {}).get("id")},
                }
            )
            continue

        if event_type == "ISRIN":
            open_isr_spans[name] = ts_us
            continue

        if event_type == "ISROUT":
            start_us = open_isr_spans.pop(name, None)
            if start_us is None:
                continue
            trace_events.append(
                {
                    "name": name,
                    "ph": "X",
                    "pid": process_id,
                    "tid": irq_tid,
                    "ts": start_us,
                    "dur": max(ts_us - start_us, 0),
                    "args": {"isr_id": isrs.get(name, {}).get("id")},
                }
            )
            continue

    if stats:
        trace_events.append(
            {
                "name": "trace_stats",
                "ph": "i",
                "s": "g",
                "pid": process_id,
                "tid": marker_tid,
                "ts": events[-1]["ts_us"] if events else 0,
                "args": stats,
            }
        )

    return {"traceEvents": trace_events, "displayTimeUnit": "ms"}


def main():
    parser = argparse.ArgumentParser(
        description="Convert FreeRTOS UART trace dump to Perfetto JSON."
    )
    parser.add_argument("input", help="UART trace log text file")
    parser.add_argument(
        "-o",
        "--output",
        help="Output Perfetto JSON path (default: <input>.perfetto.json)",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output) if args.output else input_path.with_suffix(
        input_path.suffix + ".perfetto.json"
    )

    with input_path.open("r", encoding="utf-8") as f:
        events, tasks, isrs, markers, stats = parse_trace_lines(f)

    if not events:
        raise SystemExit("No trace events found in input log.")

    perfetto = build_perfetto_trace(events, tasks, isrs, markers, stats)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(perfetto, f, indent=2)

    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
