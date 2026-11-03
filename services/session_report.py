"""Export a factual run timeline; never infer task success or rewrite operator words."""
from bisect import bisect_left
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import time

from services.json_io import write_private_json


def read_json(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


def read_events(path, warnings):
    if not path.exists():
        return []
    values = []
    with path.open() as stream:
        for number, line in enumerate(stream, 1):
            try:
                values.append(json.loads(line))
            except ValueError:
                warnings.append(f'{path.name}:{number}: incomplete or invalid JSON retained in raw source')
    return values


def motion_index(root, warnings):
    clips = []
    for path in sorted((root/'motion-archive').rglob('manifest.json')):
        manifest = read_json(path)
        if not manifest:
            warnings.append(f'Could not read {path.relative_to(root)}')
            continue
        clip = {'manifest': str(path.relative_to(root)), **manifest}
        clip['camera_timing'] = {}
        for camera, info in manifest.get('cameras', {}).items():
            if not info.get('sidecar') or not info.get('video'):
                continue
            sidecar = read_json(path.parent/info['sidecar'], {})
            clip['camera_timing'][camera] = {
                'video': str((path.parent/info['video']).relative_to(root)),
                'sidecar': str((path.parent/info['sidecar']).relative_to(root)),
                'frames_ms': sidecar.get('frames_ms', []), 'fps': sidecar.get('fps_nominal')}
        clips.append(clip)
    return clips


def matching_frames(event, clips):
    matches = []
    target = event['time']*1000
    for clip in clips:
        for camera, timing in clip['camera_timing'].items():
            frames, fps = timing['frames_ms'], timing['fps']
            if not frames or not fps or not frames[0] <= target <= frames[-1]:
                continue
            index = bisect_left(frames, target)
            index = min((max(0, index-1), min(index, len(frames)-1)), key=lambda i: abs(frames[i]-target))
            matches.append({'camera': camera, 'video': timing['video'], 'frame_index': index,
                            'video_seconds': index/fps, 'frame_time_ms': frames[index],
                            'frame_delta_ms': frames[index]-target})
    return matches


def fenced(text):
    fence = '`'*max(3, max((len(s) for s in re.findall(r'`+', text)), default=0)+1)
    return f'{fence}text\n{text}\n{fence}\n'


def export_report(root, destination=None):
    root = Path(root).resolve()
    destination = Path(destination or root/'progress-report').resolve()
    destination.mkdir(parents=True, exist_ok=True, mode=0o700)
    warnings = []
    events = sorted(read_events(root/'operator'/'events.jsonl', warnings), key=lambda e: e['time_ns'])
    lifecycle = read_json(root/'agent-lifecycle.json', {})
    config = read_json(root/'operator-input.json', {})
    starts = [e['time'] for e in events if e['event'] == 'session_launch_requested']
    start = min(starts, default=lifecycle.get('started_at', events[0]['time'] if events else time.time()))
    clips = motion_index(root, warnings)
    # Keep the original JSONL (including an incomplete last write) as evidence.
    journal = root/'operator'/'events.jsonl'
    if journal.exists():
        shutil.copyfile(journal, destination/'operator-events.jsonl')
        (destination/'operator-events.jsonl').chmod(0o600)
    fields = ['id', 'utc', 'elapsed_s', 'event', 'action', 'source', 'actor', 'request_id',
              'source_event_id', 'text', 'text_sha256', 'outcome', 'media_matches']
    timeline = []
    for event in events:
        timeline.append({**event, 'elapsed_s': round(event['time']-start, 6),
                         'media_matches': matching_frames(event, clips)})
    with (destination/'timeline.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for event in timeline:
            writer.writerow({**event, 'media_matches': json.dumps(event['media_matches'], ensure_ascii=False)})
    (destination/'timeline.csv').chmod(0o600)
    write_private_json(destination/'timeline.json', timeline)
    write_private_json(destination/'motion-clips.json', clips)
    attachments = [e for e in events if e['event'] in ('attachment_archived', 'attachment_unavailable', 'context_staged')]
    write_private_json(destination/'attachments.json', attachments)
    counts = Counter(e['event'] for e in events)
    artifacts = [str(p.relative_to(root)) for name in (
        'run-logs', 'operator', 'mcp.jsonl', 'model-usage.jsonl', 'agent-lifecycle.json',
        'frontend.log', 'learning-snapshots', 'motion-archive', 'agent-workspace/memory',
        'agent-workspace/code', 'agent-workspace/output') if (p := root/name).exists()]
    summary = {'generated_utc': datetime.now(timezone.utc).isoformat(), 'session': str(root),
               'task': config.get('task'), 'lifecycle': lifecycle, 'event_counts': dict(counts),
               'motion_clips': len(clips), 'artifacts': artifacts, 'warnings': warnings,
               'retention': read_json(root/'run-logs'/'retention.json'),
               'interpretation': 'Counts are journal events, not unique human interventions. '
                 'Dashboard actions overlap replies/notes; hook prompts can be machine-generated. '
                 'A preserved attachment or submitted prompt does not prove model inspection. '
                 'No outcome is inferred from tool completion.'}
    write_private_json(destination/'summary.json', summary)
    lines = [f'# RAX run: {config.get("task", root.name)}\n',
             f'Session: `{root}`. Exported: {summary["generated_utc"]}.\n',
             f'Launcher state: **{lifecycle.get("status", "not launched")}**. '
             'Task success must be assessed from the observations and final agent report.\n',
             '## Evidence and timing\n',
             'The CSV and JSON timelines retain submitted wording without trimming or paraphrasing. '
             'Raw transcripts preserve CLI message/tool content, including inline image blocks when present. '
             'The hook captures text as supplied by Claude Code, not raw keystrokes or unsent edits. '
             'Launcher prompts and unclassified CLI submissions are labelled separately.\n',
             'Motion recording is unchanged. Events outside recorded motion intervals have no matching clip. '
             'Video offsets use each camera’s original frame timestamps and encoded frame index; '
             '`frame_delta_ms` records the gap to the nearest retained frame. '
             'Physical actions and speech need an operator note; they are not inferred automatically.\n',
             '## Counts\n',
             'These are event counts, not unique interventions: dashboard actions can duplicate the '
             'corresponding reply/note, and prompt hooks can also fire for generated messages.\n']
    lines += [f'- {name}: {count}\n' for name, count in sorted(counts.items())]
    lines += ['\n## Attachments\n',
              'See `attachments.json` for original names, paths, content hashes and prompt event links. '
              'Staging means a file is available; submission and inspection are separate evidence. '
              'URLs remain in the verbatim text and are not downloaded automatically.\n']
    for item in attachments:
        lines.append(f'- {item["event"]}: `{item.get("original_name", item.get("workspace_path", item.get("original_path", "")))}`; '
                     f'event `{item["id"]}`; prompt `{item.get("source_event_id", "not linked")}`.\n')
    lines += ['\n## Original artifacts\n', 'Paths are relative to the session; keep the session folder for video editing. '
              'This report indexes large artifacts instead of duplicating them.\n']
    lines += [f'- [{name}]({os.path.relpath(root/name, destination)})\n' for name in artifacts]
    lines += ['\n## Verbatim operator and prompt timeline\n']
    for event in timeline:
        if event['event'] == 'context_scan_complete':
            continue
        action = f' ({event["action"]})' if event.get('action') else ''
        lines.append(f'\n### {event["utc"]} · {event["elapsed_s"]:+.3f}s · {event["event"]}{action}\n')
        lines.append(f'Source: `{event["source"]}`. Event: `{event["id"]}`.\n')
        if 'text' in event:
            lines.append(fenced(event['text']))
        if event.get('outcome'):
            lines.append(f'Outcome: {event["outcome"]}.\n')
        if event.get('error'):
            lines.append(fenced(event['error']))
        if event.get('archive_path'):
            lines.append(f'Preserved file: [{event["archive_path"]}]({os.path.relpath(root/event["archive_path"], destination)}).\n')
        for match in event['media_matches']:
            lines.append(f'- {match["camera"]}: `{match["video"]}` at {match["video_seconds"]:.3f}s '
                         f'(frame offset {match["frame_delta_ms"]:+.1f} ms).\n')
    if warnings:
        lines += ['\n## Export warnings\n', *[f'- {warning}\n' for warning in warnings]]
    (destination/'report.md').write_text('\n'.join(lines), encoding='utf-8')
    (destination/'report.md').chmod(0o600)
    return destination
