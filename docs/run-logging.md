# Run logs and progress reports

Launch through `scripts/start_agent.sh` to enable session logging. Launching a
separate `claude` process directly does not install these hooks. The operator
starts Claude; neither importing a demonstration nor exporting a report starts
an agent or moves hardware.

Each session keeps:

- `operator/events.jsonl`: exact submitted prompt text, agent help/reset requests,
  operator replies, dashboard actions, preparation/stop outcomes and manual notes.
  Text is not trimmed or truncated. Events have UTC and Unix timestamps, IDs and
  SHA-256 text hashes. Replies retain their associated request ID and wording.
- `run-logs/claude-transcripts/`: incremental copies of raw Claude Code JSONL
  transcripts, including message and tool content and inline image blocks when
  present. Rewritten transcripts are retained as previous versions. Originals
  remain under `claude-home/projects/`.
- `run-logs/attachments/`: preserved demonstration/context files, addressed by
  SHA-256, independent of mutable workspace copies.
- `mcp.jsonl`: existing robot-tool audit, with complete text results. The existing
  audit size limit remains. Images remain in existing observation artifacts.
- `learning-snapshots/`: existing snapshots at operator replies, plus a final
  snapshot at agent shutdown.
- `progress-report/`: report, exact CSV/JSON timeline, attachment inventory and
  motion-video index. Generated on launcher exit; export on demand while running.

The original motion-recording implementation and settings are unchanged.

## Demonstration videos and other context

Archive the original **before referencing it in Claude**, from another terminal:

```bash
SESSION=/absolute/path/to/the/session
/home/jerry/rax/.venv/bin/python /home/jerry/rax/scripts/import_session_context.py \
  --session "$SESSION" "/path/to/bottle demonstration.mp4"
```

This prints `context/<sha256>/bottle demonstration.mp4`. Paste that exact path in
your Claude instruction, quoting paths containing spaces, for example:

```text
Use "context/<sha256>/bottle demonstration.mp4" as the demonstration for opening the bottle. Extract frames and inspect the sequence before planning the grasp.
```

The importer works before or during a run. Before launch it only stages the file;
the launcher publishes it after creating the workspace. During a run it publishes
immediately. It does not send an instruction on your behalf. The archive retains
original bytes, filename, size and checksum. Subsequent prompt references link the
file to the submitted wording and timestamp. Reusing a filename with changed
contents produces a separate content version.

Local media paths in prompts are also archived in the background when recognized
and still readable. This is best effort: files deleted before archival, unusual
attachment syntax, opaque desktop attachments, or URLs might yield only a text
reference. A missing local file produces an `attachment_unavailable` event.
Use the explicit importer for a demonstration you need in the final video. URLs
are preserved as text, not downloaded automatically. Embedded media in a Claude
transcript remains in that transcript; it is not necessarily the original file.

Staging, submission and model inspection are different events. An archived MP4
does not establish that Claude watched it. The agent may need to extract images;
its tool calls and subsequent responses provide evidence of inspection. This
change adds preservation, not a new native video-input API.

## Physical interventions and spoken instructions

The dashboard's **Intervention notes** records your description verbatim. These
notes do not message the agent. Type actual steering into Claude Code, or answer
its pending request in the dashboard. To save a note from a file:

```bash
/home/jerry/rax/.venv/bin/python /home/jerry/rax/scripts/log_intervention.py \
  --session "$SESSION" --text-file /path/to/intervention.txt
```

The timestamp is when the note was submitted. If recording later, include the
actual intervention time in your note. No microphone capture or automatic
inference of physical resets is added.

## Export and video editing

```bash
/home/jerry/rax/.venv/bin/python /home/jerry/rax/scripts/export_session_report.py \
  --session "$SESSION"
```

Open `progress-report/report.md`. `timeline.csv` retains exact multiline text;
`timeline.json` also retains every event detail. Counts are event counts, not an
automatic count of unique human interventions: a dashboard reply has both an
action record and a reply record. CLI hook submissions can also originate from
generated messages; launcher prompts are explicitly labelled.

The report maps events to existing motion videos using each camera's saved
`frames_ms`, frame index and encoding frame rate. `video_seconds` is the position
in that MP4; `frame_delta_ms` shows how far the nearest recorded frame is from the
event. Gaps in recording remain gaps. Physical resets outside motion clips might
have a note but no footage. The report indexes large artifacts rather than copying
them: keep the full session directory for editing and choose what to share.

## Exactness and operational limits

The prompt hook records the text Claude Code supplies at submission, rather than
unsubmitted keystrokes. The CLI may normalize pasted input or insert paste-wrapper
markup. See the official [UserPromptSubmit reference](https://code.claude.com/docs/en/hooks#userpromptsubmit).
The original raw hook payload is kept alongside its exact `prompt` field.
Operator replies and notes retain the text received by RAX, including whitespace.

Transcript copies run every two seconds and flush on normal shutdown. An abrupt
power loss can leave recent data only in the original CLI transcript. Logging
errors are surfaced in stderr/journal and `run-logs/retention.json`; they do not
block emergency stops. Hooks record submission, which can subsequently be
cancelled; consult the transcript for evidence of model processing. Logging is
installed for newly deployed sessions; older already-deployed sessions do not
gain a hook automatically. No robot motion or recording policy is changed.
