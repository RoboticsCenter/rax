# Private Hugging Face session log backup

The local `rax-hf-log-sync.timer` runs `scripts/sync_hf_logs.py` against the private
dataset `shimamooo/lightbulb-screwing`. It starts another pass three minutes after
the previous pass ends, including after a failure. Each pass uploads at most twenty
batches, so newly completed session logs are revisited during the initial backlog.
This is a file backup, not a live video stream. It needs this laptop online and its
user service manager running. It does not run the RAX agent or move the robots.

The initial scope is the lightbulb session `20261110T091931Z_screw_lightbulb_fable52`:
root logs, model usage, lifecycle, operator requests/replies/events, Claude transcript
copies, motion recordings, learning snapshots, agent memory/code/output, and progress
reports. Raw rolling streams, runtime captures, Claude home, credentials, session
configuration, and unsupported file formats are excluded. This is not a complete
restorable session backup; preserve the active session locally.

Only files unchanged for two minutes are considered. Files changed during copying
are deferred. Appending transcripts are uploaded after they settle; the service does
not claim to stream each new line immediately. Text exports redact known credentials
from the supplied token and session credential directory, common token formats, and
credential-valued fields. Redaction is a precaution, not proof that arbitrary prose
contains no private information; the destination must remain private.

Files are staged into bounded batches (128 MiB target; a larger individual file forms
its own batch). Temporary copies use `/dev/shm/rax-hf-lightbulb/staging` (RAM); copying pauses if less than 1 GiB would remain there or persistent ledger storage falls below 64 MiB. With the operator-authorized `--prune-during-upload` option, already verified
old history is removed before and after each upload pass, including during the
initial backlog. Unuploaded or changed files are retained. Each committed file's size and SHA-256 (LFS) or Git blob SHA-1 are checked at
the exact commit before the local SQLite ledger records success. Changed files are
uploaded again. Interrupted batches can be retried; already verified versions are
skipped. A failed pass is retried by the timer.

Local state and the verified progress counter are in
`/home/jerry/rax-runtime/hf-lightbulb-sync/`. The token is read from
`/home/jerry/.config/rax/huggingface-upload.token` (mode 0600, outside the repository). Do not commit this file.
Install the optional uploader dependency with
`uv pip install --python .venv/bin/python huggingface_hub`.

```bash
systemctl --user status rax-hf-log-sync.service rax-hf-log-sync.timer
journalctl --user -u rax-hf-log-sync.service -f
cat /home/jerry/rax-runtime/hf-lightbulb-sync/status.json

# Pause uploads (does not touch the robot or source logs):
systemctl --user stop rax-hf-log-sync.timer rax-hf-log-sync.service

# Resume uploads:
systemctl --user enable --now rax-hf-log-sync.timer
systemctl --user start rax-hf-log-sync.service
```

## Authorized archival cleanup

The operator authorized `--prune-during-upload` on November 16 to reclaim space
while the initial backup continues. Before and after each upload pass, the uploader
rechecks remote hashes at a fixed current repository revision and local file
identities immediately before deleting eligible files older than one hour.
`--prune-verified-history` remains available for deferring cleanup until a full
upload pass has completed, but is not the active policy.

Eligible files: old learning snapshot contents (except the two newest snapshots),
completed motion archive contents, and `.jsonl.previous-*` transcript copies.
Archive manifests, current transcripts, `claude-home`, the complete live agent
workspace, configs, calibration, operator data and credentials remain local.
Unsupported/unuploaded files and files changed since upload also remain local.
The entire runtime folder is never deleted. Text backups may have secrets redacted.

Learning snapshots share unchanged files with the previous completed snapshot using
hard links, after comparing file bytes and metadata. The live agent workspace is
always independent: changing its output cannot change old snapshots. Treat saved
snapshots as immutable; copy a file elsewhere before editing it. Deleting one
snapshot leaves other snapshots intact. Shared files occupy disk space only once,
although tools that sum apparent sizes may count each path separately. Existing
snapshots can be consolidated with
`services.controller.snapshot_storage.deduplicate_snapshots`; pause the uploader
during consolidation because inode changes invalidate its local verification cache.

Deletion intent, remote path, revision and hash are logged durably in
`hf-lightbulb-sync/cleanup.jsonl`; `cleanup-status.json` summarizes each cleanup.
To disable future cleanup while retaining uploads, remove `--prune-during-upload`
from the user service's ExecStart, run `systemctl --user daemon-reload`, and restart
the upload service. Pausing both the service and timer stops uploads and cleanup.
