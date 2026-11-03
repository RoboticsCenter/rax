# Using an operator demonstration

Use this workflow when the operator references a video or image as task context.
The host importer publishes files at `context/<sha256>/<original-name>` and keeps
an independent original. It does not send an instruction or inspect the video.

1. Resolve the exact workspace path given by the operator. If it is unavailable,
   ask for it to be imported into this session; do not search private host folders.
2. Keep the source unchanged. Inspect selected timestamps with local video tools
   and save extracted images plus their source timestamps under
   `output/demonstrations/<name>/`. Ordinary local Python/OpenCV or `ffmpeg` can
   extract frames. Do not pass an MP4 path as an image content block.
{% if gemini %}
   `gemini_generate` accepts inline PNG/JPEG images, not video files. Select and
   label frames with their source timestamps before using that tool.
{% endif %}
3. Record the file path/hash, the inspected time ranges, relevant visible actions
   and any occluded or ambiguous steps. Distinguish observed actions from inferred
   forces, contacts or object states; do not describe uninspected segments as seen.
4. Transfer the task sequence and grasp relationships to fresh robot observations.
   Human hand motions or a different robot's coordinates are not this robot's
   joint targets. Account for the current hands, active joints and camera/base frames.
5. In `memory/RUNS.md`, link the demonstration segments to the attempted robot
   actions and decisive observation/motion-clip references. Record discrepancies
   and operator corrections so the progress report can reconstruct what changed.

The host journal preserves submitted wording and links recognized local media
references to their prompt events. Do not claim that a staged or archived video
was processed by the model just because it exists. Save your actual inspection
and derived evidence in the workspace; a physical intervention outside recorded
motion intervals may have an operator note without corresponding footage.
