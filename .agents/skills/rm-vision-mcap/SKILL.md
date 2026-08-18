---
name: rm-vision-mcap
description: Analyze local MCAP recordings produced by rm_vision_2027 with bounded, read-only inspection, field queries, statistics, project diagnostic presets, and synchronized image overlays. Use when Codex needs to inspect .mcap files, investigate Foxglove topics, diagnose detector/PnP/prediction/control behavior, calculate timing or accuracy metrics, or extract a small set of visual evidence from this repository's recordings. Do not use for recording, rewriting, recovering, filtering, or otherwise modifying MCAP files.
---

# RM Vision MCAP

Analyze recordings through the bundled script. Keep every operation read-only and keep large payloads
out of the conversation.

## Workflow

1. Resolve the recording to an explicit local `.mcap` path. Never enable recording on the user's
   behalf.
2. Run `inspect` first. Treat grouped `out_of_order_log_time` warnings as a known consequence of
   interleaved vision and control publishers unless other structural diagnostics are present.
3. Run the narrowest relevant project `preset`, or use `stats`/`query` for a specific topic and field.
4. Narrow anomalies to a small time window before extracting frames.
5. Run `frames` with at most 6 frames initially, then view only the returned overlay paths needed to
   support the conclusion.
6. Report the source path, time window, commands, metrics, and evidence paths. Distinguish tool
   failures from failed `--check` acceptance thresholds.

Resolve the repository-scoped scripts before use so the workflow also works from a subdirectory:

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
MCAP_SKILL="$REPO_ROOT/.agents/skills/rm-vision-mcap"
MCAP_ANALYZE="$MCAP_SKILL/scripts/mcap_analyze.py"
```

Require a compatible `mcap` CLI on `PATH`. If it is unavailable, report the missing dependency;
this skill does not install software or modify the host environment.

## Commands

```bash
python3 "$MCAP_ANALYZE" inspect recording.mcap
python3 "$MCAP_ANALYZE" preset recording.mcap pnp
python3 "$MCAP_ANALYZE" stats recording.mcap \
  --topic /vision/armor/stats --field total_ms --json
python3 "$MCAP_ANALYZE" query recording.mcap \
  --topic /vision/pnp/stats --field summary.detection.position_error_m.p95 --limit 5
python3 "$MCAP_ANALYZE" frames recording.mcap \
  --start 1786977451.0 --end 1786977453.0 --count 6 --layers pnp
```

Use epoch seconds, epoch nanoseconds, or RFC3339 for `--start` and `--end`. Use repeated `--field`
options for several numeric fields. Use `query --where FIELD OP VALUE`; supported operations are
`eq`, `ne`, `lt`, `le`, `gt`, `ge`, `contains`, and `exists`.

`query` emits at most 20 messages by default and refuses unlimited stdout. Send intentional large
exports to an explicit file with `--output`; do not paste them into chat. `stats` keeps at most
10,000 numeric samples per field for quantiles and labels approximate results. `frames` writes to a
unique directory under `/tmp/mcap-analysis` unless `--output` is explicit, and never writes beside
the MCAP.

Use `preset --check` only when the recording context matches the repository's documented acceptance
conditions. A return code of 3 means at least one threshold failed; it does not mean decoding failed.

## Topic guidance

Read [references/topics.md](references/topics.md) when choosing fields, presets, or overlay layers.
Do not load it for a simple `inspect` request.

## Safety boundaries

- Never invoke `mcap add`, `filter`, `compress`, `decompress`, `merge`, `recover`, or `sort` on a
  user recording.
- Never overwrite, rename, truncate, or move a source MCAP.
- Keep generated JSON, JPEG, PNG, and manifests in `/tmp/mcap-analysis` or a user-approved output
  directory.
- Start with one topic and a bounded time window for large recordings.
- Do not output image Base64 or unbounded NDJSON to the conversation.
