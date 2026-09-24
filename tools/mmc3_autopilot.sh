#!/usr/bin/env bash
# mmc3_autopilot.sh — mechanical supervisor for the MMC3/Mother agent pair.
#
# It does NOT make domain decisions. It performs only the mechanical
# handoffs the orchestrator would otherwise repeat by hand:
#   * detect MMC3:-prefixed commits on mmc3/debug-trap-triage, merge them
#     into mmc3/wip, run the gate set, push mmc3/wip to GitHub, refresh
#     checkpoint tag + bundle;
#   * when a session exits, preserve uncommitted work on a rescue branch
#     (never merged), then relaunch the session with a continuation prompt
#     built from its own last report;
#   * stage (never apply) any [[bank_entry]]/[[bank_call]] TOML the coverage
#     agent returns, for human review;
#   * detect stalled sessions (log + CPU frozen) and restart them;
#   * hard-stop on merge conflict, red gate, or the cycle cap.
#
# Never: force, history rewrite, or auto-edit profiles/**.
# Push policy (user standing order): every successful wip merge is pushed
# to origin/mmc3/wip immediately; a failed push hard-stops the loop.
#
# Usage:  MM3_MODE=dry-run tools/mmc3_autopilot.sh   # one evaluation pass, no launches
#         tools/mmc3_autopilot.sh                    # live loop
# Kill:   touch /tmp/mm3_autopilot/STOP
set -uo pipefail

MAIN=/home/haruki/nes-to-sms
WT_DEBUG=/home/haruki/nes-to-sms-mmc3-debug
WT_COV=/home/haruki/nes-to-sms-mmc3-coverage
DBG_BRANCH=mmc3/debug-trap-triage
COV_BRANCH=mmc3/coverage-grind
BACKUP=/home/haruki/nes-to-sms-backups

STATE=/tmp/mm3_autopilot
LOG=$STATE/autopilot.log
STOP=$STATE/STOP
SEEDS=$STATE/coverage_seeds.toml
MAX_CYCLES=${MM3_MAX_CYCLES:-20}
POLL=${MM3_POLL:-60}
STALL_MIN=${MM3_STALL_MIN:-8}
MODE=${MM3_MODE:-live}
GATE_CRATES=(-p analysis -p lower -p z80_emit -p validation -p profile)

mkdir -p "$STATE"
touch "$STATE/cycles"

log() { printf '[%s] %s\n' "$(date -u +%FT%TZ)" "$*" >>"$LOG"; echo "$*"; }

# single-instance lock (pidfile, NOT flock: launched sessions inherit fds and
# would keep an flock held after the supervisor exits)
PIDFILE=$STATE/autopilot.pid
if [ -f "$PIDFILE" ]; then
  old=$(cat "$PIDFILE" 2>/dev/null)
  if [ -n "$old" ] && kill -0 "$old" 2>/dev/null; then
    echo "autopilot already running (pid $old)"; exit 1
  fi
fi
echo $$ >"$PIDFILE"
# EXIT removes the pidfile; INT/TERM exit so they trigger the EXIT trap
# (a bare TERM trap would remove the pidfile and then keep looping).
trap 'rm -f "$PIDFILE"' EXIT
trap 'exit 0' INT TERM

is_alive() { [ -n "${1:-}" ] && ps -p "$1" >/dev/null 2>&1; }

read_state() { cat "$STATE/$1" 2>/dev/null; }
write_state() { printf '%s' "$2" >"$STATE/$1"; }

run_gates() {
  cd "$MAIN" || return 1
  if ! cargo fmt --all -- --check >"$STATE/gate_fmt.log" 2>&1; then
    log "GATE FAIL: fmt (see $STATE/gate_fmt.log)"; return 1
  fi
  if ! cargo test "${GATE_CRATES[@]}" >"$STATE/gate_crates.log" 2>&1; then
    log "GATE FAIL: crates (see $STATE/gate_crates.log)"; return 1
  fi
  if ! cargo test -p nes_to_sms --test synthetic_pipeline >"$STATE/gate_synth.log" 2>&1; then
    log "GATE FAIL: synthetic_pipeline (see $STATE/gate_synth.log)"; return 1
  fi
  return 0
}

refresh_backup() {
  cd "$MAIN" || return 0
  git tag -f -a mmc3/mother-checkpoint -m "autopilot checkpoint $(date -u +%FT%TZ)" >/dev/null 2>&1
  git bundle create "$BACKUP/mmc3-mother-$(date -u +%Y%m%d).bundle" --all >/dev/null 2>&1
}

# Merge new MMC3: commits from the debug branch into mmc3/wip.
# returns 0 ok, 2 conflict, 3 gate failure
merge_debug() {
  local base new_subjects n
  base=$(read_state debug_base)
  [ -z "$base" ] && { write_state debug_base "$(git -C "$MAIN" rev-parse "$DBG_BRANCH")"; return 0; }
  new_subjects=$(git -C "$MAIN" log --format=%s "$base".."$DBG_BRANCH" 2>/dev/null || true)
  n=$(printf '%s\n' "$new_subjects" | grep -c '^MMC3:' || true)
  [ "${n:-0}" -eq 0 ] && return 0
  log "found $n new MMC3: commit(s) on $DBG_BRANCH; merging into mmc3/wip"
  cd "$MAIN" || return 2
  if ! git merge --no-edit "$DBG_BRANCH" >>"$LOG" 2>&1; then
    log "STOP: merge conflict (wip <- debug); manual resolution required"; return 2
  fi
  if ! run_gates; then
    log "STOP: red gate after merging debug"; return 3
  fi
  if ! git push origin mmc3/wip >>"$LOG" 2>&1; then
    log "STOP: push failed (wip -> origin); manual push required"; return 4
  fi
  refresh_backup
  write_state debug_base "$(git -C "$MAIN" rev-parse "$DBG_BRANCH")"
  log "merged + gated; checkpoint refreshed"
  return 0
}

# Preserve uncommitted work on a rescue branch (never merged, human reviews).
rescue_uncommitted() {
  local wt=$1 rc=0
  [ -z "$(git -C "$wt" status --porcelain)" ] && return 0
  local br="mmc3/rescue-$(date -u +%Y%m%d-%H%M%S)"
  if [ "$MODE" = dry-run ]; then log "DRY-RUN: would rescue uncommitted work in $wt to $br"; return 0; fi
  log "WARN: uncommitted work in $wt; committing to $br"
  git -C "$wt" checkout -b "$br" >>"$LOG" 2>&1 || return 1
  git -C "$wt" add -A >>"$LOG" 2>&1
  git -C "$wt" commit -m "MMC3: rescue uncommitted work from exited session ($br)" >>"$LOG" 2>&1 || rc=1
  log "rescue branch $br created (NOT merged; review before use)"
  return $rc
}

# Last assistant text from the newest session log for an agent prefix.
last_report() {
  local prefix=$1 f
  f=$(ls -t "$STATE/${prefix}_session_"*.json /tmp/opencode_"${prefix}"_*.json 2>/dev/null | head -n1)
  [ -z "$f" ] && return 0
  log "continuation source: $f"
  python3 - "$f" <<'PY' 2>/dev/null
import json, sys
texts = []
for line in open(sys.argv[1]):
    line = line.strip()
    if not line:
        continue
    try:
        p = json.loads(line).get('part', {})
    except Exception:
        continue
    if p.get('type') == 'text' and p.get('text', '').strip():
        texts.append(p['text'])
print(texts[-1][:3000] if texts else '')
PY
}

continuation_prompt() {
  local agent=$1 prefix=$2 report hint
  report=$(last_report "$prefix")
  hint=$(cat "$STATE/${prefix}_task_hint" 2>/dev/null || true)
  cat <<EOF
Continue your standing assignment (see your agent instructions; 3h stop rule; never invent bank numbers; commit with MMC3: prefix where applicable). An automated supervisor merged any MMC3: commits into mmc3/wip and gated them; start from the current HEAD of your branch and re-check for drift.

CURRENT OBJECTIVE (authoritative; supersedes stale text below):
${hint:-Continue the standing MMC3 objective for your role.}

--- BEGIN PREVIOUS SESSION REPORT (may be empty if that session produced none) ---
$report
--- END PREVIOUS SESSION REPORT ---
EOF
}

launch() {
  local agent=$1 dir=$2 prompt=$3 pfx=$4 model_override="${5:-}" logf pid
  logf="$STATE/${pfx}_session_$(date +%s).json"
  if [ "$MODE" = dry-run ]; then
    log "DRY-RUN: would launch $agent in $dir (model=${model_override:-default}) -> $logf"; return 0
  fi
  local cmd=(opencode run --agent "$agent" --dir "$dir" --auto --format json)
  [ -n "$model_override" ] && cmd+=(-m "$model_override")
  cmd+=("$prompt")
  nohup "${cmd[@]}" >"$logf" 2>&1 &
  pid=$!
  write_state "${pfx}_pid" "$pid"
  write_state "${pfx}_log" "$logf"
  write_state "${pfx}_started" "$(date +%s)"
  : >"$STATE/${pfx}_lastsz"
  log "launched $agent pid=$pid model=${model_override:-default} log=$logf"
}

# Restart a stalled session. A session is stalled when, for STALL_MIN minutes,
# its log has not grown AND it has no descendant tool processes (frame-diff,
# cargo, python...). CPU-time creep alone is NOT treated as progress: a
# free-tier request parked in epoll_wait still ticks ~3s CPU/min.
# returns 0 if healthy/restarted, 1 if it stalled (caller relaunches)
check_stall() {
  local pfx=$1 pid logf sz now lastsz last started
  pid=$(read_state "${pfx}_pid"); is_alive "$pid" || return 0
  logf=$(read_state "${pfx}_log")
  [ -z "$logf" ] && logf=$(ls -t /tmp/opencode_"$pfx"_*.json 2>/dev/null | head -n1)
  [ -z "$logf" ] || [ ! -f "$logf" ] && return 0
  sz=$(stat -c %s "$logf" 2>/dev/null || echo 0)
  lastsz=$(read_state "${pfx}_lastsz"); last=$(read_state "${pfx}_lastchange")
  now=$(date +%s)
  started=$(read_state "${pfx}_started"); started=${started:-$now}
  # grace period: never judge a session younger than STALL_MIN
  if [ $(( now - started )) -lt $(( STALL_MIN * 60 )) ]; then
    write_state "${pfx}_lastsz" "$sz"; write_state "${pfx}_lastchange" "$now"; return 0
  fi
  # progress: log grew, or any descendant process is alive (real local work)
  if [ "$sz" != "$lastsz" ] || [ -n "$(pgrep -P "$pid" 2>/dev/null)" ]; then
    write_state "${pfx}_lastsz" "$sz"; write_state "${pfx}_lastchange" "$now"; return 0
  fi
  [ -z "$last" ] && { write_state "${pfx}_lastchange" "$now"; return 0; }
  if [ $(( now - last )) -gt $(( STALL_MIN * 60 )) ]; then
    log "WARN: $pfx pid=$pid stalled (no log growth / no children >${STALL_MIN}m); killing"
    kill "$pid" 2>/dev/null; sleep 2
    return 1
  fi
  return 0
}

# Extract TOML blocks from the newest coverage report into $SEEDS (review only).
stage_seeds() {
  local f
  f=$(ls -t "$STATE/coverage_session_"*.json /tmp/opencode_coverage_*.json 2>/dev/null | head -n1)
  [ -z "$f" ] && return 0
  [ "$MODE" = dry-run ] && { log "DRY-RUN: would scan $f for TOML blocks"; return 0; }
  # never clobber an already-staged (possibly human-curated) file
  local target="$SEEDS"
  [ -s "$SEEDS" ] && target="$SEEDS.new"
  python3 - "$f" "$target" <<'PY' 2>/dev/null
import json, re, sys
texts = []
for line in open(sys.argv[1]):
    line = line.strip()
    if not line:
        continue
    try:
        p = json.loads(line).get('part', {})
    except Exception:
        continue
    if p.get('type') == 'text' and p.get('text', '').strip():
        texts.append(p['text'])
blocks, cur = [], None
for ln in "\n".join(texts).splitlines():
    if re.match(r'^\[\[(bank_entry|bank_call|jump_engine|function)\]\]', ln.strip()):
        cur = [ln.strip()]; blocks.append(cur); continue
    if cur is not None:
        if not ln.strip():
            cur = None; continue
        cur.append(ln)
if blocks:
    open(sys.argv[2], 'w').write("\n\n".join("\n".join(b) for b in blocks) + "\n")
PY
  if [ -s "$SEEDS" ] || [ -s "$SEEDS.new" ]; then
    log "STAGED $(grep -c '^\[\[' "$target") TOML block(s) -> $target (REVIEW ONLY; not applied)"
  fi
}

# ── main ─────────────────────────────────────────────────────────────────────
log "autopilot start mode=$MODE cap=$MAX_CYCLES poll=${POLL}s stall=${STALL_MIN}m"
[ -f "$STOP" ] && log "note: STOP file already present"

while true; do
  [ -f "$STOP" ] && { log "STOP file present -> clean exit"; exit 0; }

  # debug side
  if merge_debug; then :; else
    log "stopping: debug merge/gate/push failure"; exit 2
  fi
  dpid=$(read_state debug_pid)
  if is_alive "$dpid"; then
    if ! check_stall debug; then
      launch mmc3-debug "$WT_DEBUG" "$(continuation_prompt debug debug)" debug
    fi
  else
    [ -n "$dpid" ] && log "debug session ended"
    rescue_uncommitted "$WT_DEBUG"
    cycles=$(read_state cycles); cycles=${cycles:-0}
    if [ "$cycles" -ge "$MAX_CYCLES" ]; then log "cycle cap ($MAX_CYCLES) reached -> exit"; exit 0; fi
    launch mmc3-debug "$WT_DEBUG" "$(continuation_prompt debug debug)" debug
    write_state cycles "$((cycles + 1))"
  fi

  # coverage side. Free tier is the default (cheap); after 2 stall restarts
  # escalate to the paid route for the same model (reliability).
  cov_model() {
    local r; r=$(read_state coverage_restarts); r=${r:-0}
    if [ "$r" -ge 2 ]; then echo "opencode-go/muse-spark-1.3-contributor"; fi
  }
  cpid=$(read_state coverage_pid)
  if is_alive "$cpid"; then
    if ! check_stall coverage; then
      r=$(read_state coverage_restarts); r=${r:-0}; r=$(( r + 1 )); write_state coverage_restarts "$r"
      [ "$r" -ge 2 ] && log "coverage stalled ${r}x -> escalating to PAID muse-spark"
      launch mmc3-coverage "$WT_COV" "$(continuation_prompt coverage coverage)" coverage "$(cov_model)"
    fi
  else
    [ -n "$cpid" ] && { log "coverage session ended"; stage_seeds; }
    cycles=$(read_state cycles); cycles=${cycles:-0}
    if [ "$cycles" -ge "$MAX_CYCLES" ]; then log "cycle cap ($MAX_CYCLES) reached -> exit"; exit 0; fi
    launch mmc3-coverage "$WT_COV" "$(continuation_prompt coverage coverage)" coverage "$(cov_model)"
    write_state cycles "$((cycles + 1))"
  fi

  if [ "$MODE" = dry-run ]; then log "DRY-RUN pass complete -> exit 0"; exit 0; fi
  sleep "$POLL"
done
