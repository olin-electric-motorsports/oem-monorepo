#!/usr/bin/env bash
# Local test of pi/versions.sh, the Pi's version store. No Pi needed:
#
#   bash tests/test_versions.sh [SEED]
#
# Drives the real pi/versions.sh in a temp folder (through OEM_DEPLOY_DIR and
# OEM_STARTUP_CONFIG) through a long pseudo-random run of snapshots: edits,
# added/removed/renamed files, binary and empty files, files without a final
# newline, reverts to old states (cache hits), rollbacks, and max_versions
# changes that prune several versions at once. After every step it checks the
# whole store (verify, index invariants, no parent loops, stored bytes) and
# that every version restores to exactly what was snapshotted. Then it tests
# version arguments, bad deploy manifests, damaged stores (a lost index.json
# loses no versions), locking, recovery from runs killed part-way (and that
# the next run finishes them), the checks after writing a version, file
# modes, and a rollback applied by the real startup.sh. Exits 0 if everything
# passed; on failure it keeps the temp folder and prints the seed (rerun with
# it to reproduce).

set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DASH=$(dirname "$HERE")
ENGINE=$DASH/pi/versions.sh
PI_LIB=$DASH/pi/lib/common.sh
SEED=${1:-4242}
RANDOM=$SEED
STEPS=40

T=$(mktemp -d "${TMPDIR:-/tmp}/test_versions.XXXXXX")
KEEP=
# The engine keeps its work cache in /tmp if that's a tmpfs. The random run
# uses one (/dev/shm, if it's a tmpfs here); everything else a folder that
# doesn't exist, so the cache goes in the store.
SHM=
if [ "$(stat -f -c %T /dev/shm 2>/dev/null)" = tmpfs ]; then SHM=$(mktemp -d /dev/shm/test_versions.XXXXXX); fi
trap '[ -z "$SHM" ] || rm -rf "$SHM"; [ -n "$KEEP" ] && echo "(kept $T)" >&2 || rm -rf "$T"' EXIT
# Everything happens under $T; nothing may touch the real home folder.
export HOME=$T/home
export OEM_DEPLOY_DIR=$HOME/deploy OEM_STARTUP_CONFIG=$T/startup_config.json OEM_VERSIONS_TMP=$T/no-tmpfs
VD=$OEM_DEPLOY_DIR/versions
INDEX=$VD/index.json
PENDING=$OEM_DEPLOY_DIR/pending
MANIFEST=$OEM_DEPLOY_DIR/deploy-manifest.json
ROOT=$HOME/oem   # where the "installed" files live
OUT=$T/out ERR=$T/err
mkdir -p "$ROOT" "$OEM_DEPLOY_DIR" "$T/snap"
STEP=setup

fail() {
  echo "FAIL (seed $SEED, step $STEP): $*" >&2
  [ ! -s "$OUT" ] || sed 's/^/  stdout: /' "$OUT" >&2
  [ ! -s "$ERR" ] || sed 's/^/  stderr: /' "$ERR" >&2
  KEEP=1
  exit 1
}

# vs ARGS...: run the engine, stdout -> $OUT, stderr -> $ERR.
vs() { bash "$ENGINE" "$@" >"$OUT" 2>"$ERR"; }

set_max() { MAX=$1; printf '{"max_versions": %s}\n' "$1" >"$OEM_STARTUP_CONFIG"; }

hash_of() {  # hash_of DIR: version_hash of DIR/manifest.json + DIR/files
  (source "$PI_LIB" && version_hash "$1/manifest.json" "$1/files")
}

# bytes_ok VERSIONS_DIR: every version's stored_bytes is what's in its full/ and patch/.
bytes_ok() {
  local id b line
  local -A disk=()
  while read -r b line; do
    id=${line%%/*}
    disk[$id]=$((${disk[$id]:-0} + b))
  done < <(find "$1" -mindepth 3 -type f \( -path '*/full/*' -o -path '*/patch/*' \) -printf '%s %P\n')
  while read -r id b; do
    [ "${disk[$id]:-0}" -eq "$b" ] || fail "$id: stored_bytes $b but ${disk[$id]:-0} bytes on disk"
  done < <(jq -r '.versions[] | "\(.id) \(.stored_bytes)"' "$1/index.json")
}

# ------------------------------------------------------------------ the files being "deployed"

# Deterministic binary data (with NULs) for binary files.
POOL=$T/pool
for i in $(seq 1 120); do printf '%s' "$i-$SEED" | sha256sum | cut -c1-64 | tr a-f A-F; done |
  basenc --base16 -d >"$POOL"

SLOTS=(dash dbc cfg bin empty crlf spaced dup mixed)
declare -A SLOT_PATH=(
  [dash]=$ROOT/dashboard.py [dbc]=$ROOT/mkvi.dbc [cfg]=$ROOT/conf/settings.ini
  [bin]=$ROOT/assets/logo.bin [empty]=$ROOT/empty.txt [crlf]=$ROOT/windows.txt
  [spaced]="$ROOT/my notes.txt" [dup]=$ROOT/tools/dashboard.py [mixed]=$ROOT/data.dat
)
declare -A SLOT_NAME=(
  [dash]=dashboard.py [dbc]=mkvi.dbc [cfg]=settings.ini [bin]=logo.bin [empty]=empty.txt
  [crlf]=windows.txt [spaced]="my notes.txt" [dup]=dashboard_1.py [mixed]=data.dat
)
declare -A SLOT_REPO=() SLOT_IN=()
for s in "${SLOTS[@]}"; do SLOT_REPO[$s]=dash/${SLOT_NAME[$s]}; SLOT_IN[$s]=; mkdir -p "$(dirname "${SLOT_PATH[$s]}")"; done

# Anything using $RANDOM stays out of subshells and pipelines: bash reseeds
# RANDOM there, which would make a run impossible to repeat from its seed.
bin_write() {
  local tag=$RANDOM from=$((RANDOM % 2000 + 1)) len=$((300 + RANDOM % 2500))
  { printf 'BIN\0%05d\0' "$tag"; tail -c +"$from" "$POOL" | head -c "$len"; } >"$1"
}

text_edit() {  # a random edit; the file may end up without a final newline
  local f=$1 n
  [ -f "$f" ] || printf 'recreated %s\n' "$RANDOM" >"$f"
  n=$(wc -l <"$f")
  case $((RANDOM % 7)) in
    0 | 1) sed -i "$((RANDOM % (n + 1) + 1))s/.*/edited line $RANDOM/" "$f" ;;
    2) sed -i "$((RANDOM % (n + 1) + 1))i inserted line $RANDOM" "$f" ;;
    3) sed -i "$((RANDOM % (n + 1) + 1))d" "$f" ;;
    4) printf 'appended %s\nno final newline %s' "$RANDOM" "$RANDOM" >>"$f" ;;
    5) if [ -s "$f" ] && [ -z "$(tail -c1 "$f" | tr -d '\n')" ]; then truncate -s -1 "$f"; else echo >>"$f"; fi ;;
    6) for n in 1 2 3; do sed -i "$((RANDOM % 40 + 1))s/\$/ x$RANDOM/" "$f"; done ;;
  esac
}

init_files() {
  cp "$DASH/dashboard.py" "${SLOT_PATH[dash]}"
  cp "$DASH/mkvi.dbc" "${SLOT_PATH[dbc]}"
  printf '[display]\nbrightness=%s\nunits=metric' "$RANDOM" >"${SLOT_PATH[cfg]}"
  bin_write "${SLOT_PATH[bin]}"
  : >"${SLOT_PATH[empty]}"
  printf 'line one\r\nline two\r\nline three\r\n' >"${SLOT_PATH[crlf]}"
  printf 'notes with a space in the name\n' >"${SLOT_PATH[spaced]}"
  head -c 3000 "$DASH/dashboard.py" >"${SLOT_PATH[dup]}"
  printf 'text for now\n' >"${SLOT_PATH[mixed]}"
  SLOT_IN=([dash]=1 [dbc]=1 [cfg]=1 [bin]=1 [empty]= [crlf]= [spaced]= [dup]= [mixed]=)
}

write_manifest() {  # the deploy manifest for the slots that are "deployed"
  local s
  for s in "${SLOTS[@]}"; do
    [ -z "${SLOT_IN[$s]}" ] || printf '%s\t%s\t%s\n' "${SLOT_NAME[$s]}" "${SLOT_PATH[$s]}" "${SLOT_REPO[$s]}"
  done | jq -R -s 'split("\n") | map(select(length > 0) | split("\t") | {file_name: .[0], file_path: .[1], repo_path: .[2]})' >"$MANIFEST"
}

slots_from_manifest() {  # after a revert/rollback replaced the manifest
  local s name path repo
  for s in "${SLOTS[@]}"; do SLOT_IN[$s]=; done
  while IFS=$'\t' read -r name path repo; do
    for s in "${SLOTS[@]}"; do
      if [ "${SLOT_PATH[$s]}" = "$path" ]; then SLOT_IN[$s]=1 SLOT_NAME[$s]=$name SLOT_REPO[$s]=$repo; fi
    done
  done < <(jq -r '.[] | [.file_name, .file_path, .repo_path] | @tsv' "$MANIFEST")
}

present_slots() {  # deployed slots, one per line
  local s
  for s in "${SLOTS[@]}"; do [ -z "${SLOT_IN[$s]}" ] || echo "$s"; done
}

pick_present() { local -a p; mapfile -t p < <(present_slots); PICK=${p[RANDOM % ${#p[@]}]}; }

# install_state DIR [MANIFEST]: put DIR/files in place and use DIR's manifest
# (by default DIR/manifest.json), like startup.sh does with pending/.
install_state() {
  local name path manifest=${2:-$1/manifest.json}
  while IFS=$'\t' read -r name path; do
    mkdir -p "$(dirname "$path")"
    cp "$1/files/$name" "$path"
  done < <(jq -r '.[] | [.file_name, .file_path] | @tsv' "$manifest")
  cp "$manifest" "$MANIFEST"
  slots_from_manifest
}

existing_files() {  # how many deployed files exist on "the Pi"
  local s n=0
  for s in $(present_slots); do [ ! -f "${SLOT_PATH[$s]}" ] || n=$((n + 1)); done
  echo "$n"
}

# ------------------------------------------------------------------ the model of what the store should hold

MODEL_IDS=() MODEL_CUR= MODEL_NEXT=1
declare -A MIRROR=() MIRROR_HASH=() HASH_ID=() MIRROR_FULL=()

model_prune() {
  local id keep=() extra=$((${#MODEL_IDS[@]} - MAX))
  for id in "${MODEL_IDS[@]}"; do
    if [ "$extra" -gt 0 ] && [ "$id" != "$MODEL_CUR" ]; then extra=$((extra - 1)); else keep+=("$id"); fi
  done
  MODEL_IDS=("${keep[@]}")
}

in_model() { [[ " ${MODEL_IDS[*]} " == *" $1 "* ]]; }

# mirror_state DIR: copy what a snapshot should see now (the deployed files
# that exist, and a manifest of those) into DIR/files and DIR/manifest.json.
mirror_state() {
  local name path repo
  mkdir -p "$1/files"
  while IFS=$'\t' read -r name path repo; do
    if [ -f "$path" ]; then
      cp "$path" "$1/files/$name"
      printf '%s\t%s\t%s\n' "$name" "$path" "$repo"
    fi
  done < <(jq -r '.[] | [.file_name, .file_path, .repo_path] | @tsv' "$MANIFEST") |
    jq -R -s 'split("\n") | map(select(length > 0) | split("\t") | {file_name: .[0], file_path: .[1], repo_path: .[2]})' >"$1/manifest.json"
}

model_new() {  # model_new ID DIR: ID is a new version holding what's in DIR
  MIRROR[$1]=$2 MIRROR_HASH[$1]=$(hash_of "$2") MIRROR_FULL[$1]=$(cat "$2"/files/* | wc -c)
  HASH_ID[${MIRROR_HASH[$1]}]=$1
  MODEL_IDS+=("$1") MODEL_CUR=$1 MODEL_NEXT=$((MODEL_NEXT + 1))
  model_prune
}

# snapshot SOURCE DETAIL: copy what the engine should see into $T/snap/<step>,
# run the snapshot, and check it was a cache hit or a new version as expected.
# Either way it prunes down to max_versions.
snapshot() {
  local dir=$T/snap/$STEP.$RANDOM h id missing expect
  mirror_state "$dir"
  missing=$(jq length "$MANIFEST")
  missing=$((missing - $(jq length "$dir/manifest.json")))
  h=$(hash_of "$dir")
  vs snapshot --source "$1" --detail "$2" || fail "snapshot failed"
  id=${HASH_ID[$h]-}
  if [ -n "$id" ] && in_model "$id"; then
    expect="$id (already stored)"
    [ "$(cat "$OUT")" = "$expect" ] || fail "expected '$expect'"
    MODEL_CUR=$id
    model_prune
    HITS=$((HITS + 1))
  else
    id=v$MODEL_NEXT
    grep -q "^$id (new, " "$OUT" || fail "expected a new version $id"
    model_new "$id" "$dir"
    NEWS=$((NEWS + 1))
  fi
  [ "$(jq '.versions | length' "$INDEX")" -le "$MAX" ] || fail "more than max_versions=$MAX versions after a snapshot"
  # Only expected chatter on stderr: prune notes and missing-file warnings.
  if grep -v -e '^Pruned v[0-9]*' -e "doesn't exist; leaving it out of this version" "$ERR" | grep -q .; then
    fail "unexpected messages from snapshot"
  fi
  [ "$(grep -c "doesn't exist; leaving it out" "$ERR" || true)" -eq "$missing" ] || fail "expected $missing missing-file warning(s)"
  if grep -q "^Pruned" "$ERR"; then PRUNES=$((PRUNES + $(grep -c '^Pruned' "$ERR"))); fi
}

# check_restore ID: restore ID and compare pending/ with what was snapshotted.
check_restore() {
  local id=$1 arg=${2:-$1}
  vs restore "$arg" || fail "restore $arg failed"
  diff -r "${MIRROR[$id]}/files" "$PENDING/files" >/dev/null || fail "restored files of $id differ from what was snapshotted"
  jq -e -n --slurpfile a "$PENDING/deploy-manifest.json" --slurpfile b "${MIRROR[$id]}/manifest.json" \
    --slurpfile i "$PENDING/deploy-info.json" --arg id "$id" '
    def norm: map({file_name, file_path, repo_path: (.repo_path // "")}) | sort_by(.file_path);
    ($a[0] | norm) == ($b[0] | norm) and ($a[0][0] | keys) == ["file_name", "file_path", "repo_path"]
    and $i[0].source == "rollback" and $i[0].version == $id and $i[0].detail == "rollback to \($id)"
    and ($i[0].time | test("^[0-9]{4}-[0-9]{2}-[0-9]{2}T"))' >/dev/null ||
    fail "restored manifest/deploy-info of $id is wrong"
}

# check_store [all]: the whole store is sound and holds exactly what the model
# says. verify rebuilds every version and checks it against its version hash,
# which must be the hash of what was snapshotted; on top of that, every
# version whose folder is new or was rewritten (or all, with "all") is
# restored and compared byte for byte.
declare -A CHECKED=()
check_store() {
  local line next cur id parent hash bytes p hops n ino kind size
  local -a ids=()
  local -A par=() sbytes=() disk=() inode=()
  vs verify || fail "verify failed"
  { read -r next cur; while read -r id parent hash bytes; do
      ids+=("$id") par[$id]=$parent sbytes[$id]=$bytes
      [ "$hash" = "${MIRROR_HASH[$id]-}" ] || fail "$id's hash in index.json isn't the hash of what was snapshotted"
    done; } < <(jq -r '"\(.next_id) \(.current // "-")", (.versions[] | "\(.id) \(.parent // "-") \(.hash) \(.stored_bytes)")' "$INDEX")
  [ "${ids[*]}" = "${MODEL_IDS[*]}" ] || fail "index has [${ids[*]}], expected [${MODEL_IDS[*]}]"
  [ "$cur" = "$MODEL_CUR" ] || fail "current is $cur, expected $MODEL_CUR"
  [ "$next" = "$MODEL_NEXT" ] || fail "next_id is $next, expected $MODEL_NEXT"
  for id in "${ids[@]}"; do   # every parent exists, and following parents always ends
    p=${par[$id]} hops=0
    while [ "$p" != - ]; do
      [ -n "${par[$p]+set}" ] || fail "$id's parent chain reaches $p, which isn't in the index"
      p=${par[$p]} hops=$((hops + 1))
      [ "$hops" -le ${#ids[@]} ] || fail "parent loop through $id"
    done
    [ "$hops" -le "$MAX_DEPTH" ] || MAX_DEPTH=$hops
  done
  # One look at the folder: version folders (and their inodes), the bytes in
  # full/ and patch/, and anything that shouldn't be there.
  n=0
  while read -r kind ino size line; do
    case $kind in
      d) if [ "${line%%/*}" = "$line" ]; then inode[$line]=$ino n=$((n + 1)); fi ;;
      f) case $line in index.json | .lock) ;; */full/* | */patch/*) id=${line%%/*}; disk[$id]=$((${disk[$id]:-0} + size)) ;; */manifest.json) ;; *) fail "unexpected file $line in $VD" ;; esac ;;
    esac
  done < <(find "$VD" -mindepth 1 -printf '%y %i %s %P\n')
  [ "$n" -eq ${#ids[@]} ] || fail "unexpected folders in $VD: $(ls -A "$VD")"
  # stored_bytes = what's really in full/ and patch/; never more than storing in full.
  for id in "${ids[@]}"; do
    [ "${disk[$id]:-0}" -eq "${sbytes[$id]}" ] || fail "$id: stored_bytes ${sbytes[$id]} but ${disk[$id]:-0} bytes on disk"
    [ "${sbytes[$id]}" -le "${MIRROR_FULL[$id]}" ] || fail "$id is stored in more bytes than its files"
    [ "${par[$id]}" != - ] || [ "${sbytes[$id]}" -eq "${MIRROR_FULL[$id]}" ] || fail "$id has no parent but isn't stored in full"
  done
  for id in "${ids[@]}"; do
    # A rewrite makes a new folder (vN.tmp, renamed) while the old one still exists, so its inode differs.
    if [ "${1-}" = all ] || [ "${CHECKED[$id]-}" != "${inode[$id]}" ]; then
      check_restore "$id"
      CHECKED[$id]=${inode[$id]} RESTORES=$((RESTORES + 1))
    fi
  done
  rm -rf "$PENDING"
}

# ------------------------------------------------------------------ 1. the long random run

echo "test_versions: seed $SEED"
start=$SECONDS t=$SECONDS HITS=0 NEWS=0 PRUNES=0 ROLLBACKS=0 MAX_DEPTH=0 RESTORES=0
set_max 4
init_files
write_manifest
vs list || fail "list on an empty store failed"
[ "$(cat "$OUT")" = "No versions stored yet." ] || fail "list on an empty store"
vs current && [ ! -s "$OUT" ] || fail "current on an empty store should print nothing"
vs verify && [ "$(cat "$OUT")" = "No versions stored yet." ] || fail "verify on an empty store"
[ -z "$SHM" ] || export OEM_VERSIONS_TMP=$SHM
STEP=0
snapshot deploy "first deploy"
check_store

for ((STEP = 1; STEP <= STEPS; STEP++)); do
  case $STEP in   # max_versions changes, including drops that prune several at once
    12) set_max 2 ;;
    18) set_max 1 ;;
    23) set_max 6 ;;
    33) set_max 3 ;;
  esac
  source=deploy
  r=$((RANDOM % 100))
  case $((STEP % 8)) in 3) r=95 ;; 6) r=80 ;; esac   # make sure rollbacks and reverts happen
  if [ "$r" -lt 30 ]; then
    text_edit "${SLOT_PATH[dash]}"
  elif [ "$r" -lt 38 ]; then
    text_edit "${SLOT_PATH[dbc]}"
  elif [ "$r" -lt 46 ]; then
    pick_present; s=$PICK
    case $s in bin) bin_write "${SLOT_PATH[bin]}" ;; *) text_edit "${SLOT_PATH[$s]}" ;; esac
  elif [ "$r" -lt 52 ]; then   # binary changes, and a file switching between text and binary
    if [ $((RANDOM % 2)) -eq 0 ]; then bin_write "${SLOT_PATH[bin]}"; SLOT_IN[bin]=1
    elif [ $((RANDOM % 2)) -eq 0 ]; then bin_write "${SLOT_PATH[mixed]}"; SLOT_IN[mixed]=1
    else printf 'text again %s\n' "$RANDOM" >"${SLOT_PATH[mixed]}"; SLOT_IN[mixed]=1; fi
  elif [ "$r" -lt 56 ]; then   # empty <-> not empty
    if [ -s "${SLOT_PATH[empty]}" ]; then : >"${SLOT_PATH[empty]}"; else printf 'not empty %s\n' "$RANDOM" >"${SLOT_PATH[empty]}"; fi
    SLOT_IN[empty]=1
  elif [ "$r" -lt 63 ]; then   # add a file
    for s in crlf spaced dup empty mixed bin cfg; do [ -n "${SLOT_IN[$s]}" ] || { SLOT_IN[$s]=1; break; }; done
    [ -f "${SLOT_PATH[$s]}" ] || printf 'new file %s\n' "$RANDOM" >"${SLOT_PATH[$s]}"
  elif [ "$r" -lt 68 ]; then   # remove a file from the deploy
    if [ "$(present_slots | wc -l)" -gt 1 ]; then pick_present; SLOT_IN[$PICK]=; fi
  elif [ "$r" -lt 72 ]; then   # same file_path, different file_name
    pick_present; s=$PICK
    case ${SLOT_NAME[$s]} in renamed_*) SLOT_NAME[$s]=${SLOT_NAME[$s]#renamed_} ;; *) SLOT_NAME[$s]=renamed_${SLOT_NAME[$s]} ;; esac
  elif [ "$r" -lt 75 ]; then   # same content, different repo_path
    pick_present; s=$PICK
    SLOT_REPO[$s]=moved/$RANDOM/${SLOT_NAME[$s]}
  elif [ "$r" -lt 79 ]; then   # a deployed file went missing on the Pi
    if [ "$(existing_files)" -ge 2 ]; then pick_present; rm -f "${SLOT_PATH[$PICK]}"; fi
  elif [ "$r" -lt 88 ]; then   # back to an earlier state (a cache hit if it's still stored)
    mapfile -t old < <(ls -d "$T"/snap/*)
    install_state "${old[RANDOM % ${#old[@]}]}"
    source=github
  elif [ "$r" -lt 93 ]; then   # nothing changed: a cache hit
    source=local
  else   # roll back: restore -> what startup.sh does with pending/ -> snapshot hits the cache
    id=${MODEL_IDS[RANDOM % ${#MODEL_IDS[@]}]}
    case $((RANDOM % 3)) in 0) arg=$id ;; 1) arg=${id#v} ;; 2) arg=${MIRROR_HASH[$id]:0:$((4 + RANDOM % 8))} ;; esac
    check_restore "$id" "$arg"
    install_state "$PENDING" "$PENDING/deploy-manifest.json"
    rm -rf "$PENDING"
    vs snapshot --source rollback --detail "rollback to $id" || fail "snapshot after rollback failed"
    [ "$(cat "$OUT")" = "$id (already stored)" ] || fail "snapshot after rolling back to $id should be a cache hit"
    [ "$(bash "$ENGINE" current)" = "$id" ] || fail "current should be $id after the rollback"
    MODEL_CUR=$id ROLLBACKS=$((ROLLBACKS + 1))
    model_prune   # a cache hit prunes too, e.g. after max_versions went down
    check_store
    continue
  fi
  [ "$source" = github ] || write_manifest
  if [ "$(existing_files)" -eq 0 ]; then   # the engine refuses to store nothing
    printf 'back again\n' >"${SLOT_PATH[dash]}"; SLOT_IN[dash]=1; write_manifest
  fi
  snapshot "$source" "step $STEP"
  check_store
done
STEP=after-run
check_store all
if [ -n "$SHM" ]; then
  [ -z "$(ls -A "$SHM")" ] || fail "the engine left its work cache behind in $SHM: $(ls -A "$SHM")"
  where="cache on tmpfs"
else
  where="no tmpfs here: cache in the store"
fi
export OEM_VERSIONS_TMP=$T/no-tmpfs
echo "  random run: $STEPS steps, $NEWS new versions, $HITS cache hits, $ROLLBACKS rollbacks, $PRUNES prunes, deepest chain $MAX_DEPTH, $RESTORES restores checked, $where ($((SECONDS - t)) s)"
[ "$NEWS" -ge 20 ] && [ "$HITS" -ge 3 ] && [ "$ROLLBACKS" -ge 3 ] && [ "$PRUNES" -ge 5 ] ||
  fail "the random run didn't exercise enough (try another seed)"

# ------------------------------------------------------------------ 2. shapes: a deep chain, a pruned version with several surviving children

STEP=shapes t=$SECONDS MAX_DEPTH=0
set_max 20
for s in "${SLOTS[@]}"; do SLOT_IN[$s]=; done
SLOT_IN[dash]=1 SLOT_IN[dbc]=1 SLOT_IN[bin]=1
for s in dash dbc bin; do [ -f "${SLOT_PATH[$s]}" ] || printf 'back\n' >"${SLOT_PATH[$s]}"; done
# With a final newline, "base + one appended line" is closest to the base, not to a sibling.
[ -z "$(tail -c1 "${SLOT_PATH[dash]}" | tr -d '\n')" ] || echo >>"${SLOT_PATH[dash]}"
write_manifest
snapshot deploy "fewer files"
base=$MODEL_CUR
for i in $(seq 1 8); do   # a chain: each version appends to the one before
  printf '# chain %s\n' "$i" >>"${SLOT_PATH[dash]}"
  snapshot deploy "chain $i"
done
check_store
[ "$MAX_DEPTH" -ge 7 ] || fail "expected a deep parent chain, got depth $MAX_DEPTH"
depth=$MAX_DEPTH
for i in 1 2 3; do   # each is the base plus one small change: all stored as diffs vs the base
  install_state "${MIRROR[$base]}"
  printf '# child %s\n' "$i" >>"${SLOT_PATH[dash]}"
  SLOT_IN[dash]=1
  write_manifest
  snapshot deploy "child $i of $base"
done
check_store
[ "$(jq --arg b "$base" '[.versions[] | select(.parent == $b)] | length' "$INDEX")" -ge 4 ] ||
  fail "expected the first of the chain and three more versions stored against $base"
set_max 4   # one snapshot now prunes a dozen versions: the base, the whole chain and more
install_state "${MIRROR[$base]}"
printf '# child 4\n' >>"${SLOT_PATH[dash]}"
SLOT_IN[dash]=1
write_manifest
snapshot deploy "prune a lot"
n=$(grep -c '^Pruned' "$ERR")
[ "$n" -ge 9 ] || fail "expected at least 9 prunes in one snapshot"
grep -q "^Pruned $base; v[0-9]* now [^,]*, v[0-9]* now [^,]*, v[0-9]* now" "$ERR" ||
  fail "expected $base's three surviving children re-pointed"
# Only versions that survive are re-pointed (and reported).
for id in $(grep -o 'v[0-9]* now' "$ERR" | cut -d' ' -f1); do
  in_model "$id" || fail "$id was re-pointed, but the same prune deleted it"
done
check_store all
echo "  shapes: a chain of depth $depth, $n prunes in one snapshot, one of them with three surviving children ($((SECONDS - t)) s)"

# ------------------------------------------------------------------ 3. commands and arguments

STEP=commands t=$SECONDS
set_max 4
for i in 1 2; do printf '# more %s\n' "$i" >>"${SLOT_PATH[dash]}"; snapshot deploy "more $i"; done
check_store
id=$MODEL_CUR h=${MIRROR_HASH[$id]}
for arg in "$id" "${id#v}" "V${id#v}" "${h:0:4}" "${h:0:12}" "${h^^}" "$h"; do
  vs show "$arg" || fail "show $arg failed"
  head -1 "$OUT" | grep -q "^$id  (current)$" || fail "show $arg should show $id as current"
done
grep -q "^  hash      $h$" "$OUT" || fail "show should print the full hash"
grep -qE '^  [0-9a-f]{8}  [^ ]+ +[0-7]{3,4}  ' "$OUT" || fail "show should print each file's mode"
vs list -n 2 || fail "list -n 2 failed"
[ "$(grep -c '^ [ *] v[0-9]' "$OUT")" -eq 2 ] && grep -q "^ \* $id " "$OUT" && grep -q 'older not shown' "$OUT" || fail "list -n 2 output"
vs list || fail "list failed"
[ "$(grep -c '^ [ *] v[0-9]' "$OUT")" -eq ${#MODEL_IDS[@]} ] || fail "list should show every version"
for bad in v0 999 abc zzzz 12345678901234567890 "v1 2" ""; do
  if vs show "$bad"; then fail "show '$bad' should fail"; fi
  grep -q '^ERROR: ' "$ERR" || fail "show '$bad' should print an error"
done
if vs snapshot; then fail "snapshot without --source should fail"; fi
if vs snapshot --source nonsense; then fail "snapshot --source nonsense should fail"; fi
if vs frobnicate; then fail "an unknown command should fail"; fi
if vs list -n 0; then fail "list -n 0 should fail"; fi
vs -h && grep -q 'versions.sh restore VERSION' "$OUT" || fail "-h should print the usage"

# A hash prefix two versions share is ambiguous (made up in a copy of the store).
cp -a "$OEM_DEPLOY_DIR" "$T/amb"
a=${MODEL_IDS[0]} b=${MODEL_IDS[1]}
jq --arg a "$a" --arg b "$b" '(.versions[] | select(.id == $a) | .hash) = ("abcd1" + (.hash[5:])) |
  (.versions[] | select(.id == $b) | .hash) = ("abcd2" + (.hash[5:]))' "$T/amb/versions/index.json" >"$T/amb/i" &&
  mv "$T/amb/i" "$T/amb/versions/index.json"
if OEM_DEPLOY_DIR=$T/amb vs show abcd; then fail "an ambiguous hash prefix should fail"; fi
grep -q "more than one version" "$ERR" || fail "ambiguous prefix error should say so"
OEM_DEPLOY_DIR=$T/amb vs show abcd2 && head -1 "$OUT" | grep -q "^$b" || fail "show abcd2 should find $b"

# restore replaces a pending deploy and says so.
mkdir -p "$PENDING/files"
echo '{"source": "deploy", "detail": "deploy by someone@laptop"}' >"$PENDING/deploy-info.json"
check_restore "$a"
grep -q 'replaces a pending deploy.*deploy by someone@laptop' "$OUT" || fail "restore should say it replaced the pending deploy"
[ -z "$(find "$OEM_DEPLOY_DIR" -maxdepth 1 -name '.restore-*')" ] || fail "restore left its temp folder behind"
rm -rf "$PENDING"

# deploy.sh on the laptop must compute the same version hash as the Pi.
h2=$(source "$DASH/deploy/lib/common.sh" && version_hash "${MIRROR[$id]}/manifest.json" "${MIRROR[$id]}/files")
[ "$h2" = "$h" ] || fail "deploy/lib/common.sh's version_hash differs from pi/lib/common.sh's"

# A missing deploy manifest is an error, not a crash.
mv "$MANIFEST" "$T/manifest.bak"
if vs snapshot --source local; then fail "snapshot without a deploy manifest should fail"; fi
mv "$T/manifest.bak" "$MANIFEST"

# Deploy manifest entries the store can't hold are left out, each with its
# reason; the rest is stored. Two entries for one file are an error.
M=$T/m
mkdir -p "$M"
jq -n --arg r "$ROOT" '[{file_name: "dashboard.py", file_path: "\($r)/dashboard.py", repo_path: "dash/dashboard.py"},
  {file_name: "odd\\name.py", file_path: "\($r)/odd.py", repo_path: "dash/odd.py"},
  {file_name: "rel.py", file_path: "relative/rel.py"},
  42,
  {file_name: "mkvi.dbc", file_path: "\($r)/mkvi.dbc", repo_path: "dash/mkvi.dbc"}]' >"$M/deploy-manifest.json"
OEM_DEPLOY_DIR=$M vs snapshot --source local --detail "bad entries" || fail "snapshot should store the good entries of a manifest with bad ones"
grep -q '^v1 (new, ' "$OUT" || fail "expected v1 from a manifest with bad entries"
grep -q '^WARNING: Entry 2 of .*(file_name "odd\\\\name.py", file_path ".*/odd.py") can.t be stored: file_name contains a backslash; leaving it out' "$ERR" &&
  grep -q '^WARNING: Entry 3 of .*: file_path is not an absolute path; leaving it out' "$ERR" &&
  grep -q '^WARNING: Entry 4 of .*(42) can.t be stored: it is not an object; leaving it out' "$ERR" ||
  fail "snapshot should name each bad entry and say what's wrong with it"
[ "$(jq -r 'map(.file_name) | join(" ")' "$M/versions/v1/manifest.json")" = "dashboard.py mkvi.dbc" ] ||
  fail "v1 should hold the two good entries"
OEM_DEPLOY_DIR=$M vs verify || fail "verify after storing a manifest with bad entries"
jq --arg r "$ROOT" '. + [{file_name: "again.py", file_path: "\($r)/dashboard.py"}]' "$M/deploy-manifest.json" >"$M/m" &&
  mv "$M/m" "$M/deploy-manifest.json"
if OEM_DEPLOY_DIR=$M vs snapshot --source local; then fail "two entries for one file_path should be an error"; fi
grep -q "more than one entry with file_path \".*/dashboard.py\"" "$ERR" || fail "the error should name the duplicate file_path"
echo '{"not": "a list"}' >"$M/deploy-manifest.json"
if OEM_DEPLOY_DIR=$M vs snapshot --source local; then fail "a manifest that isn't a list should be an error"; fi
[ "$(jq '.versions | length' "$M/versions/index.json")" -eq 1 ] || fail "failed snapshots shouldn't store anything"
echo "  commands: version arguments, list/show/current, errors, ambiguous prefixes, restore over a pending deploy, bad manifest entries ($((SECONDS - t)) s)"

# ------------------------------------------------------------------ 4. damaged stores are reported

STEP=damage t=$SECONDS
damaged() {  # damaged DESCRIPTION SHELL_EDIT: break a copy of the store; verify must fail
  rm -rf "$T/bad"
  cp -a "$OEM_DEPLOY_DIR" "$T/bad"
  (cd "$T/bad/versions" && eval "$2") || fail "couldn't damage the store: $1"
  if OEM_DEPLOY_DIR=$T/bad vs verify; then fail "verify missed: $1"; fi
  grep -q "BROKEN\|ERROR" "$OUT" "$ERR" || fail "verify should explain: $1"
}
set_max 6
for i in 1 2 3; do sed -i "$((i * 7))s/\$/ # damage test $i/" "${SLOT_PATH[dash]}"; snapshot deploy "for damage tests $i"; done
check_store
f=$(find "$VD" -mindepth 3 -path '*/patch/*' -type f | sort | head -1)   # a version with a patch
pv=$(basename "$(dirname "$(dirname "$f")")") pf=$(basename "$f")
pp=$(jq -r --arg v "$pv" '.versions[] | select(.id == $v) | .parent' "$INDEX")
f=$(find "$VD" -mindepth 3 -path '*/full/*' -type f | sort | head -1)    # one with a full copy
fv=$(basename "$(dirname "$(dirname "$f")")") ff=$(basename "$f")
[ -n "$pv" ] && [ -n "$fv" ] && [ "$pp" != null ] || fail "the store should have patches and full copies here"
damaged "an edited patch" "sed -i '\$s/./X/' '$pv/patch/$pf'"
grep -q "^  $pv  *BROKEN" "$OUT" || fail "verify should name $pv"
damaged "a missing full copy" "rm '$fv/full/$ff'"
damaged "a changed full copy" "echo extra >>'$fv/full/$ff'"
damaged "a wrong sha256 in a manifest" "jq '.[0].sha256 = (\"0\" * 64)' $fv/manifest.json >m && mv m $fv/manifest.json"
damaged "a damaged manifest.json" "echo '[{' >$fv/manifest.json"
damaged "a missing version folder" "rm -rf $fv"
damaged "a parent that doesn't exist" "jq '(.versions[] | select(.id == \"$pv\") | .parent) = \"v999\"' index.json >i && mv i index.json"
damaged "a parent loop" "jq '(.versions[] | select(.id == \"$pp\") | .parent) = \"$pv\"' index.json >i && mv i index.json"
grep -q "loop" "$OUT" || fail "verify should report the parent loop"
damaged "a wrong version hash" "jq '(.versions[0].hash) = (\"f\" * 64)' index.json >i && mv i index.json"
damaged "two versions with the same hash" "jq '.versions[1].hash = .versions[0].hash' index.json >i && mv i index.json"
damaged "index.json that isn't JSON" "echo '{' >index.json"
damaged "an index entry without a hash" "jq 'del(.versions[0].hash)' index.json >i && mv i index.json"
grep -q "^  ${MODEL_IDS[0]}  *BROKEN: its hash isn't 64 hex digits" "$OUT" && ! grep -q "subscript" "$ERR" ||
  fail "verify should report the version without a hash as BROKEN (not crash)"

# A lost index.json: the store's other commands refuse to run and change
# nothing; the next snapshot (the boot's) moves the whole store aside, losing
# nothing, and starts a new one.
store_state() { find "$1" \( -name .lock -o -name .pending.lock \) -prune -o -printf '%P %y %s %T@\n' | sort; }
for how in missing empty "a list" "without versions"; do
  rm -rf "$T/bad" "$T/bad-before"
  cp -a "$OEM_DEPLOY_DIR" "$T/bad"
  case $how in
    missing) rm "$T/bad/versions/index.json" ;;
    empty) : >"$T/bad/versions/index.json" ;;
    "a list") echo '[]' >"$T/bad/versions/index.json" ;;
    "without versions") echo '{"next_id": 9, "current": "v1"}' >"$T/bad/versions/index.json" ;;
  esac
  cp -a "$T/bad/versions" "$T/bad-before"
  before=$(store_state "$T/bad")
  for cmd in list "show $MODEL_CUR" current verify "restore $MODEL_CUR"; do
    # shellcheck disable=SC2086
    if OEM_DEPLOY_DIR=$T/bad vs $cmd; then fail "$cmd should refuse to run with index.json $how"; fi
    grep -q "^ERROR: .*/versions/index.json is missing or damaged; nothing was changed" "$ERR" ||
      fail "$cmd with index.json $how should say it's missing or damaged"
  done
  [ "$(store_state "$T/bad")" = "$before" ] || fail "the commands changed the store with index.json $how"
  OEM_DEPLOY_DIR=$T/bad vs snapshot --source local --detail "after losing index.json" || fail "snapshot with index.json $how"
  grep -q '^v1 (new, ' "$OUT" || fail "snapshot with index.json $how should start a new store at v1"
  aside=$(find "$T/bad" -maxdepth 1 -name 'versions.damaged-*')
  [ -n "$aside" ] && [ "$(wc -l <<<"$aside")" -eq 1 ] || fail "expected one versions.damaged-* folder, got: $aside"
  grep -q "^WARNING: Moved the whole version store to $aside (nothing was deleted)" "$ERR" ||
    fail "snapshot should say where it moved the store"
  diff -r -x .lock "$T/bad-before" "$aside" >/dev/null || fail "the store moved aside isn't the whole old store"
  OEM_DEPLOY_DIR=$T/bad vs verify && grep -q "All 1 versions OK" "$OUT" || fail "the new store should hold v1"
done
# An empty index.json without any version folders is just a new store.
rm -rf "$T/fresh"; mkdir -p "$T/fresh/versions"; : >"$T/fresh/versions/index.json"
OEM_DEPLOY_DIR=$T/fresh vs list && [ "$(cat "$OUT")" = "No versions stored yet." ] || fail "list with an empty index.json and no versions"
cp "$MANIFEST" "$T/fresh/"
OEM_DEPLOY_DIR=$T/fresh vs snapshot --source local && grep -q '^v1 (new, ' "$OUT" && ! grep -q WARNING "$ERR" ||
  fail "snapshot with an empty index.json and no versions should just start the store"
# A new snapshot still works when an old version is damaged (it's just not used as a base).
rm -rf "$T/bad"; cp -a "$OEM_DEPLOY_DIR" "$T/bad"
echo garbage >"$T/bad/versions/$fv/full/$ff"
printf '# with a damaged store\n' >>"${SLOT_PATH[dash]}"
OEM_DEPLOY_DIR=$T/bad vs snapshot --source local --detail x || fail "snapshot should still work with a damaged version"
grep -q "^v[0-9]* (new" "$OUT" || fail "snapshot with a damaged version should store a new version"
echo "  damage: verify catches edited/missing files, bad manifests, missing or looping parents, bad or missing hashes; a lost index.json loses nothing ($((SECONDS - t)) s)"

# ------------------------------------------------------------------ 5. locking and recovery

STEP=locking t=$SECONDS
snapshot deploy "with a damaged store, in the real one"
check_store
# A rollback waits for a snapshot that holds the lock.
rm -f "$T/locked"
flock "$VD/.lock" bash -c "touch '$T/locked'; sleep 2" &
until [ -f "$T/locked" ]; do sleep 0.05; done
t0=$EPOCHREALTIME
check_restore "$MODEL_CUR"
wait
t1=$EPOCHREALTIME
grep -q "waiting for it" "$ERR" || fail "restore should say it's waiting for the lock"
[ $((${t1//[.,]/} - ${t0//[.,]/})) -ge 1000000 ] || fail "restore didn't wait for the lock"
rm -rf "$PENDING"
# Snapshots and rollbacks at the same time: they take turns. Each snapshot
# stores a new version of what was on disk when it started.
for i in 1 2 3; do
  printf '# concurrent %s\n' "$i" >>"${SLOT_PATH[dash]}"
  id=v$MODEL_NEXT dir=$T/snap/concurrent-$i
  mirror_state "$dir"
  bash "$ENGINE" snapshot --source deploy --detail "concurrent $i" >"$T/c1" 2>&1 &
  bash "$ENGINE" restore "$MODEL_CUR" >"$T/c2" 2>&1 || fail "concurrent restore failed: $(cat "$T/c2")"
  wait $! || fail "concurrent snapshot failed: $(cat "$T/c1")"
  grep -q "^$id (new, " "$T/c1" || fail "concurrent snapshot $i should have stored $id: $(cat "$T/c1")"
  model_new "$id" "$dir"
  rm -rf "$PENDING"
  vs verify || fail "verify after concurrent snapshot + restore"
done
check_store

STEP=recovery
# Leftovers of runs cut off part-way are cleaned up (or finished) by the next run.
a=${MODEL_IDS[0]} b=${MODEL_IDS[1]}
cp -a "$VD/$b" "$VD/v$MODEL_NEXT"                     # renamed into place, index never updated
mkdir -p "$VD/v$((MODEL_NEXT + 1)).tmp" "$VD/.work-dead"; echo '{' >"$INDEX.tmp"
mv "$VD/$a" "$VD/$a.old"; mkdir "$VD/$a.tmp"           # cut off between swap_dir's renames
cp -a "$VD/$b" "$VD/$b.old"                            # cut off just before the old copy was deleted
vs verify || fail "verify after simulated interruptions"
grep -q "Restoring $a" "$ERR" && grep -q "Removing v$MODEL_NEXT" "$ERR" || fail "recovery should say what it did"
check_store

echo "  locking and recovery: waits for the lock, concurrent runs, leftovers of interrupted runs cleaned up ($((SECONDS - t)) s)"

# ------------------------------------------------------------------ 6. power cuts at every step

# Every change to the store is a rename. Kill the engine just before and just
# after each rename (mv) of a snapshot that also prunes and re-points versions:
# after every crash the store must verify with nothing lost and stored_bytes
# right, and the next run must finish the job: the new version current, no
# more than max_versions left.
STEP=crash t=$SECONDS
C=$T/crash
mkdir -p "$C/bin" "$C/root" "$C/pre" "$C/new/files"
cat >"$C/bin/mv" <<'EOF_MV'
#!/usr/bin/env bash
n=$(($(cat "$CRASH_COUNT") + 1))
echo "$n" >"$CRASH_COUNT"
if [ "$n" = "$CRASH_AT" ] && [ "$CRASH_WHEN" = before ]; then kill -9 "$PPID"; exit 1; fi
command -p mv "$@" || exit
if [ "$n" = "$CRASH_AT" ] && [ "$CRASH_WHEN" = after ]; then kill -9 "$PPID"; fi
EOF_MV
chmod +x "$C/bin/mv"
# Each case works in its own folder W: the store in W/deploy, output in W/out, W/err.
crash_snapshot() {  # crash_snapshot W AT WHEN: snapshot into W, killed at its AT-th mv
  echo 0 >"$1/count"
  # In a subshell so the shell's "Killed" notice goes to /dev/null.
  (CRASH_COUNT=$1/count CRASH_AT=$2 CRASH_WHEN=$3 PATH=$C/bin:$PATH OEM_DEPLOY_DIR=$1/deploy \
    bash "$ENGINE" snapshot --source deploy --detail crash >"$OUT" 2>"$ERR" || exit) 2>/dev/null
}
hashes_ok() {  # hashes_ok W: every version is one we know, with the hash it had
  local id h
  while read -r id h; do
    if [ "${C_HASH[$id]-}" != "$h" ] && { [ "$id" != "$C_NEW_ID" ] || [ "$h" != "$C_NEW_HASH" ]; }; then
      fail "$id has an unexpected hash"
    fi
  done < <(jq -r '.versions[] | "\(.id) \(.hash)"' "$1/deploy/versions/index.json")
}
crash_case() {  # crash_case AT WHEN: one power cut, then the next boot's snapshot
  local w=$C/case-$1-$2 OUT=$C/case-$1-$2/out ERR=$C/case-$1-$2/err STEP="crash at mv $1 ($2)"
  mkdir -p "$w"
  cp -a "$C/pre" "$w/deploy"
  if crash_snapshot "$w" "$1" "$2"; then fail "the crash didn't happen"; fi
  OEM_DEPLOY_DIR=$w/deploy vs verify || fail "verify after the crash"
  hashes_ok "$w"
  bytes_ok "$w/deploy/versions"
  OEM_DEPLOY_DIR=$w/deploy vs snapshot --source deploy --detail rerun || fail "snapshot after the crash"
  OEM_DEPLOY_DIR=$w/deploy vs verify || fail "verify after the snapshot that followed the crash"
  hashes_ok "$w"
  bytes_ok "$w/deploy/versions"
  [ "$(jq -r .current "$w/deploy/versions/index.json")" = "$C_NEW_ID" ] || fail "$C_NEW_ID should be current"
  [ "$(jq '.versions | length' "$w/deploy/versions/index.json")" -le 3 ] || fail "the next run didn't finish the prune"
  rm -rf "$w"
}
set_max 3
cp "$DASH/dashboard.py" "$C/root/app.py"
bin_write "$C/root/data.bin"
jq -n --arg r "$C/root" '[{file_name: "app.py", file_path: "\($r)/app.py", repo_path: "a/app.py"},
  {file_name: "data.bin", file_path: "\($r)/data.bin", repo_path: "a/data.bin"}]' >"$C/pre/deploy-manifest.json"
for v in base a b; do   # v1 in full; v2 and v3 each a diff against v1
  cp "$DASH/dashboard.py" "$C/root/app.py"
  [ "$v" = base ] || printf '# %s\n' "$v" >>"$C/root/app.py"
  OEM_DEPLOY_DIR=$C/pre vs snapshot --source deploy --detail "$v" || fail "crash test setup"
done
[ "$(jq -r '[.versions[].parent] | map(. // "-") | join(" ")' "$C/pre/versions/index.json")" = "- v1 v1" ] ||
  fail "crash test setup: expected v2 and v3 to be stored against v1"
declare -A C_HASH=()
while read -r id h; do C_HASH[$id]=$h; done < <(jq -r '.versions[] | "\(.id) \(.hash)"' "$C/pre/versions/index.json")
cp "$DASH/dashboard.py" "$C/root/app.py"
printf '# c\n' >>"$C/root/app.py"   # the new version: v4, which prunes v1 and re-points v2 and v3
cp "$C/root/app.py" "$C/root/data.bin" "$C/new/files/"
cp "$C/pre/deploy-manifest.json" "$C/new/manifest.json"
C_NEW_ID=v4 C_NEW_HASH=$(hash_of "$C/new")
mkdir -p "$C/plain"
cp -a "$C/pre" "$C/plain/deploy"
crash_snapshot "$C/plain" 0 none || fail "snapshot without a crash"
grep -q '^Pruned v1; v2 now stored as a diff vs v[0-9], v3 now stored' "$ERR" || fail "crash test: expected v2 re-pointed at a diff"
OEM_DEPLOY_DIR=$C/plain/deploy vs verify || fail "verify without a crash"
nmv=$(cat "$C/plain/count")
[ "$nmv" -ge 8 ] || fail "expected the snapshot to do more renames ($nmv)"
# The cases are independent, so run them a few at a time.
crashes=0 jobs_=()
for ((k = 1; k <= nmv; k++)); do
  for when in before after; do
    crash_case "$k" "$when" &
    jobs_+=($!) crashes=$((crashes + 1))
    if [ ${#jobs_[@]} -ge 8 ]; then wait "${jobs_[0]}" || fail "a crash case failed (see above)"; jobs_=("${jobs_[@]:1}"); fi
  done
done
for pid in "${jobs_[@]}"; do wait "$pid" || fail "a crash case failed (see above)"; done
echo "  power cuts: crashed before and after each of the $nmv renames of a pruning snapshot ($crashes crashes, $((SECONDS - t)) s)"

# ------------------------------------------------------------------ 7. the check after writing a version

# A version written as diffs is rebuilt and checked before it's used; if that
# fails it's stored in full instead. A patch that corrupts what it builds from
# a version being written (vN.tmp/patch/...) forces both fallbacks: a new
# version, and versions re-pointed by the prune.
STEP=fallbacks t=$SECONDS
F=$T/fallback
mkdir -p "$F/bin"
cat >"$F/bin/patch" <<'EOF_PATCH'
#!/usr/bin/env bash
# The real patch, plus: PATCH_LOG logs each output file; with PATCH_BREAK, the
# output for a version being written gets an extra byte.
out= prev=
for a; do [ "$prev" != -o ] || out=$a; prev=$a; done
[ -z "${PATCH_LOG:-}" ] || echo "$out" >>"$PATCH_LOG"
command -p patch "$@" || exit
if [ -n "${PATCH_BREAK:-}" ]; then case ${!#} in */v[0-9]*.tmp/patch/*) printf x >>"$out" ;; esac; fi
EOF_PATCH
chmod +x "$F/bin/patch"
cp -a "$C/pre" "$F/deploy"
PATH=$F/bin:$PATH PATCH_BREAK=1 OEM_DEPLOY_DIR=$F/deploy vs snapshot --source deploy --detail fallback ||
  fail "snapshot with a patch that breaks the check"
grep -q "^v4 (new, .* full)$" "$OUT" || fail "v4 should be stored in full"
grep -q "^WARNING: v4 didn't rebuild correctly from diffs against v[0-9] (app.py (stored as 'patch') doesn't match its sha256); storing it in full" "$ERR" ||
  fail "snapshot should say why v4 is stored in full"
grep -q "^WARNING: v2 didn't rebuild correctly from diffs against v[0-9] .*; keeping it in full" "$ERR" &&
  grep -q "^Pruned v1; v2 now stored in full, v3 now stored in full$" "$ERR" || fail "re-pointing should keep v2 and v3 in full"
OEM_DEPLOY_DIR=$F/deploy vs verify || fail "verify after the fallbacks"
hashes_ok "$F"
bytes_ok "$F/deploy/versions"
[ "$(jq -c '[.versions[] | [.id, .parent]]' "$F/deploy/versions/index.json")" = '[["v2",null],["v3",null],["v4",null]]' ] ||
  fail "the versions that fell back should have no parent"

# Where the work cache goes: the rebuilt files (patch's output) are on the tmpfs, or in the store.
grep -q '"patch"' "$VD"/v*/manifest.json || fail "expected patches in the main store"
if [ -n "$SHM" ]; then
  PATH=$F/bin:$PATH PATCH_LOG=$F/log OEM_VERSIONS_TMP=$SHM vs verify || fail "verify with the cache on tmpfs"
  [ -s "$F/log" ] && ! grep -qv "^$SHM/oem-versions\.[^/]*/" "$F/log" || fail "with a tmpfs, the cache should be there: $(head -3 "$F/log")"
  [ -z "$(ls -A "$SHM")" ] || fail "the engine left its work cache behind in $SHM"
fi
rm -f "$F/log"
PATH=$F/bin:$PATH PATCH_LOG=$F/log vs verify || fail "verify with the cache in the store"
[ -s "$F/log" ] && ! grep -qv "^$VD/\.work-[^/]*/cache/" "$F/log" || fail "without a tmpfs, the cache should be in the store: $(head -3 "$F/log")"
echo "  checks after writing: a new version and re-pointed ones fall back to full storage; the cache is where it should be ($((SECONDS - t)) s)"

# ------------------------------------------------------------------ 8. file modes, and a rollback through the real startup.sh

# Each file's mode is recorded and restored, whether the file is stored in
# full, as a patch or unchanged, and survives re-pointing. Then a rollback is
# applied by pi/startup.sh, the boot script (offline: no GitHub sync, a fake
# python for the dashboard).
STEP=modes t=$SECONDS
B=$T/boot
BH=$B/home
mkdir -p "$B/root" "$B/bin" "$BH/deploy"
cat >"$B/bin/python" <<'EOF_PY'
#!/usr/bin/env bash
echo "$PWD $*" >>"${0%/*}/ran"
EOF_PY
chmod +x "$B/bin/python"
jq -n --arg b "$B" '{repo_url: "file:///nowhere/not-github.git", branch: "main", network_timeout_s: 2,
  dashboard_command: "\($b)/bin/python dashboard.py", dashboard_dir: "\($b)/root",
  restart_dashboard_on_exit: false, restart_delay_s: 1, max_versions: 3, github_users: []}' >"$B/startup_config.json"
# bvs ARGS...: the engine on this Pi's store.
bvs() { HOME=$BH OEM_DEPLOY_DIR=$BH/deploy OEM_STARTUP_CONFIG=$B/startup_config.json vs "$@"; }
modes() { (cd "$1" && stat -c '%n %a' tool.sh data.txt dashboard.py); }
{ printf '#!/usr/bin/env bash\n'; seq 1 60 | sed 's/^/# line /'; } >"$B/root/tool.sh"
printf 'some data\n' >"$B/root/data.txt"
cp "$DASH/dashboard.py" "$B/root/dashboard.py"
chmod 755 "$B/root/tool.sh"; chmod 640 "$B/root/data.txt"; chmod 644 "$B/root/dashboard.py"
jq -n --arg r "$B/root" '[{file_name: "tool.sh", file_path: "\($r)/tool.sh", repo_path: "t/tool.sh"},
  {file_name: "data.txt", file_path: "\($r)/data.txt", repo_path: "t/data.txt"},
  {file_name: "dashboard.py", file_path: "\($r)/dashboard.py", repo_path: "t/dashboard.py"}]' >"$BH/deploy/deploy-manifest.json"
bvs snapshot --source deploy --detail one && grep -q '^v1 (new' "$OUT" || fail "modes: v1"
echo 'echo two' >>"$B/root/tool.sh"
mkdir -p "$B/v2"; cp -p "$B/root"/* "$B/v2/"
bvs snapshot --source deploy --detail two && grep -q '^v2 (new, .* patch vs v1)' "$OUT" || fail "modes: v2 should be a patch vs v1"
[ "$(jq -c 'map([.file_name, .stored, .mode])' "$BH/deploy/versions/v2/manifest.json")" = \
  '[["tool.sh","patch","755"],["data.txt","same","640"],["dashboard.py","same","644"]]' ] ||
  fail "modes: v2's manifest should record each file's mode: $(cat "$BH/deploy/versions/v2/manifest.json")"
echo 'echo three' >>"$B/root/tool.sh"; echo more >>"$B/root/data.txt"
chmod 700 "$B/root/tool.sh"; chmod 600 "$B/root/data.txt"
bvs snapshot --source deploy --detail three && grep -q '^v3 (new' "$OUT" || fail "modes: v3"
bvs show v2 && grep -q '^  [0-9a-f]\{8\}  [^ ]* *755   patch vs v1' "$OUT" || fail "modes: show v2 should show tool.sh's mode"
bvs restore v2 || fail "modes: restore v2"
[ "$(modes "$BH/deploy/pending/files")" = "$(modes "$B/v2")" ] || fail "modes: restored v2 has modes $(modes "$BH/deploy/pending/files" | tr '\n' ' ')"
# A manifest from before modes were recorded: the files come back 644.
jq 'map(del(.mode))' "$BH/deploy/versions/v1/manifest.json" >"$B/m" && mv "$B/m" "$BH/deploy/versions/v1/manifest.json"
mv "$BH/deploy/pending" "$B/pending-v2"
bvs restore v1 && [ "$(stat -c %a "$BH/deploy/pending/files"/* | sort -u)" = 644 ] || fail "modes: a manifest without modes should restore as 644"
bvs verify || fail "modes: verify with a manifest without modes"
rm -rf "$BH/deploy/pending"; mv "$B/pending-v2" "$BH/deploy/pending"

# The rollback to v2 staged above, applied by the boot script.
STEP=boot
rc=0
HOME=$BH OEM_DEPLOY_DIR=$BH/deploy OEM_STARTUP_CONFIG=$B/startup_config.json OEM_AUTHORIZED_KEYS=$BH/.ssh/authorized_keys \
  bash "$DASH/pi/startup.sh" >"$OUT" 2>"$ERR" || rc=$?
[ "$rc" -eq 0 ] || fail "startup.sh exited with $rc"
grep -q "Applying the pending rollback: rollback to v2" "$ERR" && grep -q "Applied the rollback: 3 file(s) installed" "$ERR" ||
  fail "startup.sh should apply the rollback"
grep -q "^versions: v2 (already stored)$" "$ERR" || fail "the boot's snapshot should find v2"
[ "$(cat "$B/bin/ran")" = "$B/root dashboard.py" ] || fail "startup.sh should have run dashboard_command in dashboard_dir"
[ ! -e "$BH/deploy/pending" ] && [ ! -e "$BH/deploy/.applying" ] || fail "startup.sh should have used up the rollback"
for f in tool.sh data.txt dashboard.py; do cmp -s "$B/v2/$f" "$B/root/$f" || fail "startup.sh installed the wrong $f"; done
[ "$(modes "$B/root")" = "$(modes "$B/v2")" ] || fail "the installed files have modes $(modes "$B/root" | tr '\n' ' ')"
[ "$("$B/root/tool.sh")" = two ] || fail "the rolled-back tool.sh should run"
[ "$(bvs current && cat "$OUT")" = v2 ] || fail "current should be v2 after the boot"
[ ! -e "$BH/.ssh/authorized_keys" ] || fail "the key sync shouldn't have written anything"

# A prune re-points v2 (its parent v1 goes): its modes come along.
STEP=modes
echo 'echo four' >>"$B/root/tool.sh"
bvs snapshot --source deploy --detail four && grep -q '^Pruned v1; v2 now ' "$ERR" || fail "modes: v4 should prune v1 and re-point v2"
bvs restore v2 && [ "$(modes "$BH/deploy/pending/files")" = "$(modes "$B/v2")" ] || fail "modes: re-pointed v2 lost its modes"
rm -rf "$BH/deploy/pending"
# A restore waits for the pending-deploy lock (startup.sh holds it while it
# claims a deploy), unless OEM_PENDING_LOCKED says the caller holds it
# (deploy/versions.sh rollback does).
rm -f "$B/locked"
flock "$BH/deploy/.pending.lock" bash -c "touch '$B/locked'; sleep 3" &
holder=$!
until [ -f "$B/locked" ]; do sleep 0.05; done
OEM_PENDING_LOCKED=1 bvs restore v3 || fail "restore with OEM_PENDING_LOCKED"
kill -0 "$holder" 2>/dev/null || fail "with OEM_PENDING_LOCKED set, restore shouldn't wait for the pending lock"
t0=$EPOCHREALTIME
bvs restore v3 || fail "restore while the pending lock is held"
t1=$EPOCHREALTIME
wait "$holder"
[ $((${t1//[.,]/} - ${t0//[.,]/})) -ge 1000000 ] || fail "restore didn't wait for the pending lock"
echo "  modes and boot: modes kept (patch, same, re-pointed, old manifests), a rollback applied by startup.sh, restore waits for the pending lock ($((SECONDS - t)) s)"

STEP=done
echo "PASS ($((SECONDS - start)) s)"
