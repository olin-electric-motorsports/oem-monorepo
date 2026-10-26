#!/usr/bin/env bash
# The Pi's version store: every code state the Pi runs (a deploy, a rollback,
# a GitHub sync, or files edited on the Pi) is kept as a version you can roll
# back to. startup.sh takes a snapshot on every boot; deploy/versions.sh on
# the laptop lists versions and rolls back.
#
#   versions.sh snapshot --source SOURCE [--detail TEXT]
#                                store the files listed in ~/deploy/deploy-manifest.json
#                                as they are now (SOURCE: deploy, github, rollback, local)
#   versions.sh list [-n N]      newest first (* = current); only the N newest
#   versions.sh show VERSION     details and files of one version
#   versions.sh restore VERSION  stage VERSION as ~/deploy/pending, a rollback that
#                                the next boot applies like a deploy
#   versions.sh verify           rebuild every version and check all its hashes
#   versions.sh current          print the current version's id
#
# VERSION is v12, 12, or the start of a version hash (at least 4 hex digits).
#
# Versions live in ~/deploy/versions. index.json lists them with their hashes
# and parents, so a code state that's already stored is recognised without
# rebuilding anything. vN/ holds a version's files: each in full, as a diff
# against the same file in vN's parent, or nothing if it's unchanged, and a
# manifest.json with each file's sha256 and mode (a rollback restores both).
# The parent is whichever version gives the smallest diffs. Only the newest
# max_versions (startup_config.json, default 10) are kept; the current one is
# never deleted.
#
# The folders can't be used without index.json (it says which ones are
# versions and what each is stored against), so a missing or damaged one
# never gets them deleted: snapshot moves the whole store aside to
# ~/deploy/versions.damaged-<time> and starts a new one; the other commands
# refuse to run.

set -euo pipefail
# shellcheck source=lib/common.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/common.sh"

# Byte semantics everywhere: ${#var} counts bytes, sorting and regexes don't
# depend on the locale.
export LC_ALL=C

INDEX=$VERSIONS_DIR/index.json
# What every reader and writer of index.json requires of it.
INDEX_CHECK='if type == "object" and (.versions | type) == "array" then . else error("not an index") end'
LOCK_WAIT_S=60
DEFAULT_MAX_VERSIONS=10
SOURCES="deploy github rollback local"
US=$'\x1f'   # field separator for jq -> bash (unlike tab, keeps empty fields)
# The work cache goes here if it's in RAM (tmpfs), to spare the SD card.
# OEM_VERSIONS_TMP is only for testing.
TMP_ROOT=${OEM_VERSIONS_TMP:-/tmp}

# index.json, loaded by load_index. IDS is oldest first.
IDS=()
NEXT_ID=1 CURRENT=
declare -A PARENT=() HASH=() CREATED=() SOURCE=() DETAIL=() SBYTES=()

# Version manifests, loaded by load_manifests and keyed "LABEL/FILE_NAME"
# (LABEL is a version id): M_PATH, M_REPO, M_SHA, M_SIZE, M_STORED, M_MODE.
# M_FILES[LABEL] lists the file names, one per line. M_BYPATH["LABEL/FILE_PATH"]
# is the file name for a path. M_DIR[LABEL] is the folder holding the files.
# M_TOTAL adds up the sizes loaded of files not stored as "same" (rebuilding
# those writes new files; the others are hardlinks), M_BIGGEST is the
# biggest version loaded.
declare -A M_FILES=() M_PATH=() M_REPO=() M_SHA=() M_SIZE=() M_STORED=() M_MODE=() M_BYPATH=() M_DIR=()
M_TOTAL=0 M_BIGGEST=0

# Rebuilt versions: $CACHE/ID/FILE_NAME, each version rebuilt at most once per
# run (see make_cache). BROKEN[ID] says why one can't be rebuilt.
WORK= CACHE= CACHE_TMP= RESTORE_TMP=
declare -A DONE=() BUSY=() BROKEN=() TEXT=() WARNED=()

# The files being stored (a new snapshot, or a version being re-pointed):
# S_NAMES in manifest order; S_PATH/S_SHA/S_SIZE/S_MODE/S_FILE (content) by name.
S_NAMES=()
declare -A S_PATH=() S_SHA=() S_SIZE=() S_MODE=() S_FILE=()
# choose_parent's result: BEST ("" = full), BEST_BYTES, PLAN_KIND/PLAN_DIFF by name.
BEST= BEST_BYTES=0
declare -A PLAN_KIND=() PLAN_DIFF=()

cleanup() {
  local d
  local -a dirs=()
  for d in "$WORK" "$CACHE_TMP" "$RESTORE_TMP"; do [ -z "$d" ] || dirs+=("$d"); done
  [ ${#dirs[@]} -eq 0 ] || rm -rf -- "${dirs[@]}"
}
trap cleanup EXIT

# ------------------------------------------------------------------ helpers

# human VAR BYTES: VAR=310B, 1.2K, 14K, 3.4M. (Sets a variable rather than
# printing, to save a subshell per call on the Pi Zero.)
human() {
  local -n _h=$1
  local b=$2 unit=K div=1024 t
  if [ "$b" -lt 1024 ]; then _h=${b}B; return; fi
  if [ "$b" -ge 1048576 ]; then unit=M div=1048576; fi
  t=$(((b * 10 + div / 2) / div))
  if [ "$t" -lt 100 ]; then _h=$((t / 10)).$((t % 10))$unit; else _h=$(((b + div / 2) / div))$unit; fi
}

describe() {  # describe VAR BYTES PARENT: VAR="1.2K patch vs v12" / "14K full"
  local -n _d=$1
  local size
  human size "$2"
  if [ -n "$3" ]; then _d="$size patch vs $3"; else _d="$size full"; fi
}

tilde() { echo "${1/#"$HOME"/\~}"; }

join() {  # join SEP ITEM...
  local sep=$1 out=${2-}
  shift 2 || return 0
  for item; do out+=$sep$item; done
  echo "$out"
}

max_versions() {
  local m=$DEFAULT_MAX_VERSIONS
  if [ -f "$CONFIG" ]; then m=$(cfg max_versions "$DEFAULT_MAX_VERSIONS") || m=$DEFAULT_MAX_VERSIONS; fi
  if ! [[ $m =~ ^[0-9]+$ ]]; then
    warn "max_versions in $CONFIG isn't a whole number ($m); using $DEFAULT_MAX_VERSIONS"
    m=$DEFAULT_MAX_VERSIONS
  fi
  m=$((10#$m))
  if [ "$m" -lt 1 ]; then warn "max_versions must be at least 1; using 1"; m=1; fi
  echo "$m"
}

# Make new files durable before anything refers to them: the Pi often loses
# power without shutting down, and a renamed-into-place folder whose files
# never reached the SD card would be a broken version.
sync_fs() { sync -f "$1" 2>/dev/null || sync; }

# ------------------------------------------------------------------ locking and recovery

# Every command that reads or changes version folders holds this lock, so a
# boot-time snapshot and a rollback over ssh never interleave. With "aside",
# a store whose index.json is lost is moved aside (see damaged_index).
lock_store() {
  mkdir -p "$VERSIONS_DIR"
  exec 9>"$VERSIONS_DIR/.lock"
  if ! flock -n 9; then
    log "Another versions.sh is running; waiting for it (up to $LOCK_WAIT_S s)..."
    flock -w "$LOCK_WAIT_S" 9 || die "gave up waiting for $VERSIONS_DIR/.lock after $LOCK_WAIT_S s"
  fi
  recover "${1-}"
  WORK=$(mktemp -d "$VERSIONS_DIR/.work-XXXXXX")
}

have_folders() {  # true if any version folder (vN, vN.tmp, vN.old) exists
  local d
  for d in "$VERSIONS_DIR"/v[0-9]*; do [ ! -e "$d" ] || return 0; done
  return 1
}

# damaged_index [aside]: index.json is missing, empty or not an index while
# version folders exist. Without it nobody knows which folders are versions
# or what each is stored against, and deleting them as leftovers would lose
# every version. So the read-only commands and restore refuse to run, and
# snapshot (the boot's, which must keep working unattended) moves the whole
# store aside and starts a new one.
damaged_index() {
  local aside k=1 d
  local -a items=()
  [ "${1-}" = aside ] ||
    die "$(tilde "$INDEX") is missing or damaged; nothing was changed. The next snapshot (every boot takes one) moves $(tilde "$VERSIONS_DIR") aside to $(tilde "$DEPLOY_DIR")/versions.damaged-<date> and starts a new version store."
  aside=$DEPLOY_DIR/versions.damaged-$(date +%Y%m%d-%H%M%S)
  d=$aside
  while [ -e "$aside" ]; do aside=$d.$k k=$((k + 1)); done
  # Everything but .lock: this run holds it, and others may be waiting on it.
  for d in "$VERSIONS_DIR"/* "$VERSIONS_DIR"/.[!.]*; do
    [ -e "$d" ] && [ "$d" != "$VERSIONS_DIR/.lock" ] || continue
    items+=("$d")
  done
  mkdir "$aside"
  [ ${#items[@]} -eq 0 ] || mv -- "${items[@]}" "$aside/"
  warn "$(tilde "$INDEX") is missing or damaged, so the versions stored with it can't be used."
  warn "Moved the whole version store to $(tilde "$aside") (nothing was deleted) and started a new one."
}

# Clean up after a run that was cut off (e.g. by a power cut). Every change to
# the store keeps it valid at each step, so this only has to drop leftovers
# and finish a folder swap (see swap_dir). Called with the lock held.
recover() {
  local d base id ids known=" " b json=
  local -a old=() olds=()
  local -A bytes=()
  if [ -s "$INDEX" ]; then
    if ! ids=$(jq -r "$INDEX_CHECK"' | [.versions[].id | tostring] | join(" ")' "$INDEX" 2>/dev/null); then
      damaged_index "${1-}"
      return 0
    fi
    known+="$ids "
  elif have_folders; then
    damaged_index "${1-}"
    return 0
  fi
  rm -f "$INDEX.tmp"
  for d in "$VERSIONS_DIR"/v* "$VERSIONS_DIR"/.work-*; do
    [ -e "$d" ] || continue
    base=${d##*/}
    if [[ $base == .work-* ]]; then rm -rf "$d"; continue; fi
    id=${base%.*}   # v7.old, v7.tmp -> v7
    [[ $id =~ ^v[0-9]+$ ]] || continue
    if [[ $known != *" $id "* ]]; then
      log "Removing $base (not in index.json; left over from an interrupted snapshot or prune)"
      rm -rf "$d"
    elif [ "$base" = "$id.old" ] && [ ! -d "$VERSIONS_DIR/$id" ]; then
      log "Restoring $id (rewriting it was interrupted)"
      mv "$d" "$VERSIONS_DIR/$id"
    elif [ "$base" = "$id.old" ]; then
      old+=("$id") olds+=("$d")
    elif [ "$base" != "$id" ]; then
      rm -rf "$d"
    fi
  done
  [ ${#old[@]} -gt 0 ] || return 0
  # vN was swapped in, but the index edit that goes with it may not have
  # happened: count its stored bytes again, then drop vN.old.
  log "Finishing the interrupted rewrite of ${old[*]}"
  for id in "${old[@]}"; do bytes[$id]=0; done
  while read -r b d; do
    id=${d%/*}
    bytes[$id]=$((${bytes[$id]} + b))
  done < <(cd "$VERSIONS_DIR" && find "${old[@]/%//full}" "${old[@]/%//patch}" -type f -printf '%s %h\n' 2>/dev/null)
  for id in "${old[@]}"; do json+="\"$id\": ${bytes[$id]},"; done
  index_edit --argjson b "{${json%,}}" '.versions |= map(if $b[.id | tostring] != null then .stored_bytes = $b[.id | tostring] else . end)'
  rm -rf -- "${olds[@]}"
}

# make_cache NEED: choose CACHE for rebuilt versions that take up to NEED
# bytes: /tmp if it's in RAM (tmpfs) with room to spare, so they never wear
# the SD card, else WORK. Files are hardlinked within it (see rebuild). Out
# of room it would make good versions look damaged, hence the margin.
make_cache() {
  local type avail bsize
  [ -z "$CACHE" ] || return 0
  if read -r type avail bsize < <(stat -f -c '%T %a %S' "$TMP_ROOT" 2>/dev/null) && [ "$type" = tmpfs ] &&
    [ $((avail * bsize)) -ge $(($1 + $1 / 2 + 4194304)) ] && CACHE_TMP=$(mktemp -d "$TMP_ROOT/oem-versions.XXXXXX"); then
    CACHE=$CACHE_TMP
  else
    CACHE=$WORK/cache
    mkdir -p "$CACHE"
  fi
}

# ------------------------------------------------------------------ index.json

new_index() {
  printf '{\n  "next_id": 1,\n  "current": null,\n  "versions": []\n}\n' >"$INDEX.tmp" && [ -s "$INDEX.tmp" ] ||
    { rm -f "$INDEX.tmp"; die "couldn't create $INDEX"; }
  mv -f "$INDEX.tmp" "$INDEX"
}

# index_edit JQ_ARGS... FILTER: rewrite index.json with jq (temp file +
# rename). The result is checked first (jq -e: there is one, and it's an
# index), so a jq hiccup or a full SD card can't leave an empty or broken one.
index_edit() {
  jq -e "${@:1:$#-1}" "(${!#}) | $INDEX_CHECK" "$INDEX" >"$INDEX.tmp" && [ -s "$INDEX.tmp" ] ||
    { rm -f "$INDEX.tmp"; die "couldn't update $INDEX"; }
  mv -f "$INDEX.tmp" "$INDEX"
}

load_index() {
  local out id parent hash created source detail bytes
  IDS=() DUP_IDS=() NEXT_ID=1 CURRENT=
  PARENT=() HASH=() CREATED=() SOURCE=() DETAIL=() SBYTES=()
  if [ ! -s "$INDEX" ]; then
    if have_folders; then damaged_index; fi
    return 0
  fi
  out=$(jq -r 'def s: tostring | gsub("[[:cntrl:]]"; " ");
      '"$INDEX_CHECK"' |
      ([.next_id // 1, .current // ""] | map(s) | join("\u001f")),
      (.versions[] | [.id, .parent // "", .hash // "", .created // "", .source // "",
                      .detail // "", .stored_bytes // 0] | map(s) | join("\u001f"))' "$INDEX" 2>/dev/null) ||
    damaged_index
  {
    IFS=$US read -r NEXT_ID CURRENT
    while IFS=$US read -r id parent hash created source detail bytes; do
      [ -n "$id" ] || continue
      [[ $bytes =~ ^[0-9]+$ ]] || bytes=0
      if [ -n "${HASH[$id]+set}" ]; then DUP_IDS+=("$id"); continue; fi
      IDS+=("$id")
      PARENT[$id]=$parent HASH[$id]=$hash CREATED[$id]=$created
      SOURCE[$id]=$source DETAIL[$id]=$detail SBYTES[$id]=$bytes
    done
  } <<<"$out"
  [[ $NEXT_ID =~ ^[0-9]+$ ]] || NEXT_ID=1
}
DUP_IDS=()

# ------------------------------------------------------------------ manifests

MANIFEST_TSV='def s: tostring | gsub("[[:cntrl:]]"; " ");
  input_filename as $f | .[] |
  [$f, .file_name, .file_path, .repo_path // "", .sha256 // "", .size // 0, .stored // "", .mode // ""]
  | map(s) | join("\u001f")'

# load_manifests LABEL DIR [LABEL DIR ...]: read DIR/manifest.json for each
# (one jq call for all of them, unless one is damaged).
load_manifests() {
  local -A label_of=() total=()
  local -a files=()
  local out o f label name path repo sha size stored mode
  while [ $# -gt 0 ]; do
    label_of[$2/manifest.json]=$1
    M_DIR[$1]=$2 M_FILES[$1]=
    unset 'BROKEN[$1]'
    if [ -f "$2/manifest.json" ]; then
      files+=("$2/manifest.json")
    else
      BROKEN[$1]="its folder or manifest.json is missing"
    fi
    shift 2
  done
  [ ${#files[@]} -gt 0 ] || return 0
  if ! out=$(jq -r "$MANIFEST_TSV" "${files[@]}" 2>/dev/null); then
    out=
    for f in "${files[@]}"; do
      if o=$(jq -r "$MANIFEST_TSV" "$f" 2>/dev/null); then
        out+=$o$'\n'
      else
        BROKEN[${label_of[$f]}]="its manifest.json is damaged"
      fi
    done
  fi
  while IFS=$US read -r f name path repo sha size stored mode; do
    [ -n "$f" ] || continue
    label=${label_of[$f]}
    [[ $size =~ ^[0-9]+$ ]] || size=0
    # Manifests from before modes were recorded: what a plain copy would get.
    [[ $mode =~ ^[0-7]{3,4}$ ]] || mode=644
    M_FILES[$label]+=$name$'\n'
    M_PATH[$label/$name]=$path M_REPO[$label/$name]=$repo M_SHA[$label/$name]=$sha
    M_SIZE[$label/$name]=$size M_STORED[$label/$name]=$stored M_MODE[$label/$name]=$mode
    M_BYPATH[$label/$path]=$name
    [ "$stored" = same ] || M_TOTAL=$((M_TOTAL + size))
    total[$label]=$((${total[$label]:-0} + size))
  done <<<"$out"
  for label in "${!total[@]}"; do [ "${total[$label]}" -le "$M_BIGGEST" ] || M_BIGGEST=${total[$label]}; done
}

# load_all [EXTRA]: load every version's manifest, and make the cache (with
# room for EXTRA more bytes) and their (empty) folders in it.
load_all() {
  local -a args=() dirs=()
  local id
  for id in "${IDS[@]}"; do args+=("$id" "$VERSIONS_DIR/$id"); done
  M_TOTAL=0 M_BIGGEST=0
  [ ${#args[@]} -eq 0 ] || load_manifests "${args[@]}"
  # Every version rebuilt, plus one version checked in full at a time (.check).
  make_cache $((M_TOTAL + M_BIGGEST + ${1:-0}))
  for id in "${IDS[@]}"; do dirs+=("$CACHE/$id"); done
  [ ${#dirs[@]} -eq 0 ] || mkdir -p -- "${dirs[@]}"
}

names_of() {  # names_of LABEL ARRAY: the file names of LABEL into ARRAY
  local -n _names=$2
  _names=()
  [ -z "${M_FILES[$1]-}" ] || mapfile -t _names <<<"${M_FILES[$1]%$'\n'}"
}

# ------------------------------------------------------------------ rebuilding versions

# recon ID: rebuild version ID into $CACHE/ID (parents first, once per run).
# Returns 1, with the reason in BROKEN[ID], if the store is damaged.
recon() {
  local id=$1
  [ -z "${DONE[$id]-}" ] || return 0
  [ -z "${BROKEN[$id]-}" ] || return 1
  if [ -n "${BUSY[$id]-}" ]; then BROKEN[$id]="its parents form a loop"; return 1; fi
  BUSY[$id]=1
  if rebuild "$id" "$CACHE/$id"; then DONE[$id]=1; fi
  unset 'BUSY[$id]'
  [ -n "${DONE[$id]-}" ]
}

# rebuild LABEL OUT: write LABEL's files into OUT from the folder M_DIR[LABEL]
# and its parent's rebuilt files, checking each against its sha256: files
# stored in full or as a patch are hashed; an unchanged ("same") one is a
# hardlink to the parent's copy, which was checked when the parent was
# rebuilt, so it only needs the same sha256 in both manifests. (Nothing ever
# writes to a cached file, so sharing them is safe.)
# Runs as a condition (set -e is off), so every step is checked by hand.
rebuild() {
  local id=$1 out=$2 dir=${M_DIR[$1]-} p=${PARENT[$1]-} name stored pf msg i
  local -a names sums full=() same=() hashed=()
  names_of "$id" names
  if [ -z "$dir" ]; then BROKEN[$id]="it isn't in the version store"; return 1; fi
  if [ ${#names[@]} -eq 0 ]; then BROKEN[$id]=${BROKEN[$id]:-"its manifest lists no files"}; return 1; fi
  [ -d "$out" ] || mkdir -p "$out" || { BROKEN[$id]="couldn't create $out"; return 1; }
  for name in "${names[@]}"; do
    stored=${M_STORED[$id/$name]}
    case $stored in
      full)
        [ -f "$dir/full/$name" ] || { BROKEN[$id]="full/$name is missing"; return 1; }
        full+=("$dir/full/$name") hashed+=("$name")
        continue ;;
      same | patch) ;;
      *) BROKEN[$id]="$name has an unknown storage type '$stored'"; return 1 ;;
    esac
    if [ -z "$p" ]; then BROKEN[$id]="$name is stored as '$stored' but $id has no parent"; return 1; fi
    if ! recon "$p"; then BROKEN[$id]="its parent $p is broken"; return 1; fi
    pf=${M_BYPATH[$p/${M_PATH[$id/$name]}]-}
    if [ -z "$pf" ]; then BROKEN[$id]="$name is stored as '$stored' but $p has no ${M_PATH[$id/$name]}"; return 1; fi
    if [ "$stored" = same ]; then
      if [ "${M_SHA[$id/$name]}" != "${M_SHA[$p/$pf]}" ]; then
        BROKEN[$id]="$name (stored as 'same') doesn't match its sha256"
        return 1
      fi
      if [ "$pf" = "$name" ]; then
        same+=("$CACHE/$p/$pf")
      else
        cp -l -- "$CACHE/$p/$pf" "$out/$name" || { BROKEN[$id]="couldn't link $p's $pf"; return 1; }
      fi
    elif msg=$(patch -s -f --reject-file=- --no-backup-if-mismatch -o "$out/$name" -- \
      "$CACHE/$p/$pf" "$dir/patch/$name" 2>&1); then
      hashed+=("$name")
    else
      BROKEN[$id]="patch/$name doesn't apply to $p's copy (${msg//$'\n'/ })"
      return 1
    fi
  done
  # One cp per kind of copy (the source names match), one sha256sum for all
  # files (it prints them in argument order): process starts are slow on a Pi Zero.
  if [ ${#full[@]} -gt 0 ] && ! cp -- "${full[@]}" "$out/"; then BROKEN[$id]="couldn't copy its full/ files"; return 1; fi
  if [ ${#same[@]} -gt 0 ] && ! cp -l -- "${same[@]}" "$out/"; then BROKEN[$id]="couldn't link files from $p"; return 1; fi
  [ ${#hashed[@]} -gt 0 ] || return 0
  mapfile -t sums < <(cd "$out" && sha256sum -- "${hashed[@]}" 2>/dev/null)
  for i in "${!hashed[@]}"; do
    name=${hashed[i]}
    if [ -z "${sums[i]-}" ] || [ "${sums[i]:0:64}" != "${M_SHA[$id/$name]}" ]; then
      BROKEN[$id]="$name (stored as '${M_STORED[$id/$name]}') doesn't match its sha256"
      return 1
    fi
    # Only text files are stored as patches, so that saves is_text a grep.
    [ "${M_STORED[$id/$name]}" != patch ] || TEXT[${sums[i]:0:64}]=yes
  done
}

# is_text SHA FILE: no NUL bytes, so diff and patch can handle it. Cached by content.
is_text() {
  if [ -z "${TEXT[$1]-}" ]; then
    local rc=0
    grep -qaP '\x00' -- "$2" 2>/dev/null || rc=$?
    # 0 = found a NUL, 1 = none; anything else (grep can't tell): binary, i.e. stored in full.
    if [ "$rc" -eq 1 ]; then TEXT[$1]=yes; else TEXT[$1]=no; fi
  fi
  [ "${TEXT[$1]}" = yes ]
}

usable_base() {  # usable_base ID: rebuild ID, warning (once) if it's damaged
  recon "$1" && return 0
  if [ -z "${WARNED[$1]-}" ]; then
    warn "$1 is damaged (${BROKEN[$1]}); not using it as a base. Run versions.sh verify."
    WARNED[$1]=1
  fi
  return 1
}

# ------------------------------------------------------------------ storing versions

# choose_parent CANDIDATE... (newest first): find the cheapest base for the S_*
# files. Per file: unchanged 0; a changed text file the smaller of its diff
# and its size; anything else (new path, binary) its size. Ties go to the
# newer candidate; a version that saves nothing over full storage gets no parent.
# A candidate is only rebuilt when a diff against it is needed (the check
# after writing rebuilds the winner anyway).
# Sets BEST ("" = full), BEST_BYTES, PLAN_KIND (full|patch|same), PLAN_DIFF.
choose_parent() {
  local p n pn cost d rc
  local -A kind=() dif=()
  BEST= BEST_BYTES=0 PLAN_KIND=() PLAN_DIFF=()
  for n in "${S_NAMES[@]}"; do
    BEST_BYTES=$((BEST_BYTES + ${S_SIZE[$n]})) PLAN_KIND[$n]=full
  done
  for p in "$@"; do
    [ "$BEST_BYTES" -gt 0 ] || break   # nothing beats free
    cost=0 kind=() dif=()
    for n in "${S_NAMES[@]}"; do
      kind[$n]=full
      pn=${M_BYPATH[$p/${S_PATH[$n]}]-}
      if [ -z "$pn" ]; then
        :
      elif [ "${M_SHA[$p/$pn]}" = "${S_SHA[$n]}" ]; then
        kind[$n]=same
      elif is_text "${S_SHA[$n]}" "${S_FILE[$n]}"; then
        usable_base "$p" || continue 2
        if is_text "${M_SHA[$p/$pn]}" "$CACHE/$p/$pn"; then
          rc=0
          d=$(diff -a -u --label "${S_PATH[$n]}" --label "${S_PATH[$n]}" -- \
            "$CACHE/$p/$pn" "${S_FILE[$n]}") || rc=$?
          [ "$rc" -eq 1 ] || die "diff failed (exit $rc) comparing ${S_PATH[$n]} with $p's copy"
          # $(...) dropped diff's final newline (its only trailing one: no
          # diff -u line is empty), hence the +1 here and printf '%s\n' later.
          if [ $((${#d} + 1)) -lt "${S_SIZE[$n]}" ]; then kind[$n]=patch dif[$n]=$d; fi
        fi
      fi
      case ${kind[$n]} in
        full) cost=$((cost + ${S_SIZE[$n]})) ;;
        patch) cost=$((cost + ${#dif[$n]} + 1)) ;;
      esac
      [ "$cost" -lt "$BEST_BYTES" ] || continue 2   # can't win
    done
    [ -z "${BROKEN[$p]-}" ] || continue
    BEST=$p BEST_BYTES=$cost PLAN_KIND=() PLAN_DIFF=()
    for n in "${S_NAMES[@]}"; do
      PLAN_KIND[$n]=${kind[$n]}
      [ "${kind[$n]}" != patch ] || PLAN_DIFF[$n]=${dif[$n]}
    done
  done
}

# write_dir DEST SRC_MANIFEST [full]: write the S_* files into DEST as
# PLAN_KIND says (or all in full), plus a manifest.json that takes file_path
# and repo_path from SRC_MANIFEST. Sets DIR_BYTES.
write_dir() {
  local dest=$1 src=$2 all_full=${3-} n kind info=
  local -a full=() dirs=("$dest")
  local -A kinds=()
  for n in "${S_NAMES[@]}"; do
    kind=${PLAN_KIND[$n]-full}
    [ -z "$all_full" ] || kind=full
    kinds[$n]=$kind
  done
  # Only the subfolders it needs: an empty folder still takes 4 KB of SD card.
  [[ " ${kinds[*]} " != *" full "* ]] || dirs+=("$dest/full")
  [[ " ${kinds[*]} " != *" patch "* ]] || dirs+=("$dest/patch")
  rm -rf "$dest"
  mkdir -p "${dirs[@]}"
  DIR_BYTES=0
  for n in "${S_NAMES[@]}"; do
    kind=${kinds[$n]}
    case $kind in
      full) full+=("${S_FILE[$n]}") DIR_BYTES=$((DIR_BYTES + ${S_SIZE[$n]})) ;;
      patch)
        printf '%s\n' "${PLAN_DIFF[$n]}" >"$dest/patch/$n"
        DIR_BYTES=$((DIR_BYTES + ${#PLAN_DIFF[$n]} + 1)) ;;
    esac
    info+=$n$US${S_SHA[$n]}$US${S_SIZE[$n]}$US$kind$US${S_MODE[$n]}$'\n'
  done
  # The S_FILE paths all end in the file's name (files/NAME or cache/ID/NAME).
  [ ${#full[@]} -eq 0 ] || cp -- "${full[@]}" "$dest/full/"
  printf '%s' "$info" | jq -R -n --slurpfile m "$src" '
    (reduce (inputs | split("\u001f")) as $r ({};
      .[$r[0]] = {sha256: $r[1], size: ($r[2] | tonumber), stored: $r[3], mode: $r[4]})) as $info
    | $m[0] | map(select($info[.file_name]) | {file_name, file_path, repo_path} + $info[.file_name])
  ' >"$dest/manifest.json"
}

# check_dir ID DIR PARENT: rebuild ID from DIR against PARENT; true if every
# file comes out with the right sha256. Leaves M_*[ID] describing DIR.
check_dir() {
  load_manifests "$1" "$2"
  PARENT[$1]=$3
  rm -rf "$CACHE/.check"
  rebuild "$1" "$CACHE/.check"
}

# swap_dir ID INDEX_EDIT_ARGS...: replace vID with vID.tmp, then index_edit to
# match. vID.old goes only after that edit, so recover() can finish a swap
# that was cut off: without vID it puts vID.old back; with vID, the edit may
# be missing, so it counts vID's stored bytes again.
swap_dir() {
  local id=$1 d=$VERSIONS_DIR/$1
  shift
  sync_fs "$VERSIONS_DIR"
  rm -rf "$d.old"
  mv "$d" "$d.old"
  mv "$d.tmp" "$d"
  index_edit "$@"
  rm -rf "$d.old"
  M_DIR[$id]=$d
}

subject_from_version() {  # the S_* files = version ID's rebuilt files
  local id=$1 n i
  local -a sizes
  names_of "$id" S_NAMES
  S_PATH=() S_SHA=() S_SIZE=() S_MODE=() S_FILE=()
  mapfile -t sizes < <(cd "$CACHE/$id" && stat -c %s -- "${S_NAMES[@]}")
  for i in "${!S_NAMES[@]}"; do
    n=${S_NAMES[i]}
    S_PATH[$n]=${M_PATH[$id/$n]} S_SHA[$n]=${M_SHA[$id/$n]} S_MODE[$n]=${M_MODE[$id/$n]}
    S_SIZE[$n]=${sizes[i]} S_FILE[$n]=$CACHE/$id/$n
  done
}

# Sets one version's parent and stored_bytes in index.json.
SET_LAYOUT='(.versions[] | select(.id == $id)) |= (.parent = (if $p == "" then null else $p end) | .stored_bytes = $b)'

# repoint ID: store version ID (rebuilt, and loaded as the S_* files) as
# choose_parent planned. Never invalid in between, so a power cut at any
# point is harmless: ID is first rewritten in full (valid whatever its
# parent), then its parent changes, then it's rewritten as diffs against it.
repoint() {
  local id=$1 dir=$VERSIONS_DIR/$1 n all_full=yes full_bytes=0
  for n in "${S_NAMES[@]}"; do
    [ "${M_STORED[$id/$n]}" = full ] || all_full=
    full_bytes=$((full_bytes + ${S_SIZE[$n]}))
  done
  if [ -z "$all_full" ]; then
    write_dir "$dir.tmp" "$dir/manifest.json" full
    check_dir "$id" "$dir.tmp" "" || die "couldn't rewrite $id in full: ${BROKEN[$id]}"
    swap_dir "$id" --arg id "$id" --arg p "$BEST" --argjson b "$full_bytes" "$SET_LAYOUT"
  else
    index_edit --arg id "$id" --arg p "$BEST" --argjson b "$full_bytes" "$SET_LAYOUT"
  fi
  PARENT[$id]=$BEST SBYTES[$id]=$full_bytes
  [ -n "$BEST" ] || return 0

  write_dir "$dir.tmp" "$dir/manifest.json"
  if ! check_dir "$id" "$dir.tmp" "$BEST"; then
    warn "$id didn't rebuild correctly from diffs against $BEST (${BROKEN[$id]}); keeping it in full"
    unset 'BROKEN[$id]'
    rm -rf "$dir.tmp"
    load_manifests "$id" "$dir"
    BEST= PARENT[$id]=
    index_edit --arg id "$id" --arg p "" --argjson b "$full_bytes" "$SET_LAYOUT"
    return 0
  fi
  swap_dir "$id" --arg id "$id" --arg p "$BEST" --argjson b "$DIR_BYTES" "$SET_LAYOUT"
  SBYTES[$id]=$DIR_BYTES
}

# prune MAX: delete the oldest versions (never the current one) until MAX are
# left. First each surviving version stored against one of them is rebuilt
# and re-pointed at its best remaining base: never itself or one of its
# descendants (so parents can't form a loop) and never a version about to be
# deleted (nor is one of those re-pointed). Then they all leave index.json in
# one edit, since nothing left refers to them, and their folders are deleted.
prune() {
  local extra=$((${#IDS[@]} - $1)) o c id changed i
  local -a doomed=() cands keep=() dirs=()
  local -A gone=() desc=() notes=()
  [ "$extra" -gt 0 ] || return 0
  for id in "${IDS[@]}"; do
    [ "$extra" -gt 0 ] || break
    [ "$id" != "$CURRENT" ] || continue
    doomed+=("$id") gone[$id]=1 extra=$((extra - 1))
  done
  for c in "${IDS[@]}"; do
    o=${PARENT[$c]}
    [ -z "${gone[$c]-}" ] && [ -n "$o" ] && [ -n "${gone[$o]-}" ] || continue
    if ! recon "$c"; then
      warn "$c is damaged (${BROKEN[$c]}), so it can't be re-pointed before $o is deleted"
      notes[$o]+=", $c left broken"
      continue
    fi
    desc=([$c]=1) changed=1
    while [ -n "$changed" ]; do
      changed=
      for id in "${IDS[@]}"; do
        if [ -z "${desc[$id]-}" ] && [ -n "${PARENT[$id]}" ] && [ -n "${desc[${PARENT[$id]}]-}" ]; then
          desc[$id]=1 changed=1
        fi
      done
    done
    cands=()
    for ((i = ${#IDS[@]} - 1; i >= 0; i--)); do
      id=${IDS[i]}
      [ -n "${gone[$id]-}" ] || [ -n "${desc[$id]-}" ] || cands+=("$id")
    done
    subject_from_version "$c"
    choose_parent "${cands[@]}"
    repoint "$c"
    if [ -n "$BEST" ]; then notes[$o]+=", $c now stored as a diff vs $BEST"; else notes[$o]+=", $c now stored in full"; fi
  done
  index_edit --arg d "${doomed[*]}" '($d | split(" ")) as $d | .versions |= map(select(.id as $i | all($d[]; . != $i)))'
  for o in "${doomed[@]}"; do dirs+=("$VERSIONS_DIR/$o"); unset 'PARENT[$o]' 'HASH[$o]'; done
  rm -rf -- "${dirs[@]}"
  for id in "${IDS[@]}"; do [ -n "${gone[$id]-}" ] || keep+=("$id"); done
  IDS=("${keep[@]}")
  for o in "${doomed[@]}"; do log "Pruned $o${notes[$o]:+; ${notes[$o]#, }}"; done
}

# ------------------------------------------------------------------ commands

# Checks the deploy manifest and prints a line per entry: "ok US file_name US
# file_path US repo_path", or "bad US N US why US the entry" for one the store
# can't hold (file_name names a file in the store, so no slashes; like
# deploy.sh, no backslashes or control characters anywhere). If two entries
# share a file_path or file_name, just "dup US key US value".
MANIFEST_CHECK='
  def s: tostring | gsub("[[:cntrl:]]"; " ");
  def odd($k): if test("\\\\") then "\($k) contains a backslash"
    elif test("[[:cntrl:]]") then "\($k) contains a control character" else empty end;
  def why:
    if type != "object" then "it is not an object"
    elif (.file_name | type) != "string" or .file_name == "" then "file_name is missing or not a string"
    elif (.file_path | type) != "string" then "file_path is missing or not a string"
    elif (.repo_path // "" | type) != "string" then "repo_path is not a string"
    else first(
      (.file_name | if test("/") then "file_name contains a slash"
                    elif . == "." or . == ".." then "file_name is \(.)" else odd("file_name") end),
      (.file_path | if startswith("/") then odd("file_path") else "file_path is not an absolute path" end),
      (.repo_path // "" | odd("repo_path")),
      "")
    end;
  if type != "array" then error("not a list") else . end
  | [.[] | objects] as $o
  | ([$o[] | .file_path | strings] | group_by(.) | map(select(length > 1) | .[0])) as $dp
  | ([$o[] | .file_name | strings] | group_by(.) | map(select(length > 1) | .[0])) as $dn
  | if $dp != [] then ["dup", "file_path", ($dp[0] | tojson)]
    elif $dn != [] then ["dup", "file_name", ($dn[0] | tojson)]
    else range(length) as $i | .[$i] | why as $w
      | if $w == "" then ["ok", .file_name, .file_path, .repo_path // ""]
        else ["bad", $i + 1, $w, if type == "object" then "file_name \(.file_name | tojson), file_path \(.file_path | tojson)" else tojson end]
        end
    end
  | map(s) | join("\u001f")'

cmd_snapshot() {
  local source= detail= out kind a b c hash hit cur count id num i n max extra how need=0 entries=0 manifest=$MANIFEST
  local -a same_name=() sums stats cands=()
  local -A doomed=()
  while [ $# -gt 0 ]; do
    case $1 in
      --source) [ $# -ge 2 ] || die "--source needs a value"; source=$2; shift 2 ;;
      --detail) [ $# -ge 2 ] || die "--detail needs a value"; detail=$2; shift 2 ;;
      -h | --help) usage ;;
      *) die "snapshot: unexpected argument '$1' (see versions.sh -h)" ;;
    esac
  done
  [[ -n $source && " $SOURCES " == *" $source "* ]] || die "snapshot needs --source, one of: $SOURCES"
  [ -f "$MANIFEST" ] || die "there's no $MANIFEST yet (nothing has been deployed), so nothing to snapshot"
  lock_store aside

  # 1. Copy the installed files first (with their modes), so they can't change
  # between hashing and storing. An entry the store can't hold is left out,
  # saying why, rather than recording nothing at all.
  out=$(jq -r "$MANIFEST_CHECK" "$MANIFEST" 2>/dev/null) ||
    die "$MANIFEST isn't valid JSON, or not a list of {file_name, file_path, repo_path}"
  mkdir -p "$WORK/new/files"
  S_NAMES=()
  while IFS=$US read -r kind a b c; do
    case $kind in
      ok) entries=$((entries + 1)) ;;
      bad)
        warn "Entry $a of $MANIFEST ($c) can't be stored: $b; leaving it out of this version"
        entries=$((entries + 1))
        continue ;;
      dup) die "$MANIFEST has more than one entry with $a $b, so it's unclear which to store; deploy again to fix it" ;;
      *) continue ;;
    esac
    if [ -f "$b" ]; then
      # Usually the file is already called file_name: copy all those with one cp.
      if [ "${b##*/}" = "$a" ]; then same_name+=("$b"); else cp --preserve=mode -- "$b" "$WORK/new/files/$a"; fi
      S_NAMES+=("$a") S_PATH[$a]=$b S_FILE[$a]=$WORK/new/files/$a
    else
      warn "$b (in the deploy manifest) doesn't exist; leaving it out of this version"
    fi
  done <<<"$out"
  [ "$entries" -gt 0 ] || die "$MANIFEST lists no files; nothing to snapshot"
  [ ${#same_name[@]} -eq 0 ] || cp --preserve=mode -- "${same_name[@]}" "$WORK/new/files/"
  [ ${#S_NAMES[@]} -gt 0 ] || die "none of the files in $MANIFEST can be stored (see above); nothing to snapshot"
  if [ ${#S_NAMES[@]} -lt "$entries" ]; then
    manifest=$WORK/new/manifest.json
    jq '[.[] | objects | select(.file_name as $n | any($ARGS.positional[]; . == $n))]' \
      "$MANIFEST" --args "${S_NAMES[@]}" >"$manifest"
  fi
  hash=$(version_hash "$manifest" "$WORK/new/files")
  [[ $hash =~ ^[0-9a-f]{64}$ ]] || die "couldn't compute the version hash"

  # 2. Already stored? Then it's just the current version again. Also prune if
  # there are too many: normally there aren't, but a run cut off while
  # pruning (or a lower max_versions) leaves some.
  [ -s "$INDEX" ] || new_index
  out=$(jq -r --arg h "$hash" '(first(.versions[] | select(.hash == $h) | .id) // ""), (.current // ""),
      (.versions | length)' "$INDEX") || die "$INDEX is damaged"
  { read -r hit; read -r cur; read -r count; } <<<"$out"
  if [ -n "$hit" ]; then
    [ "$hit" = "$cur" ] || index_edit --arg id "$hit" '.current = $id'
    if [ "$count" -gt 1 ]; then
      max=$(max_versions)
      if [ "$count" -gt "$max" ]; then
        load_index
        load_all
        prune "$max"
      fi
    fi
    echo "$hit (already stored)"
    return 0
  fi

  # 3. A new version. Store it against the existing version that gives the
  # smallest diffs, leaving out those the prune below will delete.
  load_index
  mapfile -t sums < <(cd "$WORK/new/files" && sha256sum -- "${S_NAMES[@]}")
  mapfile -t stats < <(cd "$WORK/new/files" && stat -c '%s %a' -- "${S_NAMES[@]}")
  for i in "${!S_NAMES[@]}"; do
    n=${S_NAMES[i]}
    S_SHA[$n]=${sums[i]:0:64} S_SIZE[$n]=${stats[i]% *} S_MODE[$n]=${stats[i]#* }
    need=$((need + ${S_SIZE[$n]}))
  done
  load_all "$need"
  num=$((10#$NEXT_ID))
  for id in "${IDS[@]}"; do   # in case next_id is behind (it shouldn't be)
    [[ $id =~ ^v([0-9]+)$ ]] || continue
    [ $((10#${BASH_REMATCH[1]})) -lt "$num" ] || num=$((10#${BASH_REMATCH[1]} + 1))
  done
  id=v$num
  max=$(max_versions)
  extra=$((${#IDS[@]} + 1 - max))
  for i in "${IDS[@]}"; do
    [ "$extra" -gt 0 ] || break
    doomed[$i]=1 extra=$((extra - 1))
  done
  for ((i = ${#IDS[@]} - 1; i >= 0; i--)); do
    [ -n "${doomed[${IDS[i]}]-}" ] || cands+=("${IDS[i]}")
  done
  choose_parent "${cands[@]}"

  # 4. Write it, check that it rebuilds, then make it real.
  write_dir "$VERSIONS_DIR/$id.tmp" "$manifest"
  if ! check_dir "$id" "$VERSIONS_DIR/$id.tmp" "$BEST"; then
    [ -n "$BEST" ] || die "couldn't store $id: ${BROKEN[$id]}"
    warn "$id didn't rebuild correctly from diffs against $BEST (${BROKEN[$id]}); storing it in full"
    unset 'BROKEN[$id]'
    BEST=
    write_dir "$VERSIONS_DIR/$id.tmp" "$manifest" full
    check_dir "$id" "$VERSIONS_DIR/$id.tmp" "" || die "couldn't store $id: ${BROKEN[$id]}"
  fi
  sync_fs "$VERSIONS_DIR"
  mv "$VERSIONS_DIR/$id.tmp" "$VERSIONS_DIR/$id"
  M_DIR[$id]=$VERSIONS_DIR/$id
  index_edit --arg id "$id" --arg hash "$hash" --arg parent "$BEST" --arg created "$(date -Iseconds)" \
    --arg source "$source" --arg detail "$detail" --argjson files "${#S_NAMES[@]}" \
    --argjson bytes "$DIR_BYTES" --argjson next "$((num + 1))" '
    .versions += [{id: $id, hash: $hash, parent: (if $parent == "" then null else $parent end),
                   created: $created, source: $source, detail: $detail,
                   files: $files, stored_bytes: $bytes}]
    | .next_id = $next | .current = $id'
  IDS+=("$id") CURRENT=$id
  PARENT[$id]=$BEST HASH[$id]=$hash SBYTES[$id]=$DIR_BYTES
  # The check above rebuilt it into $CACHE/.check: keep that as its cached copy.
  mv "$CACHE/.check" "$CACHE/$id"
  DONE[$id]=1
  describe how "$DIR_BYTES" "$BEST"

  # 5. Keep at most max_versions.
  prune "$max"
  echo "$id (new, $how)"
}

cmd_list() {
  local n=0 i id mark created stored shown=0
  while [ $# -gt 0 ]; do
    case $1 in
      -n) [[ ${2-} =~ ^[1-9][0-9]*$ ]] || die "-n needs a number (at least 1)"; n=$2; shift 2 ;;
      -h | --help) usage ;;
      *) die "list: unexpected argument '$1' (see versions.sh -h)" ;;
    esac
  done
  load_index
  if [ ${#IDS[@]} -eq 0 ]; then echo "No versions stored yet."; return 0; fi
  printf '   %-8s %-17s %-9s %-19s %s\n' VERSION CREATED SOURCE STORED DETAIL
  for ((i = ${#IDS[@]} - 1; i >= 0; i--)); do
    [ "$n" -eq 0 ] || [ "$shown" -lt "$n" ] || break
    id=${IDS[i]}
    mark=' '
    [ "$id" != "$CURRENT" ] || mark='*'
    created=${CREATED[$id]:0:10}\ ${CREATED[$id]:11:5}
    describe stored "${SBYTES[$id]:-0}" "${PARENT[$id]}"
    printf ' %s %-8s %-17s %-9s %-19s %s\n' "$mark" "$id" "$created" "${SOURCE[$id]}" "$stored" "${DETAIL[$id]}"
    shown=$((shown + 1))
  done
  [ "$shown" -eq ${#IDS[@]} ] || echo "($((${#IDS[@]} - shown)) older not shown)"
}

# resolve VERSION: print the id that v12 / 12 / a hash prefix names.
resolve() {
  local arg=$1 id
  local -a matches=()
  if [[ $arg =~ ^[vV]?([0-9]+)$ ]]; then
    id=v$((10#${BASH_REMATCH[1]}))
    if [ -n "${HASH[$id]+set}" ]; then echo "$id"; return 0; fi
    if [[ $arg == [vV]* || ${#arg} -lt 4 ]]; then die "there's no version $id (versions.sh list shows them)"; fi
  fi
  if [[ $arg =~ ^[0-9a-fA-F]{4,64}$ ]]; then
    arg=${arg,,}
    for id in "${IDS[@]}"; do
      if [[ ${HASH[$id]} == "$arg"* ]]; then matches+=("$id"); fi
    done
    case ${#matches[@]} in
      1) echo "${matches[0]}"; return 0 ;;
      0) die "no version has a number or hash starting with $arg (versions.sh list shows them)" ;;
      *) die "$arg is the start of more than one version's hash (${matches[*]}); give more of it" ;;
    esac
  fi
  die "'$1' isn't a version: use v12, 12, or the start of a version hash (at least 4 hex digits)"
}

cmd_show() {
  [ "${1-}" != -h ] && [ "${1-}" != --help ] || usage
  [ $# -eq 1 ] || die "usage: versions.sh show VERSION"
  local id p n stored size kids=() c
  local -a names patches=() psizes=()
  local -A psize=()
  lock_store
  load_index
  id=$(resolve "$1")
  load_manifests "$id" "$VERSIONS_DIR/$id"
  [ -z "${BROKEN[$id]-}" ] || die "$id is damaged: ${BROKEN[$id]} (run versions.sh verify)"
  p=${PARENT[$id]}
  for c in "${IDS[@]}"; do [ "${PARENT[$c]}" != "$id" ] || kids+=("$c"); done
  names_of "$id" names
  for n in "${names[@]}"; do [ "${M_STORED[$id/$n]}" != patch ] || patches+=("$n"); done
  if [ ${#patches[@]} -gt 0 ]; then
    mapfile -t psizes < <(cd "$VERSIONS_DIR/$id/patch" && stat -c %s -- "${patches[@]}" 2>/dev/null)
    for c in "${!patches[@]}"; do psize[${patches[c]}]=${psizes[c]:-0}; done
  fi

  echo "$id$([ "$id" != "$CURRENT" ] || echo "  (current)")"
  echo "  hash      ${HASH[$id]}"
  echo "  created   ${CREATED[$id]}"
  echo "  source    ${SOURCE[$id]}"
  echo "  detail    ${DETAIL[$id]}"
  describe stored "${SBYTES[$id]:-0}" "$p"
  echo "  stored    $stored"
  [ ${#kids[@]} -eq 0 ] || echo "  base of   ${kids[*]} (stored as diffs against $id)"
  echo
  printf '  %-8s  %-6s  %-4s  %-20s  %s\n' SHA256 SIZE MODE STORED PATH
  for n in "${names[@]}"; do
    case ${M_STORED[$id/$n]} in
      patch) human size "${psize[$n]:-0}"; stored="patch vs $p, $size" ;;
      same) stored="same as $p" ;;
      *) stored=${M_STORED[$id/$n]} ;;
    esac
    human size "${M_SIZE[$id/$n]}"
    printf '  %-8s  %-6s  %-4s  %-20s  %s\n' "${M_SHA[$id/$n]:0:8}" "$size" "${M_MODE[$id/$n]}" "$stored" \
      "${M_PATH[$id/$n]}${M_REPO[$id/$n]:+  (repo: ${M_REPO[$id/$n]})}"
  done
}

cmd_restore() {
  [ "${1-}" != -h ] && [ "${1-}" != --help ] || usage
  [ $# -eq 1 ] || die "usage: versions.sh restore VERSION"
  local id n m replaced=
  local -a names files=()
  local -A by_mode=()
  # Replacing pending/ holds the lock startup.sh takes to claim it (see
  # lib/common.sh), taken before the store's lock like everywhere else.
  # deploy/versions.sh rollback takes it itself, to reboot while holding it.
  [ -n "${OEM_PENDING_LOCKED:-}" ] || lock_pending ||
    die "gave up waiting for $PENDING_LOCK after 30 s (the Pi is applying a deploy); try again"
  lock_store
  load_index
  id=$(resolve "$1")
  load_all
  recon "$id" || die "can't rebuild $id: ${BROKEN[$id]} (run versions.sh verify)"
  names_of "$id" names
  for n in "${names[@]}"; do files+=("$CACHE/$id/$n"); done

  # Built next to pending/ and renamed into place, so startup.sh never sees half
  # of it. Any other .restore-* is from a restore that a power cut interrupted
  # (restores can't overlap: they hold the store's lock).
  mkdir -p "$DEPLOY_DIR"
  rm -rf "$DEPLOY_DIR"/.restore-*
  RESTORE_TMP=$(mktemp -d "$DEPLOY_DIR/.restore-XXXXXX")
  mkdir "$RESTORE_TMP/files"
  cp -- "${files[@]}" "$RESTORE_TMP/files/"
  # Each file gets back the mode it had when it was snapshotted (startup.sh
  # installs it with that mode): one chmod per mode.
  for n in "${names[@]}"; do by_mode[${M_MODE[$id/$n]}]+=$RESTORE_TMP/files/$n$'\n'; done
  for m in "${!by_mode[@]}"; do
    mapfile -t files <<<"${by_mode[$m]%$'\n'}"
    chmod -- "$m" "${files[@]}"
  done
  jq 'map({file_name, file_path, repo_path})' "$VERSIONS_DIR/$id/manifest.json" >"$RESTORE_TMP/deploy-manifest.json"
  jq -n --arg v "$id" --arg t "$(date -Iseconds)" \
    '{source: "rollback", detail: "rollback to \($v)", version: $v, time: $t}' >"$RESTORE_TMP/deploy-info.json"
  sync_fs "$RESTORE_TMP"
  if [ -d "$PENDING_DIR" ]; then
    replaced=$(jq -r '.detail // empty' "$PENDING_DIR/deploy-info.json" 2>/dev/null) || true
    replaced=${replaced:-a deploy}
  fi
  rm -rf "$PENDING_DIR"
  mv "$RESTORE_TMP" "$PENDING_DIR"
  RESTORE_TMP=

  echo "Staged $id (${CREATED[$id]:0:10} ${CREATED[$id]:11:5}, ${SOURCE[$id]}: ${DETAIL[$id]}) in $(tilde "$PENDING_DIR"):"
  for n in "${names[@]}"; do echo "  ${M_PATH[$id/$n]}"; done
  [ -z "$replaced" ] || echo "It replaces a pending deploy that hadn't been applied yet ($replaced)."
  echo "startup.sh installs it on the next boot."
}

cmd_verify() {
  [ "${1-}" != -h ] && [ "${1-}" != --help ] || usage
  [ $# -eq 0 ] || die "usage: versions.sh verify"
  local id p steps prev=0 problems=0 h n how
  local -a errs
  local -A seen_hash=()
  lock_store
  load_index
  if [ ${#IDS[@]} -eq 0 ] && [ ${#DUP_IDS[@]} -eq 0 ]; then echo "No versions stored yet."; return 0; fi
  load_all
  for id in "${DUP_IDS[@]}"; do echo "  $id  BROKEN: listed more than once in index.json"; problems=$((problems + 1)); done
  if [ -n "$CURRENT" ] && [ -z "${HASH[$CURRENT]+set}" ]; then
    echo "  index.json: the current version $CURRENT doesn't exist"; problems=$((problems + 1))
  fi
  for id in "${IDS[@]}"; do
    errs=()
    if [[ ! $id =~ ^v([0-9]+)$ ]]; then
      errs+=("its id isn't vN")
    else
      n=$((10#${BASH_REMATCH[1]}))
      [ "$n" -gt "$prev" ] || errs+=("index.json isn't in id order here")
      [ "$n" -lt "$NEXT_ID" ] || errs+=("next_id ($NEXT_ID) isn't above its id")
      prev=$n
    fi
    [[ ${HASH[$id]} =~ ^[0-9a-f]{64}$ ]] || errs+=("its hash isn't 64 hex digits")
    if [ -n "${HASH[$id]}" ]; then
      if [ -n "${seen_hash[${HASH[$id]}]-}" ]; then errs+=("same hash as ${seen_hash[${HASH[$id]}]}"); fi
      seen_hash[${HASH[$id]}]=$id
    fi
    p=${PARENT[$id]} steps=0
    while [ -n "$p" ] && [ "$steps" -le ${#IDS[@]} ]; do
      if [ -z "${HASH[$p]+set}" ]; then errs+=("its parent chain reaches $p, which doesn't exist"); break; fi
      p=${PARENT[$p]} steps=$((steps + 1))
    done
    [ "$steps" -le ${#IDS[@]} ] || errs+=("its parents form a loop")
    if [ ${#errs[@]} -eq 0 ]; then
      if ! recon "$id"; then
        errs+=("${BROKEN[$id]}")
      else
        h=$(version_hash "$VERSIONS_DIR/$id/manifest.json" "$CACHE/$id")
        [ "$h" = "${HASH[$id]}" ] || errs+=("its files don't add up to its version hash")
      fi
    fi
    if [ ${#errs[@]} -eq 0 ]; then
      describe how "${SBYTES[$id]:-0}" "${PARENT[$id]}"
      printf '  %-8s ok      %s\n' "$id" "$how"
    else
      printf '  %-8s BROKEN: %s\n' "$id" "$(join "; " "${errs[@]}")"
      problems=$((problems + 1))
    fi
  done
  if [ "$problems" -gt 0 ]; then
    echo "$problems problem(s) found in $(tilde "$VERSIONS_DIR")."
    exit 1
  fi
  echo "All ${#IDS[@]} versions OK${CURRENT:+ (current: $CURRENT)}."
}

cmd_current() {
  [ "${1-}" != -h ] && [ "${1-}" != --help ] || usage
  if [ ! -s "$INDEX" ]; then
    if have_folders; then damaged_index; fi
    return 0
  fi
  jq -r "$INDEX_CHECK"' | .current // empty' "$INDEX" 2>/dev/null || damaged_index
}

case ${1-} in
  snapshot | list | show | restore | verify | current)
    cmd=$1
    shift
    "cmd_$cmd" "$@"
    ;;
  -h | --help) usage ;;
  "") usage 2 ;;
  *) die "unknown command '$1' (see versions.sh -h)" ;;
esac
