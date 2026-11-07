#!/usr/bin/env bash
# send_logs.sh - copy LIMO test logs (evaluation/*) from the lab machine back
# to the developer machine over scp (one password prompt, shared connection).
#
#   Lab machine :  tools/send_logs.sh            (interactive)
#   Dev machine :  tools/send_logs.sh --check-receiver
#
# Needs only coreutils + openssh-client. Never uses sudo interactively.

# ---- Defaults (edit here) ---------------------------------------------------
REMOTE_USER="alienator"
REMOTE_DEST="/media/alienator/Storage/Jay/Robotics/ALNC/HW_Ackermann_Drive/evaluation/from_lab/"
# -----------------------------------------------------------------------------

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="$ROOT/evaluation"
REMOTE_HOST=""
DRY_RUN=0
CHECK_RX=0
TMPD=""

usage() {
cat <<USAGE
Usage: $(basename "$0") [options]

Copy test log folders from <repo>/evaluation to another machine with scp.

Options:
  -d DIR           log folder (default: $LOGDIR)
  -i IP|HOST       destination IP / hostname (asked if omitted)
  -u USER          destination user (default: $REMOTE_USER)
  -p PATH          destination directory (default: $REMOTE_DEST)
  --dry-run        print the ssh/scp commands instead of running them
  --check-receiver run on the RECEIVING machine: check it can accept scp
  -h, --help       this help

Selection prompt: Enter = all not yet sent, a = all, or numbers like "1 3 4".
Folders sent successfully are recorded in <log folder>/.sent_log.
USAGE
}

die() { echo "ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    -d) [ $# -ge 2 ] || die "-d needs an argument"; LOGDIR="$2"; shift 2 ;;
    -i) [ $# -ge 2 ] || die "-i needs an argument"; REMOTE_HOST="$2"; shift 2 ;;
    -u) [ $# -ge 2 ] || die "-u needs an argument"; REMOTE_USER="$2"; shift 2 ;;
    -p) [ $# -ge 2 ] || die "-p needs an argument"; REMOTE_DEST="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --check-receiver) CHECK_RX=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
LOGDIR="${LOGDIR%/}"

# ============================================================================
# Receiver check
# ============================================================================
check_receiver() {
  local blocked=0 svc="" ok="[ OK ]" warn="[WARN]" bad="[FAIL]"
  echo "== Receiver check on $(hostname) (user: $(id -un)) =="

  # 1. openssh-server installed
  if dpkg -s openssh-server >/dev/null 2>&1; then
    echo "$ok openssh-server is installed"
  else
    echo "$bad openssh-server is NOT installed. Run:  sudo apt install openssh-server"
    blocked=1
  fi

  # 2. sshd active
  if command -v systemctl >/dev/null 2>&1; then
    local s
    for s in ssh sshd; do
      if [ "$(systemctl is-active "$s" 2>/dev/null)" = "active" ]; then svc="$s"; break; fi
    done
    if [ -n "$svc" ]; then
      echo "$ok sshd is active (systemctl: $svc)"
    else
      echo "$bad sshd is not active. Run:  sudo systemctl enable --now ssh"
      blocked=1
    fi
  else
    echo "$warn systemctl not found, cannot check the sshd service"
  fi

  # 3. port 22 listening
  if command -v ss >/dev/null 2>&1; then
    if ss -tln 2>/dev/null | awk 'NR>1{print $4}' | grep -Eq '[:.]22$'; then
      echo "$ok port 22 is listening"
    else
      echo "$bad nothing is listening on TCP port 22 (start sshd, see above)"
      blocked=1
    fi
  else
    echo "$warn 'ss' not found, cannot check port 22"
  fi

  # 4. ufw
  if [ -r /etc/ufw/ufw.conf ]; then
    local en
    en="$(grep -E '^ENABLED=' /etc/ufw/ufw.conf | tail -n1 | cut -d= -f2 | tr -d ' "')"
    if [ "$en" = "yes" ]; then
      echo "$warn ufw firewall is ENABLED. Make sure SSH is allowed:  sudo ufw allow ssh"
    else
      echo "$ok ufw firewall is not enabled (ENABLED=${en:-unset})"
    fi
  elif [ -e /etc/ufw/ufw.conf ]; then
    echo "$warn /etc/ufw/ufw.conf not readable without sudo. Check with:  sudo ufw status"
  else
    echo "$ok ufw not installed (no /etc/ufw/ufw.conf)"
  fi

  # 5. password authentication
  local files=() f unreadable=0 pa_no=""
  [ -e /etc/ssh/sshd_config ] && files+=(/etc/ssh/sshd_config)
  for f in /etc/ssh/sshd_config.d/*.conf; do [ -e "$f" ] && files+=("$f"); done
  if [ "${#files[@]}" -eq 0 ]; then
    echo "$warn no sshd_config found, cannot check PasswordAuthentication"
  else
    for f in "${files[@]}"; do
      if [ -r "$f" ]; then
        if grep -Ei '^[[:space:]]*PasswordAuthentication[[:space:]]+no' "$f" >/dev/null 2>&1; then
          pa_no="$f"
        fi
      else
        unreadable=1
      fi
    done
    if [ -n "$pa_no" ]; then
      echo "$bad PasswordAuthentication is set to 'no' in $pa_no"
      echo "       Set it to 'yes' (or use ssh keys), then:  sudo systemctl restart ssh"
      blocked=1
    elif [ "$unreadable" -eq 1 ]; then
      echo "$warn some sshd config files are not readable; verify with:  sudo sshd -T | grep -i passwordauthentication"
    else
      echo "$ok password authentication is not disabled (default is yes)"
    fi
  fi

  # 6. destination directory
  local d="${REMOTE_DEST%/}" anc
  anc="$d"
  while [ -n "$anc" ] && [ ! -e "$anc" ]; do
    anc="$(dirname "$anc")"
    [ "$anc" = "/" ] && break
  done
  if [ -d "$d" ] && [ -w "$d" ]; then
    echo "$ok destination exists and is writable: $d"
  elif [ -d "$d" ]; then
    echo "$bad destination exists but is not writable by $(id -un): $d"
    blocked=1
  elif [ -d "$anc" ] && [ -w "$anc" ]; then
    echo "$ok destination does not exist yet but can be created (under $anc): $d"
  else
    echo "$bad destination cannot be created by $(id -un) (nearest existing: $anc): $d"
    blocked=1
  fi

  # 7. IP addresses
  local ips
  ips="$(hostname -I 2>/dev/null)"
  echo
  echo "IP address(es) of this machine (type one on the lab machine): ${ips:-unknown}"
  echo "Lab machine command:  tools/send_logs.sh -i <one of the IPs above>"
  echo
  if [ "$blocked" -eq 0 ]; then
    echo "Result: this machine looks ready to receive."
    return 0
  fi
  echo "Result: BLOCKED, fix the [FAIL] items above (sudo commands are shown, not run)."
  return 1
}

if [ "$CHECK_RX" -eq 1 ]; then
  check_receiver
  exit $?
fi

# ============================================================================
# Sender
# ============================================================================
cleanup() {
  if [ -n "$TMPD" ] && [ -d "$TMPD" ]; then
    if [ -n "$REMOTE_HOST" ] && [ "$DRY_RUN" -eq 0 ]; then
      ssh -o ControlPath="$TMPD/cm-%r@%h:%p" -O exit "$REMOTE_USER@$REMOTE_HOST" >/dev/null 2>&1
    fi
    rm -rf "$TMPD"
  fi
}
trap cleanup EXIT
trap 'echo; echo "Interrupted."; exit 130' INT TERM

# --- dependencies ------------------------------------------------------------
if ! command -v ssh >/dev/null 2>&1 || ! command -v scp >/dev/null 2>&1; then
  echo "ssh/scp not found (openssh-client is missing)."
  if command -v sudo >/dev/null 2>&1 && sudo -n true >/dev/null 2>&1; then
    echo "Installing openssh-client (passwordless sudo available)..."
    sudo -n apt-get install -y openssh-client || die "apt install failed. Run manually:  sudo apt install openssh-client"
    if ! command -v ssh >/dev/null 2>&1 || ! command -v scp >/dev/null 2>&1; then
      die "ssh/scp still missing after install"
    fi
  else
    echo "Run this command, then start the script again:"
    echo "  sudo apt install openssh-client"
    exit 1
  fi
fi

# --- list folders ------------------------------------------------------------
[ -d "$LOGDIR" ] || die "log folder not found: $LOGDIR"
SENT_LOG="$LOGDIR/.sent_log"

NAMES=()
while IFS= read -r n; do
  [ "$n" = "from_lab" ] && continue    # what this machine received, not its own tests
  [ -d "$LOGDIR/$n" ] && NAMES+=("$n")
done < <(ls -1t "$LOGDIR" 2>/dev/null)
[ "${#NAMES[@]}" -gt 0 ] || die "no test folders in $LOGDIR"

is_sent() { [ -f "$SENT_LOG" ] && grep -qxF -- "$1" "$SENT_LOG"; }

echo "Test folders in $LOGDIR (newest first):"
i=0
for n in "${NAMES[@]}"; do
  i=$((i+1))
  sz="$(du -sh -- "$LOGDIR/$n" 2>/dev/null | cut -f1)"
  mark=""; is_sent "$n" && mark="[sent]"
  printf '  %2d) %-48s %6s  %s\n' "$i" "$n" "$sz" "$mark"
done
echo
printf 'Select: Enter = all not yet sent, a = all, or numbers like "1 3 4": ' >&2
SEL=""; read -r SEL || SEL=""

CHOSEN=()
case "$SEL" in
  "")
    for n in "${NAMES[@]}"; do is_sent "$n" || CHOSEN+=("$n"); done
    [ "${#CHOSEN[@]}" -gt 0 ] || { echo "Everything has already been sent. Use 'a' to resend all."; exit 0; } ;;
  a|A|all) CHOSEN=("${NAMES[@]}") ;;
  *)
    for t in $SEL; do
      case "$t" in *[!0-9]*|"") die "invalid selection: '$t'" ;; esac
      if [ "$t" -lt 1 ] || [ "$t" -gt "${#NAMES[@]}" ]; then die "number out of range: $t"; fi
      CHOSEN+=("${NAMES[$((t-1))]}")
    done ;;
esac
echo "Selected ${#CHOSEN[@]} folder(s): ${CHOSEN[*]}"

# --- destination host --------------------------------------------------------
valid_host() {
  local h="$1"
  if [[ "$h" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    local IFS=. o
    for o in $h; do [ "$o" -le 255 ] || return 1; done
    return 0
  fi
  [[ "$h" =~ ^[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?$ ]] && [[ ! "$h" =~ ^[0-9.]+$ ]]
}
if [ -z "$REMOTE_HOST" ]; then
  printf 'Destination IP or hostname: ' >&2
  read -r REMOTE_HOST || REMOTE_HOST=""
fi
if ! valid_host "$REMOTE_HOST"; then
  bad_host="$REMOTE_HOST"; REMOTE_HOST=""
  die "'$bad_host' is not a valid IPv4 address or hostname"
fi
case "$REMOTE_USER" in ""|*[!A-Za-z0-9._-]*) die "invalid user name: $REMOTE_USER" ;; esac

DEST="${REMOTE_DEST%/}/"
TARGET="$REMOTE_USER@$REMOTE_HOST"
TMPD="$(mktemp -d /tmp/sendlogs.XXXXXX)" || die "cannot create temp dir"
CP="$TMPD/cm-%r@%h:%p"
SSH_OPTS=(-o ConnectTimeout=10 -o ControlMaster=auto -o "ControlPath=$CP" -o ControlPersist=120)
SCP_OPTS=(-r -o ConnectTimeout=10 -o ControlMaster=no -o "ControlPath=$CP")

# run a command, or just print it in dry-run mode
run() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf 'DRY-RUN:'; printf ' %q' "$@"; printf '\n'
    return 0
  fi
  "$@"
}

echo
echo "Destination: $TARGET:$DEST"
[ "$DRY_RUN" -eq 1 ] && echo "(dry run: nothing will be executed)"
echo "You will be asked for the password of $TARGET once."

# first ssh: creates the shared master connection (password asked here)
run ssh "${SSH_OPTS[@]}" "$TARGET" "mkdir -p $DEST" \
  || die "cannot connect or create $DEST on $TARGET. On the receiver run: tools/send_logs.sh --check-receiver"

OK=(); FAIL=(); UNVERIFIED=()
for n in "${CHOSEN[@]}"; do
  echo "--> sending $n"
  if run scp "${SCP_OPTS[@]}" "$LOGDIR/$n" "$TARGET:$DEST"; then
    if [ "$DRY_RUN" -eq 1 ]; then
      run ssh "${SSH_OPTS[@]}" "$TARGET" "du -sh $DEST$n"
      OK+=("$n")
      continue
    fi
    rsz="$(ssh "${SSH_OPTS[@]}" "$TARGET" "du -sh $DEST$n" 2>/dev/null | cut -f1)"
    if [ -n "$rsz" ]; then
      echo "    verified on receiver: $rsz"
      OK+=("$n")
      is_sent "$n" || echo "$n" >> "$SENT_LOG"
    else
      echo "    WARNING: copy finished but could not verify on receiver"
      UNVERIFIED+=("$n")
    fi
  else
    FAIL+=("$n")
  fi
done

# close the shared connection
run ssh -o "ControlPath=$CP" -O exit "$TARGET" >/dev/null 2>&1 || true

echo
echo "================ Summary ================"
echo "Destination : $TARGET:$DEST"
echo "Sent OK     : ${#OK[@]}"
for n in "${OK[@]}"; do echo "    + $n"; done
if [ "${#UNVERIFIED[@]}" -gt 0 ]; then
  echo "Unverified  : ${#UNVERIFIED[@]}"
  for n in "${UNVERIFIED[@]}"; do echo "    ? $n"; done
fi
echo "Failed      : ${#FAIL[@]}"
for n in "${FAIL[@]}"; do echo "    - $n"; done
[ "$DRY_RUN" -eq 1 ] && echo "(dry run: .sent_log not updated)"
if [ "${#FAIL[@]}" -ne 0 ] || [ "${#UNVERIFIED[@]}" -ne 0 ]; then exit 1; fi
exit 0
