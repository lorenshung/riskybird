# Source this to get a shell that can run `rb`.
#
#   source ./rb-env.sh
#
# Everything docs/setup.md tells you to export, in one place. The guide is
# written for an interactive shell that keeps state between commands; anything
# driving `rb` from a script, a Makefile, CI or an agent gets a fresh
# environment every time and has to re-apply all of it. That is what this is for.
#
# Safe to source repeatedly. Sets nothing that is already set.

_rb_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# 1. rb itself.
case ":${PATH}:" in
  *":${_rb_root}:"*) ;;
  *) PATH="${_rb_root}:${PATH}"; export PATH ;;
esac

# 2. Collected devicetrees and bitstreams, for a host that cannot build them.
#    Only set if the caller has not, and only if the directory exists -- rb
#    already falls back to <root>/dts and <root>/bitstreams on its own.
for _rb_pair in "RB_DTS_DIR:${HOME}/riskybird-dts" \
                "RB_BITSTREAM_DIR:${HOME}/riskybird-bitstreams"; do
  _rb_var="${_rb_pair%%:*}"
  _rb_dir="${_rb_pair#*:}"
  if [ -z "$(eval "printf '%s' \"\${${_rb_var}:-}\"")" ] && [ -d "${_rb_dir}" ]; then
    export "${_rb_var}=${_rb_dir}"
  fi
done
unset _rb_pair _rb_var _rb_dir

# 3. A vendor cmake earlier on PATH breaks the Zephyr SDK install. See
#    docs/setup.md step 2 "Host prerequisites" for why this is not paranoia.
if command -v cmake >/dev/null 2>&1; then
  case "$(command -v cmake)" in
    /ecad/*|*/Vitis/*|*/Vivado/*)
      echo "rb-env: cmake resolves to $(command -v cmake)" >&2
      echo "rb-env: that one is likely broken; putting /usr/bin first" >&2
      PATH="/usr/bin:${PATH}"; export PATH ;;
  esac
fi

# 4. An active conda environment makes the Zephyr installer build into it
#    rather than into the checkout. Warn rather than act -- deactivating
#    someone's shell for them is worse than telling them.
if [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
  echo "rb-env: conda environment '${CONDA_DEFAULT_ENV}' is active." >&2
  echo "rb-env: run 'conda deactivate' before setup.md step 2b." >&2
fi

echo "rb-env: rb on PATH${RB_DTS_DIR:+, RB_DTS_DIR=${RB_DTS_DIR}}${RB_BITSTREAM_DIR:+, RB_BITSTREAM_DIR=${RB_BITSTREAM_DIR}}"
unset _rb_root
