#!/bin/bash
# no_ai_slop_check.sh: PostToolUse(Write|Edit) hook. Flags AI-slop wording in
# prose files and in the comment lines of source files.
#
# Wired up by .claude/settings.json. Reads the hook payload on stdin, scans the
# file that was just written, and on a hit prints the offending lines and exits
# 2, which feeds them back to Claude as an error to fix before it continues.
#
# Rules come from the no-ai-slop skill (~/.claude/skills/no-ai-slop/SKILL.md),
# whose wording section is also inlined in CLAUDE.md. Only the flat wordlist is
# enforced here; the structural patterns (colon reveals, binary contrasts,
# kicker endings) are not greppable and are left to CLAUDE.md.

set -u

payload=$(cat)
file=$(jq -r '.tool_response.filePath // .tool_input.file_path // empty' <<<"${payload}" 2>/dev/null)

[ -n "${file}" ] && [ -f "${file}" ] || exit 0

case "${file}" in
    # CLAUDE.md and the skill files quote the banned words as examples, and
    # data/ is off limits. Scanning either is a guaranteed false positive.
    */CLAUDE.md | */.claude/* | */data/* | */venv/* | */build*/*) exit 0 ;;
esac

case "${file}" in
    # Every branch yields "<line>:<text>" so reported hits carry file line numbers.
    *.md | *.txt | *.rst) subject=$(grep -n '' "${file}") ;;
    *.cpp | *.hpp | *.h | *.cc | *.cu | *.cuh) subject=$(grep -nE '^[[:space:]]*(//|/\*|\*)' "${file}") ;;
    *.py | *.sh | *.toml | *.cmake | */CMakeLists.txt) subject=$(grep -nE '^[[:space:]]*#' "${file}") ;;
    *) exit 0 ;;
esac

[ -n "${subject}" ] || exit 0

# "robust" and "harness" are on the skill's banned list but are legitimate here
# (robust estimator, wiring harness, test harness), so they are omitted.
words='delve|delving|foster|fosters|leverage|leverages|leveraging|utilize|utilizes|utilizing'
words="${words}|facilitate|facilitates|empower|empowers|streamline|streamlines|cutting-edge"
words="${words}|paradigm shift|game changer|tapestry|realm|beacon|multifaceted|meticulous"
words="${words}|meticulously|intricate|intricacies|paramount|transformative|elevates?|embark"
words="${words}|supercharge|ever-evolving"

phrases="it's worth noting|it is worth noting|it's important to note|it is important to note"
phrases="${phrases}|at the end of the day|at its core|in today's world|in the age of|the reality is"
phrases="${phrases}|going forward|in conclusion|let's dive in|stands as a testament"
phrases="${phrases}|plays a vital role|underscores|showcasing|marks a pivotal moment"
phrases="${phrases}|solidifies its position|widely regarded as|experts agree|studies show"

hits=$(grep -iE "\b(${words})\b|(${phrases})" <<<"${subject}")

[ -n "${hits}" ] || exit 0

{
    echo "no-ai-slop: banned wording in ${file}"
    echo "${hits}"
    echo "Rewrite these lines per the writing rules in CLAUDE.md, then continue."
} >&2
exit 2
