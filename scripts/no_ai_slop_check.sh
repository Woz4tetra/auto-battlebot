#!/bin/bash

set -u


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

check_file() {
    local file=$1 subject hits
    case "${file}" in
        */CLAUDE.md | CLAUDE.md | */.claude/* | .claude/* | */data/* | data/* | */venv/* | venv/* | */build*/* | build*/*) return 0 ;;
    esac
    case "${file}" in
        *.md | *.txt | *.rst) subject=$(grep -n '' "${file}") ;;
        *.cpp | *.hpp | *.h | *.cc | *.cu | *.cuh) subject=$(grep -nE '^[[:space:]]*(//|/\*|\*)' "${file}") ;;
        *.py | *.sh | *.toml | *.cmake | */CMakeLists.txt | CMakeLists.txt) subject=$(grep -nE '^[[:space:]]*#' "${file}") ;;
        *) return 0 ;;
    esac
    [ -n "${subject}" ] || return 0
    hits=$(grep -iE "\b(${words})\b|(${phrases})" <<<"${subject}")
    [ -n "${hits}" ] || return 0
    echo "no-ai-slop: banned wording in ${file}"
    echo "${hits}"
    return 1
}

if [ $# -gt 0 ]; then
    files=("$@")
    for file in "${files[@]}"; do
        [ -f "${file}" ] || { echo "no-ai-slop: no such file: ${file}" >&2; exit 2; }
    done
else
    if [ -t 0 ]; then
        echo "usage: $0 FILE... (or a PostToolUse hook payload on stdin)" >&2
        exit 2
    fi
    payload=$(cat)
    if [ -z "${payload}" ]; then
        echo "no-ai-slop: no file arguments and an empty stdin payload, nothing checked" >&2
        exit 2
    fi
    file=$(jq -r '.tool_response.filePath // .tool_input.file_path // empty' <<<"${payload}" 2>/dev/null)
    [ -n "${file}" ] && [ -f "${file}" ] || exit 0
    files=("${file}")
fi

failed=0
for file in "${files[@]}"; do
    check_file "${file}" >&2 || failed=1
done

[ "${failed}" -eq 0 ] && exit 0
echo "Rewrite these lines per the writing rules in CLAUDE.md, then continue." >&2
exit 2
