# docs

Where a doc goes: "Docs layout" in the root `CLAUDE.md`.

## Writing rules

Applies to everything generated in this repo: reports, analysis write-ups,
commit messages, PR bodies, and docs. Adapted from the
`no-ai-slop` skill; `scripts/no_ai_slop_check.sh` enforces the wordlist
automatically on write, and the rest is on you.

Preserve the author's point and voice while making the writing clearer. Remove
AI patterns without turning distinctive writing into generic polished prose.

### Principles

- **Make the minimum effective edit.** Fix AI patterns, errors, repetition, and
  unclear passages. Leave strong sentences alone.
- **Lead with the point when the setup adds nothing.** Cut generic
  throat-clearing. Keep an aside or admission when it creates real context.
- **Keep the meaning.** Don't invent claims, examples, stats, or opinions. If
  something is unclear, ask.
- **Open it up, don't dumb it down.** Keep the substance, nuance, and precision.
  Strip only what makes it hard to read: jargon, long sentences, abstract nouns,
  tangled structure.
- **Use active voice.** "The team shipped it Tuesday" beats "the decision
  emerged." Never let inanimate things do human verbs.
- **Be concrete and specific.** "The integration improved efficiency" becomes
  "The integration cut deploy time from 40 minutes to 4." Names, numbers, dates,
  mechanisms, and examples beat abstractions.
- **Protect the specific fact.** Don't smooth a useful detail into generic
  importance. This matters most in eval and replay write-ups: report the metric
  and the delta, not that a change was significant.
- **Make verbs do the work.** "Made a decision" becomes "decided." "Has the
  ability to" becomes "can."
- **Make every sentence earn its place.** Cut empty qualifiers. Keep "I think"
  or "maybe" when they express real uncertainty.
- **Untangle sentences without flattening the cadence.** Split genuinely hard
  sentences. Keep fragments and changes in pace that are clear and characteristic.
- **Preserve useful edge.** Keep strong opinions, blunt language, humor, and
  honest admissions. Don't replace them with safer wording.

### Words to cut

Banned outright: delve, foster, leverage, utilize, facilitate, empower,
streamline, cutting-edge, paradigm shift, game changer, this is huge, this
changes everything, tapestry, realm, beacon, multifaceted, meticulous, intricate,
paramount, transformative, elevate, embark, supercharge, ever-evolving.

The upstream skill also bans "robust" and "harness"; both are legitimate
technical terms here (robust estimator, wiring harness, test harness) and are
exempt in this repo.

Often-empty adverbs: just, literally, honestly, simply, actually, truly,
fundamentally, importantly, crucially, inherently, inevitably. Cut them when they
add nothing. Keep them when they carry emphasis, uncertainty, or contrast.

Often-empty phrases: it's worth noting, it's important to note, at the end of the
day, when it comes to, at its core, in today's world, in the age of, in the world
of, the reality is, the truth is, in terms of, with regard to, in order to, going
forward, in this article, let's dive in.

### Patterns to cut

**Binary contrasts.** "This is not X. It's Y." / "The question isn't X, it's Y."
State Y directly. "The question isn't the model. It's the eval." becomes "The
eval matters more than the model."

**Throat-clearing openers.** "Here's the thing," "Let me be clear," "I'll be
honest," "The uncomfortable truth is." Cut them and state the point.

**Faux-insight setups.** "What most people get wrong," "Here's what nobody tells
you," "The part everyone misses." Cut the setup and let the claim stand alone.

**Colon reveals.** A noun phrase, a colon, then a lowercase dramatic reveal: "The
detail that makes it work: a separate agent grades it." Rewrite as a plain
sentence. Use colons for lists, labels, and quotes, not fake drama.

**Superficial analysis.** Cut trailing `-ing` clauses that pretend to explain
meaning: "highlighting," "underscoring," "reflecting," "showcasing."

**Importance puffery.** "Stands as a testament," "marks a pivotal moment," "plays
a vital role," "underscores its significance." State the fact and let the reader
judge whether it matters.

**Weasel attribution.** "Experts agree," "studies show," "widely regarded as."
Name the source or cut the claim. Never invent one. In this repo the source is
usually a specific run, recording, or commit -- cite it.

**Fake-strong verbs.** Prefer "is" and "has" when they are clearer. "The app
serves as a centralized hub for X" becomes "The app tracks X."

**Synonym cycling.** If the clear word is right, repeat it. Don't rotate terms
for style, and never rotate a term that names a component in this codebase.

**Negative listing.** "Not a X. Not a Y. A Z." Just say Z.

**Dramatic fragmentation.** "X. And Y. And Z." or "That's it. That's the whole
thing." Use complete sentences.

**Robotic rhythm.** Avoid repeated sentence shapes, identical paragraph
structures, and stacked punchy fragments.

**Rhetorical setups.** "What if I told you...", "Think about it:", "Plot twist:",
and self-answered "Question? Answer." pairs. Drop them and make the point.

**Fake-profound kickers.** Cut the final "deep" line that turns the point into a
metaphor or mic-drop. Don't rewrite it into a better metaphor -- delete it and
end on the clearest concrete sentence already present.

**Summary-recap endings.** "In conclusion," "Ultimately," "Overall," or a final
paragraph restating the piece. End on the last concrete point or next action.

**Formatting slop.** Emoji in headings, bold sprinkled mid-sentence, bullet lists
where two sentences of prose would read better, headers over two-sentence
sections. Format should follow the content, not decorate it.

**Em dashes.** Not a default rhythm crutch. None in short copy; 1-2 in longer
drafts if they clearly beat commas, periods, or parentheses.

