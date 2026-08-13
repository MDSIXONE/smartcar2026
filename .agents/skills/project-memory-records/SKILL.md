---
name: project-memory-records
description: Maintain project-scoped AI change logs, mistake records, and failed-approach records. Use at the beginning of a new project conversation, before changing source code, after completing code changes, when an error yields a reusable lesson, or when considering, abandoning, or replacing an implementation approach.
---

# Project Memory Records

Maintain durable project learning records in `docs/ai-records/`. Create the folder and the three files below from their repository templates if they are absent. Do not fabricate historical events.

## Read Before Work

At the start of every new conversation, read:

1. `docs/ai-records/CHANGE_LOG.md` to understand work in progress and completed changes.
2. `docs/ai-records/MISTAKE_LOG.md` to avoid repeating known mistakes.

Before proposing or selecting a new implementation approach, also read `docs/ai-records/FAILED_APPROACHES.md`.

Follow repository-level instructions such as `AGENTS.md` after reading the records. If the task changes source code, satisfy any required sub-agent workflow before editing.

## Change Log

For every source-code change, update `CHANGE_LOG.md` twice when practical: create or mark the work unit `进行中` before implementation, then update it immediately after the change to `改动完成`. Use only those two status values. Record the date, goal, affected files, concise result, validation, and unresolved risks. Never add a third status such as “blocked” or “abandoned”; explain uncertainty in the notes instead.

Do not add an entry for documentation-only changes unless the repository explicitly requires it.

## Mistake Log

When an error, incorrect assumption, failed command, regression, or review finding produces a reusable lesson, append one concise entry to `MISTAKE_LOG.md`. Include the symptom, cause, prevention rule, and any related change-log entry. Record confirmed lessons, not routine experimentation or speculation.

## Failed Approaches

Append to `FAILED_APPROACHES.md` only when the user explicitly identifies an approach as failed, or the work must switch to a different approach. State the original approach, why it was stopped, evidence, and the replacement or follow-up. Do not record a failed approach merely because an attempt was imperfect or incomplete.

## Finish

Before handing off code work, confirm that the change log is updated, relevant reusable mistakes are captured, and any required failed-approach entry exists. Mention the record updates with the implementation summary.
