# Domain Docs

How the engineering skills should consume this repo's domain documentation
when exploring the codebase.

## Before exploring, read these

- **`CONTEXT.md`** at the repo root.
- **`docs/adr/`** for ADRs that touch the area being changed.

If either location does not exist, proceed silently. The producer skill creates
domain documentation lazily when terminology or architectural decisions are
resolved.

## File structure

This is a single-context repository:

```text
/
├── CONTEXT.md
├── docs/adr/
└── ucar_ws/
    └── src/
```

## Use the glossary's vocabulary

When output names a domain concept, use the term defined in `CONTEXT.md`. Do not
drift to synonyms the glossary explicitly avoids. If a needed concept is absent,
reconsider whether the terminology belongs to the project or note the gap for a
future domain-documentation session.

## Flag ADR conflicts

If proposed work contradicts an existing ADR, surface the conflict explicitly
instead of silently overriding the decision.
