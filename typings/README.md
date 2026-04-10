# Typings For BadgeBot

This folder contains local `.pyi` stub files used by Visual Studio Code (Pylance)
when developing BadgeBot on desktop Python.

## Why this folder exists

BadgeBot targets BadgeOS/MicroPython. Many imports used by the app (for example
`machine`, `ota`, `system.*`, `tildagonos`) are available on badge hardware but
not in a normal desktop Python environment.

Without stubs, editors report unresolved imports and missing members even when
runtime behavior on the badge is correct.

## How this interacts with VS Code, Pylance, and pylint

- VS Code + Pylance:
  - Configured in `.vscode/settings.json` with
    `"python.analysis.stubPath": "typings"`.
  - Pylance reads these stubs for symbol resolution, auto-complete, and type
    checking.

- pylint:
  - Configured in `pyproject.toml`.
  - pylint does not rely on these stubs in the same way Pylance does.
  - We handle desktop-only false positives using:
    - `[tool.pylint.master].ignored-modules` for runtime-only imports
    - `[tool.pylint.typecheck].generated-members` for runtime-added members

## What belongs in this folder

Add minimal, stable API surface only:

- Imported module names and package structure
- Classes/functions/constants used by project code
- Basic method signatures where useful

Avoid full re-implementation of runtime libraries. Keep stubs lightweight and
focused on editor/linter correctness.

## When to update stubs

Update this folder when:

- A new runtime-only module is imported in project code
- Existing project code starts using additional attributes/methods on a runtime
  object
- BadgeOS/MicroPython API changes break editor diagnostics

## Notes

- `.pyi` files are for development-time tooling only and are not deployed to the
  badge.
- Prefer keeping behavior-neutral stubs (no executable logic).
