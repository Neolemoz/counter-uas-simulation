# Scenario Infrastructure Plan

## Objective

Create a planning foundation for reproducible scenario packs that support
runtime realism, tactical robustness, and evaluation repeatability.

## Planning guardrails

- scenario metadata should remain compatible with replay and evaluation
- scenario packs should be deterministic under fixed seeds where practical
- avoid coupling scenario identity to unstable log wording

## Workstreams

### 1. Scenario pack structure

- reusable scenario packs
- stable scenario identifiers
- manifest-linked replay metadata
- deterministic scenario seed capture

### 2. Stress categories

- delayed detection
- stale tracking
- urban occlusion-like visibility breaks
- swarm and saturation conditions
- intermittent false positives
- maneuver-heavy targets

### 3. Difficulty grading

- baseline
- moderate stress
- severe stress
- adversarial edge-case

Grading here is planning-only and should later map to measurable scenario properties.

### 4. Scenario metadata

- deterministic seed fields
- realism knobs in manifest form
- supervisor/safety stress tags
- expected evaluation families

## Roadmap skeleton

### Near-term

- define scenario pack layout and metadata fields
- define stress taxonomy and grading criteria
- align scenario manifests with replay and evaluation metadata

### Mid-term

- build deterministic reference scenarios
- build realism stress packs
- build supervisor/failsafe stress packs

## Explicit non-goals

- production mission planning
- operator scenario authoring UX
- distributed scenario orchestration
