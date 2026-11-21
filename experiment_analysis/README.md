# Experiment analysis submission

This folder is ready for local review and a focused repository PR. No files have been pushed to GitHub.

## Quick start

```bash
python -m pip install -r requirements.txt
python scripts/analyze_reliability_results.py
```

Start with [the results report](results/final-analysis/REPORT.md), [reproduction instructions](docs/evaluation-reproduction.md), and [validation checks](results/final-analysis/validation.json).

## Suggested PR

**Title:** Reproduce reliability metrics and add evaluation figures

**Description:** Adds a secondary-analysis script for the existing 200-trial reliability dataset, with grouped summaries, noise-pair comparisons, four figures and reproduction instructions. Original false-stop labels and assigned-condition denominators are retained. Timing-boundary findings from the source report are explicitly attributed and are not presented as independently rescored.

**Validation:** Run the analysis script. Review `validation.json` for the full 20×10 matrix and comparison against the supplied group metrics. Review `provenance.json` for exact input/script hashes and runtime versions.

## What to submit

- `scripts/analyze_reliability_results.py`
- `docs/evaluation-reproduction.md`
- `requirements.txt` (merge its dependency into the existing project dependency file if appropriate)
- `results/final-analysis/`, including figures and validation/provenance

The `data/` copies provide portable local reproduction; reuse the repository's canonical dataset when available rather than committing redundant copies.

Original data collection and defense implementation belong to the team. This package is AI-assisted secondary-analysis work for Yang Mingzhi to review, understand and reproduce. It contains no newly collected trials.
