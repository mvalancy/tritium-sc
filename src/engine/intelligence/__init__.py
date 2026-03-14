# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Intelligence subsystem — self-improving correlation and classification.

Provides:
- TrainingStore: SQLite-backed ML training data collection
- CorrelationLearner: Trains logistic regression from operator feedback
- LearnedStrategy: Adapter for TargetCorrelator integration
"""
