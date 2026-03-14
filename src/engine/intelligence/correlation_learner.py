# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""RL-based correlation scorer — learns from TrainingStore data.

Loads training examples from the TrainingStore SQLite database. Trains
a logistic regression model (scikit-learn) on correlation features:
distance, RSSI delta, co-movement, device type match, time gap, signal
pattern. Falls back to static weights if no training data or if sklearn
is not installed.

Integrates with TargetCorrelator as a LearnedStrategy that wraps the
LearnedScorer from tritium-lib.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger("correlation_learner")

# Feature names must match what the correlator logs to TrainingStore
FEATURE_NAMES = [
    "distance",
    "rssi_delta",
    "co_movement",
    "device_type_match",
    "time_gap",
    "signal_pattern",
]

MODEL_PATH = "data/models/correlation_model.pkl"


class CorrelationLearner:
    """Trains and manages a correlation scoring model.

    Loads data from TrainingStore, trains a logistic regression, and
    provides predictions through the tritium-lib CorrelationScorer
    interface.
    """

    def __init__(
        self,
        training_store: Any = None,
        model_path: str = MODEL_PATH,
        feature_names: Optional[list[str]] = None,
    ) -> None:
        self._training_store = training_store
        self._model_path = model_path
        self._feature_names = feature_names or list(FEATURE_NAMES)
        self._model: Any = None
        self._accuracy: float = 0.0
        self._training_count: int = 0
        self._last_trained: float = 0.0
        self._sklearn_available = _check_sklearn()

        # Try to load existing model
        self._load_model()

    @property
    def is_trained(self) -> bool:
        return self._model is not None

    @property
    def accuracy(self) -> float:
        return self._accuracy

    @property
    def training_count(self) -> int:
        return self._training_count

    @property
    def last_trained(self) -> float:
        return self._last_trained

    def get_status(self) -> dict[str, Any]:
        """Return model status for API response."""
        return {
            "trained": self.is_trained,
            "accuracy": self._accuracy,
            "training_count": self._training_count,
            "last_trained": self._last_trained,
            "last_trained_iso": (
                time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(self._last_trained))
                if self._last_trained > 0
                else None
            ),
            "sklearn_available": self._sklearn_available,
            "model_path": self._model_path,
            "feature_names": self._feature_names,
        }

    def train(self) -> dict[str, Any]:
        """Train (or retrain) the model from TrainingStore data.

        Returns:
            Dict with training results: accuracy, count, success status.
        """
        if self._training_store is None:
            return {"success": False, "error": "No training store configured"}

        if not self._sklearn_available:
            return {"success": False, "error": "scikit-learn not available, using static weights"}

        try:
            # Get confirmed correlation decisions
            data = self._training_store.get_correlation_data(
                limit=10000,
                outcome_only=True,
            )

            if len(data) < 10:
                return {
                    "success": False,
                    "error": f"Insufficient training data: {len(data)} examples (need 10+)",
                    "count": len(data),
                }

            # Extract features and labels
            X, y = self._prepare_training_data(data)

            if len(X) < 10:
                return {
                    "success": False,
                    "error": f"Insufficient valid examples after filtering: {len(X)}",
                }

            # Train logistic regression
            from sklearn.linear_model import LogisticRegression
            from sklearn.model_selection import cross_val_score
            import numpy as np

            X_arr = np.array(X)
            y_arr = np.array(y)

            model = LogisticRegression(
                max_iter=1000,
                C=1.0,
                class_weight="balanced",
            )

            # Cross-validation for accuracy estimate
            if len(X) >= 20:
                cv_folds = min(5, len(X) // 4)
                scores = cross_val_score(model, X_arr, y_arr, cv=cv_folds, scoring="accuracy")
                accuracy = float(scores.mean())
            else:
                accuracy = 0.0  # Not enough data for CV

            # Train on full dataset
            model.fit(X_arr, y_arr)

            self._model = model
            self._accuracy = accuracy
            self._training_count = len(X)
            self._last_trained = time.time()

            # Save model
            self._save_model()

            logger.info(
                "Correlation model trained: accuracy=%.3f, n=%d",
                accuracy, len(X),
            )

            return {
                "success": True,
                "accuracy": accuracy,
                "training_count": len(X),
                "feature_names": self._feature_names,
            }

        except Exception as exc:
            logger.error("Model training failed: %s", exc)
            return {"success": False, "error": str(exc)}

    def predict(self, features: dict[str, float]) -> tuple[float, float]:
        """Predict correlation probability.

        Args:
            features: Feature dict with keys from FEATURE_NAMES.

        Returns:
            (probability, confidence) tuple.
        """
        if self._model is None:
            return _static_predict(features)

        try:
            import numpy as np

            X = [[features.get(fn, 0.0) for fn in self._feature_names]]
            proba = self._model.predict_proba(np.array(X))[0]
            probability = float(proba[1]) if len(proba) > 1 else float(proba[0])
            confidence = min(1.0, abs(probability - 0.5) * 2.0)
            return probability, confidence
        except Exception:
            return _static_predict(features)

    def get_scorer(self) -> Any:
        """Return a tritium-lib CorrelationScorer wrapping this model.

        Returns:
            LearnedScorer or StaticScorer from tritium-lib.
        """
        try:
            from tritium_lib.intelligence.scorer import LearnedScorer, StaticScorer

            if self._model is not None:
                return LearnedScorer(
                    model=self._model,
                    feature_names=self._feature_names,
                    accuracy=self._accuracy,
                    training_count=self._training_count,
                )
            return StaticScorer()
        except ImportError:
            return None

    def _prepare_training_data(
        self, data: list[dict[str, Any]]
    ) -> tuple[list[list[float]], list[int]]:
        """Extract feature vectors and labels from training records."""
        X: list[list[float]] = []
        y: list[int] = []

        for record in data:
            outcome = record.get("outcome", "")
            if outcome not in ("correct", "incorrect"):
                continue

            features = record.get("features", {})
            if not isinstance(features, dict):
                continue

            row = [float(features.get(fn, 0.0)) for fn in self._feature_names]
            label = 1 if outcome == "correct" else 0

            X.append(row)
            y.append(label)

        return X, y

    def _load_model(self) -> bool:
        """Attempt to load a previously saved model."""
        try:
            import pickle

            p = Path(self._model_path)
            if not p.exists():
                return False

            with open(p, "rb") as f:
                data = pickle.load(f)

            self._model = data.get("model")
            self._feature_names = data.get("feature_names", self._feature_names)
            self._accuracy = data.get("accuracy", 0.0)
            self._training_count = data.get("training_count", 0)
            self._last_trained = data.get("last_trained", 0.0)

            logger.info(
                "Loaded correlation model: accuracy=%.3f, n=%d",
                self._accuracy, self._training_count,
            )
            return True
        except Exception as exc:
            logger.debug("No existing model to load: %s", exc)
            return False

    def _save_model(self) -> bool:
        """Save the current model to disk."""
        try:
            import pickle

            p = Path(self._model_path)
            p.parent.mkdir(parents=True, exist_ok=True)

            data = {
                "model": self._model,
                "feature_names": self._feature_names,
                "accuracy": self._accuracy,
                "training_count": self._training_count,
                "last_trained": self._last_trained,
            }
            with open(p, "wb") as f:
                pickle.dump(data, f)
            return True
        except Exception as exc:
            logger.warning("Failed to save model: %s", exc)
            return False


class LearnedStrategy:
    """Correlation strategy adapter for the TargetCorrelator.

    Wraps a CorrelationLearner as a CorrelationStrategy compatible
    with the existing multi-strategy correlator framework.
    """

    def __init__(self, learner: CorrelationLearner) -> None:
        self._learner = learner

    @property
    def name(self) -> str:
        return "learned"

    def evaluate(self, target_a: Any, target_b: Any) -> Any:
        """Evaluate correlation using the learned model.

        Extracts features from the target pair and runs prediction.
        Returns a StrategyScore compatible with the correlator.
        """
        import math

        try:
            from engine.tactical.correlation_strategies import StrategyScore
        except ImportError:
            # Fallback dataclass if import fails
            from dataclasses import dataclass

            @dataclass
            class StrategyScore:
                strategy_name: str
                score: float
                detail: str

        # Extract features from target pair
        features = _extract_features(target_a, target_b)

        probability, confidence = self._learner.predict(features)

        detail = (
            f"learned model p={probability:.3f} conf={confidence:.3f}"
            if self._learner.is_trained
            else f"static fallback p={probability:.3f}"
        )

        return StrategyScore(
            strategy_name=self.name,
            score=max(0.0, min(1.0, probability)),
            detail=detail,
        )


def _extract_features(target_a: Any, target_b: Any) -> dict[str, float]:
    """Extract correlation features from a target pair.

    Works with TrackedTarget objects or any object with position,
    rssi, source, asset_type, and last_seen attributes.
    """
    import math

    features: dict[str, float] = {}

    # Distance
    try:
        pos_a = getattr(target_a, "position", (0.0, 0.0))
        pos_b = getattr(target_b, "position", (0.0, 0.0))
        dx = pos_a[0] - pos_b[0]
        dy = pos_a[1] - pos_b[1]
        features["distance"] = math.hypot(dx, dy)
    except (TypeError, IndexError):
        features["distance"] = 0.0

    # RSSI delta
    rssi_a = getattr(target_a, "rssi", 0)
    rssi_b = getattr(target_b, "rssi", 0)
    features["rssi_delta"] = abs(float(rssi_a or 0) - float(rssi_b or 0))

    # Co-movement (placeholder — requires history analysis)
    features["co_movement"] = 0.0

    # Device type match
    type_a = getattr(target_a, "asset_type", "unknown")
    type_b = getattr(target_b, "asset_type", "unknown")
    # Cross-sensor type match is valuable (BLE phone + camera person)
    source_a = getattr(target_a, "source", "")
    source_b = getattr(target_b, "source", "")
    if source_a != source_b:
        features["device_type_match"] = 1.0
    elif type_a == type_b and type_a != "unknown":
        features["device_type_match"] = 0.5
    else:
        features["device_type_match"] = 0.0

    # Time gap
    try:
        last_a = getattr(target_a, "last_seen", 0.0)
        last_b = getattr(target_b, "last_seen", 0.0)
        features["time_gap"] = abs(float(last_a) - float(last_b))
    except (TypeError, ValueError):
        features["time_gap"] = 0.0

    # Signal pattern (1.0 if both seen very recently, decays)
    try:
        now = time.monotonic()
        age_a = now - float(getattr(target_a, "last_seen", now))
        age_b = now - float(getattr(target_b, "last_seen", now))
        max_age = max(age_a, age_b)
        features["signal_pattern"] = max(0.0, 1.0 - max_age / 30.0)
    except (TypeError, ValueError):
        features["signal_pattern"] = 0.0

    return features


def _static_predict(features: dict[str, float]) -> tuple[float, float]:
    """Static weighted prediction fallback (no sklearn needed)."""
    import math

    weights = {
        "distance": -0.30,
        "rssi_delta": -0.10,
        "co_movement": 0.25,
        "device_type_match": 0.15,
        "time_gap": -0.10,
        "signal_pattern": 0.20,
    }
    bias = 0.5

    logit = 0.0
    for fn, w in weights.items():
        logit += w * features.get(fn, 0.0)

    # Sigmoid
    x = logit + (bias - 0.5) * 2.0
    if x >= 0:
        probability = 1.0 / (1.0 + math.exp(-x))
    else:
        ez = math.exp(x)
        probability = ez / (1.0 + ez)

    confidence = min(1.0, abs(probability - 0.5) * 2.0)
    return probability, confidence


def _check_sklearn() -> bool:
    """Check if scikit-learn is available."""
    try:
        import sklearn  # noqa: F401
        return True
    except ImportError:
        return False


# Singleton learner
_learner: Optional[CorrelationLearner] = None


def get_correlation_learner(
    training_store: Any = None,
    model_path: str = MODEL_PATH,
) -> CorrelationLearner:
    """Get or create the singleton CorrelationLearner."""
    global _learner
    if _learner is None:
        if training_store is None:
            try:
                from engine.intelligence.training_store import get_training_store
                training_store = get_training_store()
            except ImportError:
                pass
        _learner = CorrelationLearner(
            training_store=training_store,
            model_path=model_path,
        )
    return _learner
