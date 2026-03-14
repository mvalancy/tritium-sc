# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""BLE Classification Learner — ML-based device type classification.

Trains a multi-class classifier on labeled BLE advertisement features:
  - OUI prefix (hashed to int)
  - Name pattern length and char distribution
  - Service UUID count and common UUID flags
  - Appearance code (16-bit)
  - Company ID (16-bit)
  - RSSI (signal strength)
  - Random MAC flag

Label: device type (phone, watch, laptop, earbuds, speaker, IoT, vehicle, etc.)

Integrates with tritium-lib's DeviceClassifier as an optional ML backend
that supplements rule-based classification with data-driven predictions.
"""
from __future__ import annotations

import hashlib
import logging
import pickle
import threading
import time
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger("ble_classification_learner")

# Feature extraction keys
FEATURE_NAMES = [
    "oui_hash",         # Hash of first 3 bytes of MAC (OUI prefix)
    "name_length",      # Length of device name (0 if absent)
    "name_alpha_ratio", # Ratio of alphabetic chars in name
    "name_digit_ratio", # Ratio of digit chars in name
    "uuid_count",       # Number of advertised service UUIDs
    "has_heart_rate",   # 1.0 if heart rate UUID present
    "has_battery",      # 1.0 if battery UUID present
    "has_hid",          # 1.0 if HID UUID present
    "has_audio",        # 1.0 if audio sink/source UUID present
    "appearance",       # GAP appearance value (0 if absent)
    "company_id",       # BLE company ID (0 if absent)
    "rssi",             # Signal strength
    "is_random_mac",    # 1.0 if random MAC
]

# Common service UUID prefixes (16-bit standard UUIDs)
HEART_RATE_UUID = "180d"
BATTERY_UUID = "180f"
HID_UUID = "1812"
AUDIO_UUIDS = {"110a", "110b", "110d", "1108", "111e"}  # A2DP sink/src/etc

# Known device type labels
DEVICE_TYPES = [
    "phone", "watch", "laptop", "tablet", "earbuds", "headphones",
    "speaker", "smart_speaker", "tag", "fitness", "camera", "gamepad",
    "smart_home", "microcontroller", "vehicle", "vr_headset",
    "printer", "media_player", "unknown",
]

MODEL_PATH = "data/models/ble_classifier_model.pkl"


def extract_features(
    mac: str = "",
    name: str = "",
    service_uuids: list[str] | None = None,
    appearance: int | None = None,
    company_id: int | None = None,
    rssi: int = -100,
    is_random_mac: bool = False,
) -> list[float]:
    """Extract numeric feature vector from BLE advertisement data.

    Returns a list of floats matching FEATURE_NAMES order.
    """
    features: list[float] = []

    # OUI hash — hash the first 3 octets of MAC into a stable int
    oui = mac.upper().replace("-", ":").replace(".", ":")[:8]
    oui_hash = int(hashlib.md5(oui.encode()).hexdigest()[:8], 16) / 0xFFFFFFFF if oui else 0.0
    features.append(oui_hash)

    # Name features
    features.append(float(len(name)))
    if name:
        alpha_count = sum(1 for c in name if c.isalpha())
        digit_count = sum(1 for c in name if c.isdigit())
        features.append(alpha_count / len(name))
        features.append(digit_count / len(name))
    else:
        features.append(0.0)
        features.append(0.0)

    # Service UUID features
    uuids = service_uuids or []
    uuid_lower = [u.lower().replace("0x", "") for u in uuids]
    features.append(float(len(uuids)))
    features.append(1.0 if HEART_RATE_UUID in uuid_lower else 0.0)
    features.append(1.0 if BATTERY_UUID in uuid_lower else 0.0)
    features.append(1.0 if HID_UUID in uuid_lower else 0.0)
    features.append(1.0 if any(u in AUDIO_UUIDS for u in uuid_lower) else 0.0)

    # Appearance and company ID
    features.append(float(appearance or 0))
    features.append(float(company_id or 0))

    # RSSI
    features.append(float(rssi))

    # Random MAC
    features.append(1.0 if is_random_mac else 0.0)

    return features


class BLEClassificationLearner:
    """Trains and manages a BLE device type classification model.

    Uses labeled BLE advertisement data to train a multi-class classifier
    (Random Forest or Logistic Regression depending on data volume).
    Provides predictions that supplement the rule-based DeviceClassifier.
    """

    def __init__(
        self,
        model_path: str = MODEL_PATH,
        training_data: list[dict[str, Any]] | None = None,
    ) -> None:
        self._model_path = model_path
        self._model: Any = None
        self._label_encoder: Any = None
        self._accuracy: float = 0.0
        self._training_count: int = 0
        self._last_trained: float = 0.0
        self._sklearn_available = _check_sklearn()
        self._training_data: list[dict[str, Any]] = training_data or []
        self._lock = threading.Lock()

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
            "feature_names": FEATURE_NAMES,
            "device_types": DEVICE_TYPES,
        }

    def add_training_example(
        self,
        label: str,
        mac: str = "",
        name: str = "",
        service_uuids: list[str] | None = None,
        appearance: int | None = None,
        company_id: int | None = None,
        rssi: int = -100,
        is_random_mac: bool = False,
    ) -> None:
        """Add a labeled training example.

        Args:
            label: Device type label (e.g. "phone", "watch").
            mac: Device MAC address.
            name: Advertised device name.
            service_uuids: Advertised service UUIDs.
            appearance: GAP appearance value.
            company_id: BLE company identifier.
            rssi: Signal strength.
            is_random_mac: Whether MAC is random.
        """
        example = {
            "label": label,
            "mac": mac,
            "name": name,
            "service_uuids": service_uuids or [],
            "appearance": appearance,
            "company_id": company_id,
            "rssi": rssi,
            "is_random_mac": is_random_mac,
        }
        with self._lock:
            self._training_data.append(example)

    def train(self) -> dict[str, Any]:
        """Train the classification model from accumulated training data.

        Returns:
            Dict with training results.
        """
        if not self._sklearn_available:
            return {"success": False, "error": "scikit-learn not available"}

        with self._lock:
            data = list(self._training_data)

        if len(data) < 10:
            return {
                "success": False,
                "error": f"Insufficient training data: {len(data)} examples (need 10+)",
                "count": len(data),
            }

        try:
            import numpy as np
            from sklearn.ensemble import RandomForestClassifier
            from sklearn.model_selection import cross_val_score
            from sklearn.preprocessing import LabelEncoder

            # Extract features and labels
            X: list[list[float]] = []
            y_raw: list[str] = []

            for example in data:
                features = extract_features(
                    mac=example.get("mac", ""),
                    name=example.get("name", ""),
                    service_uuids=example.get("service_uuids"),
                    appearance=example.get("appearance"),
                    company_id=example.get("company_id"),
                    rssi=example.get("rssi", -100),
                    is_random_mac=example.get("is_random_mac", False),
                )
                X.append(features)
                y_raw.append(example.get("label", "unknown"))

            X_arr = np.array(X)
            le = LabelEncoder()
            y_arr = le.fit_transform(y_raw)

            # Need at least 2 classes
            if len(le.classes_) < 2:
                return {
                    "success": False,
                    "error": f"Need at least 2 classes, got {len(le.classes_)}",
                }

            model = RandomForestClassifier(
                n_estimators=50,
                max_depth=10,
                class_weight="balanced",
                random_state=42,
                n_jobs=1,
            )

            # Cross-validation
            accuracy = 0.0
            if len(X) >= 20:
                cv_folds = min(5, len(X) // 4)
                # Ensure each fold has all classes
                try:
                    scores = cross_val_score(model, X_arr, y_arr, cv=cv_folds, scoring="accuracy")
                    accuracy = float(scores.mean())
                except ValueError:
                    accuracy = 0.0

            # Train on full dataset
            model.fit(X_arr, y_arr)

            self._model = model
            self._label_encoder = le
            self._accuracy = accuracy
            self._training_count = len(X)
            self._last_trained = time.time()

            # Save model
            self._save_model()

            logger.info(
                "BLE classifier trained: accuracy=%.3f, n=%d, classes=%d",
                accuracy, len(X), len(le.classes_),
            )

            return {
                "success": True,
                "accuracy": accuracy,
                "training_count": len(X),
                "classes": list(le.classes_),
            }

        except Exception as exc:
            logger.error("BLE classifier training failed: %s", exc)
            return {"success": False, "error": str(exc)}

    def predict(
        self,
        mac: str = "",
        name: str = "",
        service_uuids: list[str] | None = None,
        appearance: int | None = None,
        company_id: int | None = None,
        rssi: int = -100,
        is_random_mac: bool = False,
    ) -> tuple[str, float]:
        """Predict device type from BLE advertisement data.

        Returns:
            (device_type, confidence) tuple.
        """
        if self._model is None or self._label_encoder is None:
            return "unknown", 0.0

        try:
            import numpy as np

            features = extract_features(
                mac=mac,
                name=name,
                service_uuids=service_uuids,
                appearance=appearance,
                company_id=company_id,
                rssi=rssi,
                is_random_mac=is_random_mac,
            )

            X = np.array([features])
            proba = self._model.predict_proba(X)[0]
            predicted_idx = int(np.argmax(proba))
            confidence = float(proba[predicted_idx])
            device_type = self._label_encoder.inverse_transform([predicted_idx])[0]

            return str(device_type), confidence

        except Exception as exc:
            logger.debug("BLE classifier prediction failed: %s", exc)
            return "unknown", 0.0

    def _load_model(self) -> bool:
        """Load a previously saved model."""
        try:
            p = Path(self._model_path)
            if not p.exists():
                return False

            with open(p, "rb") as f:
                data = pickle.load(f)

            self._model = data.get("model")
            self._label_encoder = data.get("label_encoder")
            self._accuracy = data.get("accuracy", 0.0)
            self._training_count = data.get("training_count", 0)
            self._last_trained = data.get("last_trained", 0.0)

            logger.info(
                "Loaded BLE classifier: accuracy=%.3f, n=%d",
                self._accuracy, self._training_count,
            )
            return True
        except Exception as exc:
            logger.debug("No existing BLE classifier to load: %s", exc)
            return False

    def _save_model(self) -> bool:
        """Save the current model to disk."""
        try:
            p = Path(self._model_path)
            p.parent.mkdir(parents=True, exist_ok=True)

            data = {
                "model": self._model,
                "label_encoder": self._label_encoder,
                "accuracy": self._accuracy,
                "training_count": self._training_count,
                "last_trained": self._last_trained,
            }
            with open(p, "wb") as f:
                pickle.dump(data, f)
            return True
        except Exception as exc:
            logger.warning("Failed to save BLE classifier: %s", exc)
            return False


class DeviceClassifierMLBackend:
    """Adapter that integrates BLEClassificationLearner with DeviceClassifier.

    When the ML model is trained, it acts as an additional signal source
    in the DeviceClassifier's multi-signal voting system.
    """

    def __init__(self, learner: BLEClassificationLearner) -> None:
        self._learner = learner

    def classify(
        self,
        mac: str = "",
        name: str = "",
        service_uuids: list[str] | None = None,
        appearance: int | None = None,
        company_id: int | None = None,
        rssi: int = -100,
        is_random_mac: bool = False,
    ) -> dict[str, Any] | None:
        """Return an ML-based classification signal for DeviceClassifier.

        Returns a signal dict compatible with DeviceClassifier's voting
        system, or None if no prediction available.
        """
        if not self._learner.is_trained:
            return None

        device_type, confidence = self._learner.predict(
            mac=mac,
            name=name,
            service_uuids=service_uuids,
            appearance=appearance,
            company_id=company_id,
            rssi=rssi,
            is_random_mac=is_random_mac,
        )

        if device_type == "unknown" or confidence < 0.3:
            return None

        return {
            "signal": "ml_classifier",
            "device_type": device_type,
            "confidence": confidence,
            "method": "random_forest",
            "training_count": self._learner.training_count,
            "model_accuracy": self._learner.accuracy,
        }


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------

_learner: Optional[BLEClassificationLearner] = None


def get_ble_classification_learner(
    model_path: str = MODEL_PATH,
) -> BLEClassificationLearner:
    """Get or create the singleton BLEClassificationLearner."""
    global _learner
    if _learner is None:
        _learner = BLEClassificationLearner(model_path=model_path)
    return _learner


def _check_sklearn() -> bool:
    """Check if scikit-learn is available."""
    try:
        import sklearn  # noqa: F401
        return True
    except ImportError:
        return False
