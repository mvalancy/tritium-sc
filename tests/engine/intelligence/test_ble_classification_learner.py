# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for BLE Classification Learner."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "src"))

from engine.intelligence.ble_classification_learner import (
    BLEClassificationLearner,
    DeviceClassifierMLBackend,
    extract_features,
    FEATURE_NAMES,
    DEVICE_TYPES,
)


def _check_sklearn_available():
    try:
        import sklearn  # noqa: F401
        return True
    except ImportError:
        return False


class TestExtractFeatures:
    """Test feature extraction from BLE advertisement data."""

    def test_empty_input(self):
        features = extract_features()
        assert len(features) == len(FEATURE_NAMES)
        assert all(isinstance(f, float) for f in features)

    def test_full_input(self):
        features = extract_features(
            mac="AC:BC:32:AA:BB:CC",
            name="iPhone 15",
            service_uuids=["180F", "180D"],
            appearance=192,
            company_id=76,
            rssi=-45,
            is_random_mac=False,
        )
        assert len(features) == len(FEATURE_NAMES)
        assert features[1] == 9.0  # name_length ("iPhone 15")
        assert features[4] == 2.0  # uuid_count
        assert features[5] == 1.0  # has_heart_rate (180D)
        assert features[6] == 1.0  # has_battery (180F)
        assert features[11] == -45.0  # rssi
        assert features[12] == 0.0  # is_random_mac

    def test_random_mac_flag(self):
        features = extract_features(is_random_mac=True)
        assert features[12] == 1.0

    def test_name_ratios(self):
        features = extract_features(name="ABC123")
        assert features[2] == pytest.approx(0.5, abs=0.01)  # alpha_ratio
        assert features[3] == pytest.approx(0.5, abs=0.01)  # digit_ratio

    def test_hid_uuid(self):
        features = extract_features(service_uuids=["1812"])
        assert features[7] == 1.0  # has_hid

    def test_audio_uuid(self):
        features = extract_features(service_uuids=["110a"])
        assert features[8] == 1.0  # has_audio


class TestBLEClassificationLearner:
    """Test the BLE classification learner."""

    def test_init_no_model(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))
        assert not learner.is_trained
        assert learner.accuracy == 0.0
        assert learner.training_count == 0

    def test_add_training_example(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))
        learner.add_training_example(
            label="phone",
            mac="AC:BC:32:AA:BB:CC",
            name="iPhone 15",
        )
        assert len(learner._training_data) == 1

    def test_train_insufficient_data(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))
        learner.add_training_example(label="phone", name="iPhone")
        result = learner.train()
        assert not result["success"]
        assert "Insufficient" in result["error"]

    def test_predict_untrained(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))
        device_type, confidence = learner.predict(name="iPhone 15")
        assert device_type == "unknown"
        assert confidence == 0.0

    @pytest.mark.skipif(
        not _check_sklearn_available(),
        reason="scikit-learn not installed",
    )
    def test_train_and_predict(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))

        # Add training data — 2 classes, 10+ examples each
        for i in range(15):
            learner.add_training_example(
                label="phone",
                mac=f"AC:BC:32:AA:BB:{i:02X}",
                name=f"iPhone {i}",
                service_uuids=["180F"],
                rssi=-50 + i,
            )
            learner.add_training_example(
                label="watch",
                mac=f"DC:A6:32:AA:BB:{i:02X}",
                name=f"Apple Watch {i}",
                service_uuids=["180D"],
                appearance=192,
                rssi=-70 + i,
            )

        result = learner.train()
        assert result["success"]
        assert result["training_count"] == 30
        assert len(result["classes"]) == 2
        assert learner.is_trained

        # Predict
        device_type, confidence = learner.predict(
            name="iPhone 16",
            mac="AC:BC:32:AA:BB:FF",
            service_uuids=["180F"],
            rssi=-48,
        )
        assert device_type in ("phone", "watch")
        assert confidence > 0.0

    @pytest.mark.skipif(
        not _check_sklearn_available(),
        reason="scikit-learn not installed",
    )
    def test_save_and_load(self, tmp_path):
        model_path = str(tmp_path / "model.pkl")
        learner = BLEClassificationLearner(model_path=model_path)

        for i in range(12):
            learner.add_training_example(label="phone", name=f"Phone{i}", rssi=-50)
            learner.add_training_example(label="speaker", name=f"JBL{i}", rssi=-70)

        result = learner.train()
        assert result["success"]

        # Load in a new instance
        learner2 = BLEClassificationLearner(model_path=model_path)
        assert learner2.is_trained
        assert learner2.training_count == learner.training_count

    def test_get_status(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))
        status = learner.get_status()
        assert "trained" in status
        assert "feature_names" in status
        assert "device_types" in status
        assert status["trained"] is False


class TestDeviceClassifierMLBackend:
    """Test the ML backend adapter."""

    def test_not_trained_returns_none(self, tmp_path):
        learner = BLEClassificationLearner(model_path=str(tmp_path / "model.pkl"))
        backend = DeviceClassifierMLBackend(learner)
        result = backend.classify(name="iPhone 15")
        assert result is None
