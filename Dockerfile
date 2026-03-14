# Tritium-SC Command Center — Production Docker Image
# Copyright 2026 Valpatel Software LLC — AGPL-3.0

FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONPATH=/app/src

WORKDIR /app

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc libffi-dev && \
    rm -rf /var/lib/apt/lists/*

# Install tritium-lib from submodule (shared models, events, auth)
COPY tritium-lib/ /tmp/tritium-lib/
RUN pip install --no-cache-dir /tmp/tritium-lib/ && \
    rm -rf /tmp/tritium-lib/

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY src/ src/
COPY assets/ assets/

# Create data directories
RUN mkdir -p data/amy data/recordings data/test_reports data/dossiers

EXPOSE 8000

# Health check against the comprehensive health endpoint
HEALTHCHECK --interval=30s --timeout=5s --start-period=15s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/api/health')" || exit 1

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "1"]
