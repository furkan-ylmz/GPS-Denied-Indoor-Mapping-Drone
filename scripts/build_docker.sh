#!/bin/bash
# ─────────────────────────────────────────────────
# Docker imajını yeniden build et
# ─────────────────────────────────────────────────
set -e

echo "🔨 Docker imajı build ediliyor: drone_jazzy_env"
docker build -t drone_jazzy_env .
echo "✅ Build tamamlandı!"
